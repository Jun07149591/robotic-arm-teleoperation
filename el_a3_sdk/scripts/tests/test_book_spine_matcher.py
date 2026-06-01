import numpy as np
import cv2
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from el_a3_sdk.realsense.book_spine_matcher import (
    BookSpineMatchConfig,
    BookSpineMatcher,
    BookSpineMatchResult,
    draw_book_spine_overlay,
    evaluate_center_alignment,
    polygon_is_reasonable,
    polygon_to_bbox,
    polygon_to_roi,
    resize_keep_aspect,
    update_stable_state,
)


def make_template(seed: int = 7) -> np.ndarray:
    rng = np.random.default_rng(seed)
    image = np.full((220, 140, 3), 255, dtype=np.uint8)
    cv2.rectangle(image, (8, 8), (132, 212), (35, 35, 35), 3)
    cv2.line(image, (20, 15), (20, 205), (0, 0, 255), 3)
    cv2.line(image, (40, 15), (40, 205), (0, 255, 0), 3)
    cv2.line(image, (60, 15), (60, 205), (255, 0, 0), 3)
    cv2.putText(image, "EL-A3", (18, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
    for _ in range(160):
        x = int(rng.integers(10, 130))
        y = int(rng.integers(12, 210))
        color = tuple(int(v) for v in rng.integers(0, 255, size=3))
        cv2.circle(image, (x, y), 1, color, -1)
    return image


def place_template(template: np.ndarray) -> np.ndarray:
    canvas = np.full((720, 1280, 3), 210, dtype=np.uint8)
    x0, y0 = 480, 180
    x1, y1 = x0 + template.shape[1], y0 + template.shape[0]
    canvas[y0:y1, x0:x1] = template
    return canvas


def test_resize_keep_aspect_no_change():
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    resized, scale = resize_keep_aspect(image, 500)
    assert resized.shape == image.shape
    assert scale == 1.0


def test_search_scale_coercion():
    from el_a3_sdk.realsense.book_spine_matcher import coerce_search_scales, normalized_search_scales

    assert coerce_search_scales("1,1.5,2") == (1.0, 1.5, 2.0)
    scales = normalized_search_scales((1.0, 1.5, 2.0), (720, 1280, 3), search_rect=None, max_scaled_frame_side=1800)
    assert scales[0] == 1.0
    assert all(scale > 0 for scale in scales)


def test_geometry_helpers():
    polygon = np.array([[[10, 20]], [[110, 18]], [[112, 200]], [[8, 198]]], dtype=np.int32)
    bbox = polygon_to_bbox(polygon, (300, 400, 3))
    roi = polygon_to_roi(polygon, (300, 400, 3), expand_ratio=0.2, min_pad=10)
    assert bbox == (8, 18, 112, 200)
    assert roi[0] <= bbox[0]
    assert roi[1] <= bbox[1]
    assert roi[2] >= bbox[2]
    assert roi[3] >= bbox[3]


def test_polygon_reasonable_filters_irregular_shape():
    polygon = np.array([[[100, 100]], [[500, 100]], [[500, 110]], [[100, 105]]], dtype=np.int32)
    assert not polygon_is_reasonable(
        polygon,
        (720, 1280, 3),
        min_area_ratio=0.0002,
        max_area_ratio=0.75,
        min_fill_ratio=0.35,
        max_skew_ratio=3.0,
    )


def test_center_alignment_and_state():
    polygon = np.array([[[300, 100]], [[500, 100]], [[500, 500]], [[300, 500]]], dtype=np.int32)
    alignment = evaluate_center_alignment(
        (600, 800, 3),
        polygon,
        center_tolerance_ratio=0.1,
        min_center_tolerance_px=20,
    )
    assert alignment.centered
    stable, green, red = update_stable_state(
        False,
        True,
        0,
        0,
        green_confirm_frames=2,
        red_confirm_frames=3,
    )
    assert not stable
    stable, green, red = update_stable_state(
        stable,
        True,
        green,
        red,
        green_confirm_frames=2,
        red_confirm_frames=3,
    )
    assert stable


def test_book_spine_matcher_finds_synthetic_template():
    template = make_template()
    frame = place_template(template)
    matcher = BookSpineMatcher(
        template,
        BookSpineMatchConfig(
            match_confidence=0.05,
            center_tolerance_ratio=0.5,
            min_center_tolerance_px=10,
            frame_max_side=0,
            template_max_side=1200,
            min_good_matches=8,
            min_inliers=6,
            roi_reacquire_after_misses=2,
            green_confirm_frames=1,
            red_confirm_frames=1,
            search_scales=(1.0, 1.8),
            max_scaled_frame_side=1800,
            use_clahe=True,
        ),
    )

    result = matcher.match_frame(frame)
    assert result.found
    assert result.bbox_xyxy is not None
    x0, y0, x1, y1 = result.bbox_xyxy
    assert abs(x0 - 480) < 35
    assert abs(y0 - 180) < 35
    assert abs(x1 - 620) < 35
    assert abs(y1 - 400) < 35
    assert result.candidate_centered is True

    overlay = draw_book_spine_overlay(
        frame,
        result,
    )
    assert overlay.shape == frame.shape
    assert overlay.dtype == frame.dtype

    result_dict = result.to_dict()
    assert result_dict["found"] is True
    assert result_dict["polygon"] is not None


def test_book_spine_matcher_holds_last_good_polygon_on_reject():
    template = make_template()
    matcher = BookSpineMatcher(
        template,
        BookSpineMatchConfig(
            match_confidence=0.05,
            center_tolerance_ratio=0.5,
            min_center_tolerance_px=10,
            frame_max_side=0,
            template_max_side=1200,
            min_good_matches=8,
            min_inliers=6,
            roi_reacquire_after_misses=2,
            green_confirm_frames=1,
            red_confirm_frames=1,
            polygon_hold_frames=2,
        ),
    )

    frame_shape = (720, 1280, 3)
    good_polygon = np.array([[[300, 100]], [[500, 100]], [[500, 500]], [[300, 500]]], dtype=np.int32)
    good_result = BookSpineMatchResult(
        good_polygon,
        16,
        12,
        0.9,
        "SIFT",
        "FULL",
        accepted=True,
    )
    stabilized = matcher._stabilize_result(good_result, frame_shape)
    assert stabilized.accepted
    assert stabilized.polygon is not None

    bad_polygon = np.array([[[100, 100]], [[500, 100]], [[500, 110]], [[100, 105]]], dtype=np.int32)
    bad_result = BookSpineMatchResult(
        bad_polygon,
        18,
        14,
        0.95,
        "SIFT",
        "FULL",
        accepted=True,
    )
    held = matcher._stabilize_result(bad_result, frame_shape)
    assert held.accepted
    assert held.polygon is not None
    assert np.array_equal(held.polygon, stabilized.polygon)


def test_book_spine_matcher_finds_smaller_template_in_frame():
    template = make_template()
    small = cv2.resize(template, (70, 110), interpolation=cv2.INTER_AREA)
    canvas = np.full((720, 1280, 3), 210, dtype=np.uint8)
    x0, y0 = 510, 220
    canvas[y0:y0 + small.shape[0], x0:x0 + small.shape[1]] = small

    matcher = BookSpineMatcher(
        template,
        BookSpineMatchConfig(
            match_confidence=0.03,
            center_tolerance_ratio=0.6,
            min_center_tolerance_px=10,
            frame_max_side=0,
            template_max_side=1200,
            min_good_matches=6,
            min_inliers=4,
            search_scales=(1.0, 2.0, 3.0),
            max_scaled_frame_side=1800,
            use_clahe=True,
            roi_reacquire_after_misses=1,
            green_confirm_frames=1,
            red_confirm_frames=1,
        ),
    )

    result = matcher.match_once(canvas)
    assert result.found
    assert result.bbox_xyxy is not None
