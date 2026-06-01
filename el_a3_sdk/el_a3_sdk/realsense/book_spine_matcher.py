"""Template-based book spine matching for RealSense color frames."""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

import cv2
import numpy as np


PathLike = Union[str, Path]
Rect = Tuple[int, int, int, int]


def coerce_search_scales(value: Any) -> Tuple[float, ...]:
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",") if part.strip()]
        return tuple(float(part) for part in parts)
    if isinstance(value, (int, float)):
        return (float(value),)
    return tuple(float(scale) for scale in value)


@dataclass(frozen=True)
class BookSpineMatchConfig:
    """Tunable parameters for template-based book spine matching."""

    match_confidence: float = 0.65
    center_tolerance_ratio: float = 0.03
    min_center_tolerance_px: int = 45
    frame_max_side: int = 0
    template_max_side: int = 1200
    min_good_matches: int = 12
    min_inliers: int = 8
    sift_ratio_test: float = 0.72
    sift_features: int = 4000
    acquire_match_confidence: float = 0.50
    acquire_min_good_matches: int = 8
    acquire_min_inliers: int = 5
    acquire_tile_columns: int = 3
    acquire_tile_overlap_ratio: float = 0.20
    search_scales: Tuple[float, ...] = (1.0, 1.5, 2.0)
    max_scaled_frame_side: int = 1800
    use_clahe: bool = True
    clahe_clip_limit: float = 2.0
    clahe_tile_grid_size: int = 8
    roi_expand_ratio: float = 0.45
    roi_min_pad: int = 60
    roi_reacquire_after_misses: int = 3
    polygon_hold_frames: int = 4
    min_polygon_area_ratio: float = 0.0002
    max_polygon_area_ratio: float = 0.75
    min_polygon_fill_ratio: float = 0.35
    max_polygon_skew_ratio: float = 3.0
    max_polygon_jump_ratio: float = 0.25
    max_polygon_area_change_ratio: float = 0.75
    polygon_smoothing_alpha: float = 0.25
    keep_last_good_on_reject: bool = True
    center_smoothing_alpha: float = 0.25
    green_confirm_frames: int = 2
    red_confirm_frames: int = 5

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "BookSpineMatchConfig":
        allowed = {field.name for field in fields(cls)}
        normalized = {
            str(key).replace("-", "_"): value
            for key, value in values.items()
            if str(key).replace("-", "_") in allowed
        }
        if "search_scales" in normalized:
            normalized["search_scales"] = coerce_search_scales(
                normalized["search_scales"]
            )
        return cls(**normalized)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def validate(self) -> None:
        if not 0.0 <= self.match_confidence <= 1.0:
            raise ValueError("match_confidence must be between 0 and 1.")
        if self.center_tolerance_ratio <= 0.0:
            raise ValueError("center_tolerance_ratio must be positive.")
        if self.min_center_tolerance_px < 0:
            raise ValueError("min_center_tolerance_px must be non-negative.")
        if self.frame_max_side < 0:
            raise ValueError("frame_max_side must be non-negative; 0 disables resizing.")
        if self.template_max_side <= 0:
            raise ValueError("template_max_side must be positive.")
        if self.min_good_matches < 4:
            raise ValueError("min_good_matches must be at least 4.")
        if self.min_inliers < 4:
            raise ValueError("min_inliers must be at least 4.")
        if not 0.0 <= self.acquire_match_confidence <= 1.0:
            raise ValueError("acquire_match_confidence must be between 0 and 1.")
        if self.acquire_min_good_matches < 4:
            raise ValueError("acquire_min_good_matches must be at least 4.")
        if self.acquire_min_inliers < 4:
            raise ValueError("acquire_min_inliers must be at least 4.")
        if self.acquire_tile_columns < 1:
            raise ValueError("acquire_tile_columns must be at least 1.")
        if not 0.0 <= self.acquire_tile_overlap_ratio < 1.0:
            raise ValueError("acquire_tile_overlap_ratio must be in [0, 1).")
        if not self.search_scales:
            raise ValueError("search_scales must not be empty.")
        if any(scale <= 0.0 for scale in self.search_scales):
            raise ValueError("all search_scales must be positive.")
        if self.max_scaled_frame_side < 0:
            raise ValueError("max_scaled_frame_side must be non-negative.")
        if self.clahe_clip_limit <= 0.0:
            raise ValueError("clahe_clip_limit must be positive.")
        if self.clahe_tile_grid_size < 1:
            raise ValueError("clahe_tile_grid_size must be at least 1.")
        if self.roi_expand_ratio < 0.0:
            raise ValueError("roi_expand_ratio must be non-negative.")
        if self.roi_min_pad < 0:
            raise ValueError("roi_min_pad must be non-negative.")
        if self.roi_reacquire_after_misses < 1:
            raise ValueError("roi_reacquire_after_misses must be at least 1.")
        if self.polygon_hold_frames < 0:
            raise ValueError("polygon_hold_frames must be non-negative.")
        if self.min_polygon_area_ratio <= 0.0:
            raise ValueError("min_polygon_area_ratio must be positive.")
        if self.max_polygon_area_ratio <= self.min_polygon_area_ratio:
            raise ValueError("max_polygon_area_ratio must be greater than min_polygon_area_ratio.")
        if not 0.0 < self.min_polygon_fill_ratio <= 1.0:
            raise ValueError("min_polygon_fill_ratio must be in (0, 1].")
        if self.max_polygon_skew_ratio < 1.0:
            raise ValueError("max_polygon_skew_ratio must be at least 1.")
        if self.max_polygon_jump_ratio <= 0.0:
            raise ValueError("max_polygon_jump_ratio must be positive.")
        if not 0.0 <= self.max_polygon_area_change_ratio < 1.0:
            raise ValueError("max_polygon_area_change_ratio must be in [0, 1).")
        if not 0.0 < self.polygon_smoothing_alpha <= 1.0:
            raise ValueError("polygon_smoothing_alpha must be in (0, 1].")
        if not 0.0 < self.center_smoothing_alpha <= 1.0:
            raise ValueError("center_smoothing_alpha must be in (0, 1].")
        if self.green_confirm_frames < 1 or self.red_confirm_frames < 1:
            raise ValueError("confirm frame counts must be at least 1.")


@dataclass(frozen=True)
class FeatureBackend:
    name: str
    detector: Any
    matcher: Any
    ratio_test: float
    min_good_matches: int
    min_inliers: int
    template_keypoints: Sequence[Any]
    template_descriptors: np.ndarray
    template_corners: np.ndarray


@dataclass(frozen=True)
class CenterAlignment:
    centered: bool
    center_score: float
    offset_px: float
    tolerance_px: float
    frame_center_x: float
    book_center_x: float
    detected_center_x: float
    left_x: float
    right_x: float


@dataclass(frozen=True)
class BookSpineMatchResult:
    """Result of matching one color frame against one book-spine template."""

    polygon: Optional[np.ndarray]
    good_count: int
    inlier_count: int
    match_confidence: float
    backend_name: str
    search_scope: str
    accepted: bool = False
    bbox_xyxy: Optional[Rect] = None
    center_px: Optional[Tuple[float, float]] = None
    centered: bool = False
    stable_centered: bool = False
    candidate_centered: bool = False
    center_score: float = 0.0
    offset_px: float = 0.0
    tolerance_px: float = 0.0
    frame_center_x: float = 0.0
    book_center_x: float = 0.0
    detected_center_x: float = 0.0
    green_count: int = 0
    red_count: int = 0

    @property
    def found(self) -> bool:
        return self.polygon is not None

    def to_dict(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "found": self.found,
            "accepted": bool(self.accepted),
            "backend_name": self.backend_name,
            "search_scope": self.search_scope,
            "good_count": int(self.good_count),
            "inlier_count": int(self.inlier_count),
            "match_confidence": float(self.match_confidence),
            "bbox_xyxy": list(self.bbox_xyxy) if self.bbox_xyxy is not None else None,
            "center_px": list(self.center_px) if self.center_px is not None else None,
            "centered": bool(self.centered),
            "stable_centered": bool(self.stable_centered),
            "candidate_centered": bool(self.candidate_centered),
            "center_score": float(self.center_score),
            "offset_px": float(self.offset_px),
            "tolerance_px": float(self.tolerance_px),
            "frame_center_x": float(self.frame_center_x),
            "book_center_x": float(self.book_center_x),
            "detected_center_x": float(self.detected_center_x),
            "green_count": int(self.green_count),
            "red_count": int(self.red_count),
        }
        if self.polygon is not None:
            payload["polygon"] = self.polygon.reshape(-1, 2).astype(int).tolist()
        else:
            payload["polygon"] = None
        return payload


def resize_keep_aspect(image: np.ndarray, max_side: int) -> Tuple[np.ndarray, float]:
    if max_side <= 0:
        return image, 1.0
    height, width = image.shape[:2]
    scale = min(float(max_side) / float(max(height, width)), 1.0)
    if scale >= 1.0:
        return image, 1.0
    resized = cv2.resize(
        image,
        (int(width * scale), int(height * scale)),
        interpolation=cv2.INTER_AREA,
    )
    return resized, scale


def scale_keep_aspect(image: np.ndarray, scale: float) -> Tuple[np.ndarray, float]:
    if scale <= 0.0:
        raise ValueError("scale must be positive.")
    if abs(scale - 1.0) < 1e-6:
        return image, 1.0

    height, width = image.shape[:2]
    interpolation = cv2.INTER_CUBIC if scale > 1.0 else cv2.INTER_AREA
    resized = cv2.resize(
        image,
        (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
        interpolation=interpolation,
    )
    return resized, scale


def preprocess_gray(
    image_bgr: np.ndarray,
    *,
    use_clahe: bool,
    clahe_clip_limit: float,
    clahe_tile_grid_size: int,
) -> np.ndarray:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    if not use_clahe:
        return gray
    clahe = cv2.createCLAHE(
        clipLimit=float(clahe_clip_limit),
        tileGridSize=(int(clahe_tile_grid_size), int(clahe_tile_grid_size)),
    )
    return clahe.apply(gray)


def load_template_image(path: PathLike, *, max_side: int = 1000) -> np.ndarray:
    template_path = Path(path)
    image = cv2.imread(str(template_path))
    if image is None:
        raise FileNotFoundError(f"Unable to read template image: {template_path}")
    return resize_keep_aspect(image, max_side)[0]


def clamp_rect(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    frame_width: int,
    frame_height: int,
) -> Rect:
    left = max(0, min(int(x0), frame_width - 1))
    top = max(0, min(int(y0), frame_height - 1))
    right = max(left + 1, min(int(x1), frame_width))
    bottom = max(top + 1, min(int(y1), frame_height))
    return left, top, right, bottom


def polygon_to_bbox(polygon: np.ndarray, frame_shape: Sequence[int]) -> Rect:
    height, width = int(frame_shape[0]), int(frame_shape[1])
    points = polygon.reshape(-1, 2).astype(np.float32)
    return clamp_rect(
        float(points[:, 0].min()),
        float(points[:, 1].min()),
        float(points[:, 0].max()),
        float(points[:, 1].max()),
        width,
        height,
    )


def polygon_to_roi(
    polygon: np.ndarray,
    frame_shape: Sequence[int],
    *,
    expand_ratio: float,
    min_pad: int,
) -> Rect:
    height, width = int(frame_shape[0]), int(frame_shape[1])
    points = polygon.reshape(-1, 2).astype(np.float32)
    left_x = float(points[:, 0].min())
    right_x = float(points[:, 0].max())
    top_y = float(points[:, 1].min())
    bottom_y = float(points[:, 1].max())
    box_w = max(right_x - left_x, 1.0)
    box_h = max(bottom_y - top_y, 1.0)
    pad_x = max(int(box_w * expand_ratio), int(min_pad))
    pad_y = max(int(box_h * expand_ratio), int(min_pad))
    return clamp_rect(
        left_x - pad_x,
        top_y - pad_y,
        right_x + pad_x,
        bottom_y + pad_y,
        width,
        height,
    )


def polygon_area(polygon: np.ndarray) -> float:
    points = polygon.reshape(-1, 2).astype(np.float32)
    return float(abs(cv2.contourArea(points)))


def polygon_bbox_area(polygon: np.ndarray) -> float:
    points = polygon.reshape(-1, 2).astype(np.float32)
    if points.size == 0:
        return 0.0
    width = float(max(points[:, 0].max() - points[:, 0].min(), 0.0))
    height = float(max(points[:, 1].max() - points[:, 1].min(), 0.0))
    return float(width * height)


def polygon_fill_ratio(polygon: np.ndarray) -> float:
    bbox_area = polygon_bbox_area(polygon)
    if bbox_area <= 0.0:
        return 0.0
    return polygon_area(polygon) / bbox_area


def polygon_skew_ratio(polygon: np.ndarray) -> float:
    points = polygon.reshape(-1, 2).astype(np.float32)
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    positive = lengths[lengths > 1e-6]
    if positive.size == 0:
        return float("inf")
    return float(positive.max() / max(positive.min(), 1e-6))


def polygon_center(polygon: np.ndarray) -> np.ndarray:
    points = polygon.reshape(-1, 2).astype(np.float32)
    return np.array([float(points[:, 0].mean()), float(points[:, 1].mean())], dtype=np.float32)


def polygon_jump_ratio(
    current_polygon: np.ndarray,
    previous_polygon: np.ndarray,
    frame_shape: Sequence[int],
) -> float:
    height, width = int(frame_shape[0]), int(frame_shape[1])
    frame_diagonal = float(max(np.hypot(width, height), 1.0))
    delta = polygon_center(current_polygon) - polygon_center(previous_polygon)
    return float(np.linalg.norm(delta) / frame_diagonal)


def polygon_area_change_ratio(
    current_polygon: np.ndarray,
    previous_polygon: np.ndarray,
) -> float:
    current_area = polygon_area(current_polygon)
    previous_area = polygon_area(previous_polygon)
    return float(
        abs(current_area - previous_area) / max(current_area, previous_area, 1.0)
    )


def smooth_polygon(
    previous_polygon: np.ndarray,
    current_polygon: np.ndarray,
    alpha: float,
) -> np.ndarray:
    if not 0.0 < alpha <= 1.0:
        raise ValueError("alpha must be in (0, 1].")
    previous_points = np.asarray(previous_polygon, dtype=np.float32).reshape(-1, 2)
    current_points = np.asarray(current_polygon, dtype=np.float32).reshape(-1, 2)
    if previous_points.shape != current_points.shape:
        raise ValueError("polygons must have the same shape.")
    blended = alpha * current_points + (1.0 - alpha) * previous_points
    return np.rint(blended).astype(np.int32).reshape(-1, 1, 2)


def polygon_is_reasonable(
    polygon: np.ndarray,
    frame_shape: Sequence[int],
    *,
    min_area_ratio: float,
    max_area_ratio: float,
    min_fill_ratio: float,
    max_skew_ratio: float,
) -> bool:
    points = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
    if points.size == 0 or points.shape[0] < 4:
        return False
    if not np.isfinite(points).all():
        return False
    height, width = int(frame_shape[0]), int(frame_shape[1])
    frame_area = float(max(width * height, 1))
    area = polygon_area(points)
    if area <= 0.0:
        return False
    area_ratio = area / frame_area
    if area_ratio < min_area_ratio or area_ratio > max_area_ratio:
        return False
    fill_ratio = polygon_fill_ratio(points)
    if fill_ratio < min_fill_ratio:
        return False
    if polygon_skew_ratio(points) > max_skew_ratio:
        return False
    if not cv2.isContourConvex(points.reshape(-1, 1, 2)):
        return False
    return True


def evaluate_center_alignment(
    frame_shape: Sequence[int],
    polygon: np.ndarray,
    center_tolerance_ratio: float,
    min_center_tolerance_px: int,
    smoothed_center_x: Optional[float] = None,
) -> CenterAlignment:
    width = int(frame_shape[1])
    frame_center_x = width / 2.0
    points = polygon.reshape(-1, 2).astype(np.float32)
    left_x = float(points[:, 0].min())
    right_x = float(points[:, 0].max())
    detected_center_x = (left_x + right_x) / 2.0
    book_center_x = smoothed_center_x if smoothed_center_x is not None else detected_center_x
    offset_px = abs(book_center_x - frame_center_x)
    tolerance_px = max(width * center_tolerance_ratio, float(min_center_tolerance_px))
    centered = offset_px <= tolerance_px
    center_score = max(0.0, 1.0 - offset_px / max(tolerance_px, 1e-6))
    return CenterAlignment(
        centered=centered,
        center_score=center_score,
        offset_px=offset_px,
        tolerance_px=tolerance_px,
        frame_center_x=frame_center_x,
        book_center_x=book_center_x,
        detected_center_x=detected_center_x,
        left_x=left_x,
        right_x=right_x,
    )


def update_stable_state(
    current_state: bool,
    candidate_state: bool,
    green_count: int,
    red_count: int,
    *,
    green_confirm_frames: int,
    red_confirm_frames: int,
) -> Tuple[bool, int, int]:
    if candidate_state:
        green_count += 1
        red_count = 0
        if green_count >= green_confirm_frames:
            current_state = True
    else:
        red_count += 1
        green_count = 0
        if red_count >= red_confirm_frames:
            current_state = False
    return current_state, green_count, red_count


def build_feature_backends(
    template_image_bgr: np.ndarray,
    config: BookSpineMatchConfig,
) -> List[FeatureBackend]:
    config.validate()
    gray = preprocess_gray(
        template_image_bgr,
        use_clahe=config.use_clahe,
        clahe_clip_limit=config.clahe_clip_limit,
        clahe_tile_grid_size=config.clahe_tile_grid_size,
    )
    height, width = template_image_bgr.shape[:2]
    corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]]).reshape(
        -1,
        1,
        2,
    )
    backends: List[FeatureBackend] = []

    def make_backend(name: str, detector: Any, norm_type: int, ratio_test: float) -> None:
        keypoints, descriptors = detector.detectAndCompute(gray, None)
        if descriptors is None or len(keypoints) < config.min_good_matches:
            return
        matcher: Any
        if name == "SIFT":
            index_params = dict(algorithm=1, trees=5)
            search_params = dict(checks=48)
            matcher = cv2.FlannBasedMatcher(index_params, search_params)
            descriptors = np.asarray(descriptors, dtype=np.float32)
        else:
            matcher = cv2.BFMatcher(norm_type)
        backends.append(
            FeatureBackend(
                name=name,
                detector=detector,
                matcher=matcher,
                ratio_test=ratio_test,
                min_good_matches=config.min_good_matches,
                min_inliers=config.min_inliers,
                template_keypoints=keypoints,
                template_descriptors=descriptors,
                template_corners=corners,
            )
        )

    if not hasattr(cv2, "SIFT_create"):
        raise ValueError("Current OpenCV build does not provide SIFT_create.")

    make_backend(
        "SIFT",
        cv2.SIFT_create(nfeatures=config.sift_features),
        cv2.NORM_L2,
        config.sift_ratio_test,
    )

    if not backends:
        raise ValueError(
            "Template image has too few SIFT features for matching."
        )
    return backends


def match_target(
    frame_bgr: np.ndarray,
    backend: FeatureBackend,
    *,
    frame_max_side: int,
    search_rect: Optional[Rect] = None,
    search_scope: Optional[str] = None,
    scale: float = 1.0,
    use_clahe: bool = True,
    clahe_clip_limit: float = 2.0,
    clahe_tile_grid_size: int = 8,
    min_good_matches: Optional[int] = None,
    min_inliers: Optional[int] = None,
) -> BookSpineMatchResult:
    if search_rect is None:
        search_frame = frame_bgr
        offset_x = 0
        offset_y = 0
        scope_name = "FULL"
    else:
        x0, y0, x1, y1 = search_rect
        search_frame = frame_bgr[y0:y1, x0:x1]
        offset_x = x0
        offset_y = y0
        scope_name = "ROI"
    if search_scope is not None:
        scope_name = search_scope

    required_good_matches = (
        backend.min_good_matches if min_good_matches is None else int(min_good_matches)
    )
    required_inliers = backend.min_inliers if min_inliers is None else int(min_inliers)

    frame_scaled, scale_factor = scale_keep_aspect(search_frame, scale)
    if frame_max_side > 0:
        frame_scaled, frame_scale = resize_keep_aspect(frame_scaled, frame_max_side)
        scale_factor *= frame_scale
    gray = preprocess_gray(
        frame_scaled,
        use_clahe=use_clahe,
        clahe_clip_limit=clahe_clip_limit,
        clahe_tile_grid_size=clahe_tile_grid_size,
    )
    frame_keypoints, frame_descriptors = backend.detector.detectAndCompute(gray, None)

    if frame_descriptors is None or len(frame_keypoints) < required_good_matches:
        return BookSpineMatchResult(None, 0, 0, 0.0, backend.name, scope_name)

    template_descriptors = np.asarray(backend.template_descriptors, dtype=np.float32)
    frame_descriptors = np.asarray(frame_descriptors, dtype=np.float32)

    raw_matches = backend.matcher.knnMatch(
        template_descriptors,
        frame_descriptors,
        k=2,
    )
    good_matches = []
    for pair in raw_matches:
        if len(pair) < 2:
            continue
        first, second = pair
        if first.distance < backend.ratio_test * second.distance:
            good_matches.append(first)

    if len(good_matches) < required_good_matches:
        return BookSpineMatchResult(
            None,
            len(good_matches),
            0,
            0.0,
            backend.name,
            scope_name,
        )

    src_pts = np.float32(
        [backend.template_keypoints[match.queryIdx].pt for match in good_matches]
    ).reshape(-1, 1, 2)
    dst_pts = np.float32(
        [frame_keypoints[match.trainIdx].pt for match in good_matches]
    ).reshape(-1, 1, 2)
    matrix, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

    if matrix is None or mask is None:
        return BookSpineMatchResult(
            None,
            len(good_matches),
            0,
            0.0,
            backend.name,
            scope_name,
        )

    inliers = int(mask.ravel().sum())
    confidence = inliers / max(len(good_matches), 1)
    if inliers < required_inliers:
        return BookSpineMatchResult(
            None,
            len(good_matches),
            inliers,
            confidence,
            backend.name,
            scope_name,
        )

    projected = cv2.perspectiveTransform(backend.template_corners, matrix)
    projected = projected / max(scale_factor, 1e-6)
    if offset_x or offset_y:
        projected[:, 0, 0] += offset_x
        projected[:, 0, 1] += offset_y

    return BookSpineMatchResult(
        projected.astype(np.int32),
        len(good_matches),
        inliers,
        confidence,
        backend.name,
        scope_name,
    )


def match_with_fallback(
    frame_bgr: np.ndarray,
    backends: Iterable[FeatureBackend],
    *,
    frame_max_side: int,
    search_rect: Optional[Rect],
    match_confidence: float,
    search_scales: Sequence[float] = (1.0,),
    max_scaled_frame_side: int = 0,
    use_clahe: bool = True,
    clahe_clip_limit: float = 2.0,
    clahe_tile_grid_size: int = 8,
    search_scope: Optional[str] = None,
    min_good_matches: Optional[int] = None,
    min_inliers: Optional[int] = None,
    polygon_validator: Optional[
        Callable[[np.ndarray, Sequence[int]], bool]
    ] = None,
) -> BookSpineMatchResult:
    best_result: Optional[BookSpineMatchResult] = None
    scales = normalized_search_scales(
        search_scales,
        frame_bgr.shape,
        search_rect=search_rect,
        max_scaled_frame_side=max_scaled_frame_side,
    )
    for scale in scales:
        for backend in backends:
            result = match_target(
                frame_bgr,
                backend,
                frame_max_side=frame_max_side,
                search_rect=search_rect,
                search_scope=search_scope,
                scale=scale,
                use_clahe=use_clahe,
                clahe_clip_limit=clahe_clip_limit,
                clahe_tile_grid_size=clahe_tile_grid_size,
                min_good_matches=min_good_matches,
                min_inliers=min_inliers,
            )
            polygon_is_valid = True
            if result.polygon is not None and polygon_validator is not None:
                polygon_is_valid = bool(polygon_validator(result.polygon, frame_bgr.shape))
            if result.polygon is not None and not polygon_is_valid:
                result = _replace_result(result, polygon=None, accepted=False)
            else:
                result = _replace_result(
                    result,
                    accepted=result.polygon is not None
                    and result.match_confidence >= match_confidence,
                )
            if result.accepted:
                return result
            if best_result is None:
                best_result = result
            elif (
                result.polygon is not None
                and result.match_confidence > best_result.match_confidence
            ):
                best_result = result
            elif best_result.polygon is None and result.inlier_count > best_result.inlier_count:
                best_result = result

    if best_result is None:
        scope_name = search_scope if search_scope is not None else ("ROI" if search_rect is not None else "FULL")
        return BookSpineMatchResult(None, 0, 0, 0.0, "NONE", scope_name)
    return best_result


def normalized_search_scales(
    search_scales: Sequence[float],
    frame_shape: Sequence[int],
    *,
    search_rect: Optional[Rect],
    max_scaled_frame_side: int,
) -> Tuple[float, ...]:
    if search_rect is None:
        source_height = int(frame_shape[0])
        source_width = int(frame_shape[1])
    else:
        x0, y0, x1, y1 = search_rect
        source_width = max(1, int(x1) - int(x0))
        source_height = max(1, int(y1) - int(y0))

    max_side = max(source_width, source_height)
    values: List[float] = []
    for raw_scale in search_scales:
        scale = float(raw_scale)
        if scale <= 0.0:
            continue
        if max_scaled_frame_side > 0 and max_side * scale > max_scaled_frame_side:
            scale = max_scaled_frame_side / float(max_side)
        if scale <= 0.0:
            continue
        if not any(abs(existing - scale) < 1e-6 for existing in values):
            values.append(scale)
    if not values:
        values.append(1.0)
    return tuple(values)


class BookSpineMatcher:
    """Stateful template matcher for one target book spine."""

    def __init__(
        self,
        template_image_bgr: np.ndarray,
        config: Optional[BookSpineMatchConfig] = None,
    ) -> None:
        self.config = config or BookSpineMatchConfig()
        self.config.validate()
        self.template_image_bgr = resize_keep_aspect(
            template_image_bgr,
            self.config.template_max_side,
        )[0]
        template_height, template_width = self.template_image_bgr.shape[:2]
        self.template_skew_ratio = max(template_width, template_height) / float(
            max(min(template_width, template_height), 1)
        )
        self.backends = build_feature_backends(self.template_image_bgr, self.config)
        self.reset_tracking()

    @classmethod
    def from_template_path(
        cls,
        path: PathLike,
        config: Optional[BookSpineMatchConfig] = None,
    ) -> "BookSpineMatcher":
        cfg = config or BookSpineMatchConfig()
        return cls(load_template_image(path, max_side=cfg.template_max_side), cfg)

    def reset_tracking(self) -> None:
        self.roi_rect: Optional[Rect] = None
        self.roi_miss_streak = 0
        self.smoothed_book_center_x: Optional[float] = None
        self.smoothed_polygon: Optional[np.ndarray] = None
        self.last_good_result: Optional[BookSpineMatchResult] = None
        self.polygon_hold_streak = 0
        self.stable_centered = False
        self.green_count = 0
        self.red_count = 0

    def match_frame(self, frame_bgr: np.ndarray) -> BookSpineMatchResult:
        use_roi = (
            self.roi_rect is not None
            and self.roi_miss_streak < self.config.roi_reacquire_after_misses
        )
        if use_roi:
            result = match_with_fallback(
                frame_bgr,
                self.backends,
                frame_max_side=self.config.frame_max_side,
                search_rect=self.roi_rect,
                match_confidence=self.config.match_confidence,
                search_scales=(1.0,),
                max_scaled_frame_side=self.config.max_scaled_frame_side,
                use_clahe=self.config.use_clahe,
                clahe_clip_limit=self.config.clahe_clip_limit,
                clahe_tile_grid_size=self.config.clahe_tile_grid_size,
                polygon_validator=self._polygon_validator,
            )
        else:
            result = self._acquire_book_spine(frame_bgr)
        result = self._stabilize_result(result, frame_bgr.shape)

        if result.accepted:
            self.roi_miss_streak = 0
        elif use_roi:
            self.roi_miss_streak += 1
            if self.roi_miss_streak >= self.config.roi_reacquire_after_misses:
                result = self._acquire_book_spine(frame_bgr)
                result = self._stabilize_result(result, frame_bgr.shape)
                if result.accepted:
                    self.roi_miss_streak = 0
                else:
                    self.roi_rect = None
                    self.roi_miss_streak = 0
        elif result.search_scope == "FULL" and not result.accepted:
            self.roi_rect = None

        if result.accepted and result.polygon is not None:
            self.roi_rect = polygon_to_roi(
                result.polygon,
                frame_bgr.shape,
                expand_ratio=self.config.roi_expand_ratio,
                min_pad=self.config.roi_min_pad,
            )
            self.roi_miss_streak = 0
        elif result.search_scope == "FULL":
            self.roi_rect = None

        return self._with_tracking_state(result, frame_bgr.shape)

    def match_once(
        self,
        frame_bgr: np.ndarray,
        search_rect: Optional[Rect] = None,
    ) -> BookSpineMatchResult:
        result = match_with_fallback(
            frame_bgr,
            self.backends,
            frame_max_side=self.config.frame_max_side,
            search_rect=search_rect,
            match_confidence=self.config.match_confidence,
            search_scales=self.config.search_scales,
            max_scaled_frame_side=self.config.max_scaled_frame_side,
            use_clahe=self.config.use_clahe,
            clahe_clip_limit=self.config.clahe_clip_limit,
            clahe_tile_grid_size=self.config.clahe_tile_grid_size,
            polygon_validator=self._polygon_validator,
        )
        return self._with_geometry(result, frame_bgr.shape)

    def _acquisition_search_rects(
        self,
        frame_shape: Sequence[int],
    ) -> List[Rect]:
        height = int(frame_shape[0])
        width = int(frame_shape[1])
        tile_columns = max(1, int(self.config.acquire_tile_columns))
        if tile_columns == 1:
            return [(0, 0, width, height)]

        overlap = min(max(float(self.config.acquire_tile_overlap_ratio), 0.0), 0.45)
        step = width / float(tile_columns)
        pad = max(0, int(round(step * overlap)))

        rects: List[Rect] = []
        for index in range(tile_columns):
            left = max(0, int(round(index * step)) - pad)
            right = min(width, int(round((index + 1) * step)) + pad)
            if right - left >= 2:
                rects.append((left, 0, right, height))

        rects.sort(key=lambda rect: abs(((rect[0] + rect[2]) / 2.0) - (width / 2.0)))
        unique_rects: List[Rect] = []
        for rect in rects:
            if rect not in unique_rects:
                unique_rects.append(rect)
        return unique_rects

    def _acquire_book_spine(self, frame_bgr: np.ndarray) -> BookSpineMatchResult:
        best_result: Optional[BookSpineMatchResult] = None
        frame_shape = frame_bgr.shape
        for search_rect in self._acquisition_search_rects(frame_shape):
            result = match_with_fallback(
                frame_bgr,
                self.backends,
                frame_max_side=self.config.frame_max_side,
                search_rect=search_rect,
                search_scope="ACQUIRE",
                match_confidence=self.config.acquire_match_confidence,
                search_scales=(1.0,),
                max_scaled_frame_side=self.config.max_scaled_frame_side,
                use_clahe=self.config.use_clahe,
                clahe_clip_limit=self.config.clahe_clip_limit,
                clahe_tile_grid_size=self.config.clahe_tile_grid_size,
                min_good_matches=self.config.acquire_min_good_matches,
                min_inliers=self.config.acquire_min_inliers,
                polygon_validator=self._polygon_validator,
            )
            result = self._stabilize_result(result, frame_shape)
            if result.accepted:
                return result
            if best_result is None:
                best_result = result
            elif (
                result.polygon is not None
                and result.match_confidence > best_result.match_confidence
            ):
                best_result = result
            elif best_result.polygon is None and result.inlier_count > best_result.inlier_count:
                best_result = result

        fallback_result = match_with_fallback(
            frame_bgr,
            self.backends,
            frame_max_side=self.config.frame_max_side,
            search_rect=None,
            search_scope="ACQUIRE",
            match_confidence=self.config.acquire_match_confidence,
            search_scales=(1.0,),
            max_scaled_frame_side=self.config.max_scaled_frame_side,
            use_clahe=self.config.use_clahe,
            clahe_clip_limit=self.config.clahe_clip_limit,
            clahe_tile_grid_size=self.config.clahe_tile_grid_size,
            min_good_matches=self.config.acquire_min_good_matches,
            min_inliers=self.config.acquire_min_inliers,
            polygon_validator=self._polygon_validator,
        )
        fallback_result = self._stabilize_result(fallback_result, frame_shape)
        if fallback_result.accepted:
            return fallback_result
        if best_result is None:
            return fallback_result
        if best_result.polygon is None and fallback_result.polygon is not None:
            return fallback_result
        if (
            best_result.polygon is not None
            and fallback_result.polygon is not None
            and fallback_result.match_confidence > best_result.match_confidence
        ):
            return fallback_result
        return best_result

    def _with_tracking_state(
        self,
        result: BookSpineMatchResult,
        frame_shape: Sequence[int],
    ) -> BookSpineMatchResult:
        result = self._with_geometry(result, frame_shape)
        centered = False
        candidate_centered = False
        alignment = None

        if result.polygon is not None:
            alignment = evaluate_center_alignment(
                frame_shape,
                result.polygon,
                self.config.center_tolerance_ratio,
                self.config.min_center_tolerance_px,
                self.smoothed_book_center_x,
            )
            detected_center_x = alignment.detected_center_x
            if self.smoothed_book_center_x is None:
                self.smoothed_book_center_x = detected_center_x
            else:
                alpha = self.config.center_smoothing_alpha
                self.smoothed_book_center_x = (
                    alpha * detected_center_x
                    + (1.0 - alpha) * self.smoothed_book_center_x
                )
            alignment = evaluate_center_alignment(
                frame_shape,
                result.polygon,
                self.config.center_tolerance_ratio,
                self.config.min_center_tolerance_px,
                self.smoothed_book_center_x,
            )
            centered = alignment.centered
        else:
            self.smoothed_book_center_x = None

        candidate_centered = (
            result.polygon is not None
            and centered
            and result.match_confidence >= self.config.match_confidence
        )
        self.stable_centered, self.green_count, self.red_count = update_stable_state(
            self.stable_centered,
            candidate_centered,
            self.green_count,
            self.red_count,
            green_confirm_frames=self.config.green_confirm_frames,
            red_confirm_frames=self.config.red_confirm_frames,
        )

        return self._with_alignment(
            result,
            alignment,
            centered=centered,
            stable_centered=self.stable_centered,
            candidate_centered=candidate_centered,
            green_count=self.green_count,
            red_count=self.red_count,
        )

    def _with_geometry(
        self,
        result: BookSpineMatchResult,
        frame_shape: Sequence[int],
    ) -> BookSpineMatchResult:
        if result.polygon is None:
            return result
        bbox = polygon_to_bbox(result.polygon, frame_shape)
        center = ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)
        return _replace_result(result, bbox_xyxy=bbox, center_px=center)

    def _polygon_validator(self, polygon: np.ndarray, frame_shape: Sequence[int]) -> bool:
        max_skew_ratio = max(
            self.config.max_polygon_skew_ratio,
            self.template_skew_ratio * 2.0,
        )
        return polygon_is_reasonable(
            polygon,
            frame_shape,
            min_area_ratio=self.config.min_polygon_area_ratio,
            max_area_ratio=self.config.max_polygon_area_ratio,
            min_fill_ratio=self.config.min_polygon_fill_ratio,
            max_skew_ratio=max_skew_ratio,
        )

    def _stabilize_result(
        self,
        result: BookSpineMatchResult,
        frame_shape: Sequence[int],
    ) -> BookSpineMatchResult:
        if result.polygon is not None:
            current_polygon = np.asarray(result.polygon, dtype=np.float32).reshape(-1, 1, 2)
            polygon_ok = result.accepted and self._polygon_validator(
                current_polygon,
                frame_shape,
            )
            if polygon_ok and self.smoothed_polygon is not None:
                if (
                    polygon_jump_ratio(current_polygon, self.smoothed_polygon, frame_shape)
                    > self.config.max_polygon_jump_ratio
                    or polygon_area_change_ratio(current_polygon, self.smoothed_polygon)
                    > self.config.max_polygon_area_change_ratio
                ):
                    polygon_ok = False
            if polygon_ok:
                if self.smoothed_polygon is None:
                    stabilized_polygon = np.rint(current_polygon).astype(np.int32)
                else:
                    stabilized_polygon = smooth_polygon(
                        self.smoothed_polygon,
                        current_polygon,
                        self.config.polygon_smoothing_alpha,
                    )
                self.smoothed_polygon = stabilized_polygon.astype(np.float32)
                self.last_good_result = _replace_result(
                    result,
                    polygon=stabilized_polygon,
                    accepted=True,
                )
                self.polygon_hold_streak = 0
                return self.last_good_result

        if (
            self.config.keep_last_good_on_reject
            and self.last_good_result is not None
            and self.polygon_hold_streak < self.config.polygon_hold_frames
        ):
            self.polygon_hold_streak += 1
            return _replace_result(self.last_good_result, accepted=True)

        self.smoothed_polygon = None
        self.last_good_result = None
        self.polygon_hold_streak = 0
        if result.polygon is not None:
            return _replace_result(result, polygon=None, accepted=False)
        return result

    def _with_alignment(
        self,
        result: BookSpineMatchResult,
        alignment: Optional[CenterAlignment],
        *,
        centered: bool,
        stable_centered: bool,
        candidate_centered: bool,
        green_count: int,
        red_count: int,
    ) -> BookSpineMatchResult:
        updates: Dict[str, Any] = {
            "centered": centered,
            "stable_centered": stable_centered,
            "candidate_centered": candidate_centered,
            "green_count": green_count,
            "red_count": red_count,
        }
        if alignment is not None:
            updates.update(
                center_score=alignment.center_score,
                offset_px=alignment.offset_px,
                tolerance_px=alignment.tolerance_px,
                frame_center_x=alignment.frame_center_x,
                book_center_x=alignment.book_center_x,
                detected_center_x=alignment.detected_center_x,
            )
        return _replace_result(result, **updates)


def draw_book_spine_overlay(
    frame_bgr: np.ndarray,
    result: BookSpineMatchResult,
) -> np.ndarray:
    display = frame_bgr.copy()
    status_color = (0, 255, 0) if result.stable_centered else (0, 0, 255)

    if result.accepted and result.polygon is not None:
        cv2.polylines(display, [result.polygon], True, status_color, 3, cv2.LINE_AA)

    if result.polygon is None:
        status = (
            f"{result.backend_name} {result.search_scope} | NOT FOUND | "
            f"matches={result.good_count} inliers={result.inlier_count}"
        )
    elif result.match_confidence < 0.001:
        status = (
            f"{result.backend_name} {result.search_scope} | LOW MATCH | "
            f"matches={result.good_count} inliers={result.inlier_count}"
        )
    elif result.stable_centered:
        status = (
            f"{result.backend_name} {result.search_scope} | CENTERED | "
            f"conf={result.match_confidence * 100.0:.1f}% "
            f"offset={result.offset_px:.0f}px / tol={result.tolerance_px:.0f}px"
        )
    else:
        status = (
            f"{result.backend_name} {result.search_scope} | OFF CENTER | "
            f"conf={result.match_confidence * 100.0:.1f}% "
            f"offset={result.offset_px:.0f}px / tol={result.tolerance_px:.0f}px"
        )

    put_text(display, status, (20, 36), status_color)
    put_text(
        display,
        f"Candidate: {'YES' if result.candidate_centered else 'NO'} "
        f"green={result.green_count} red={result.red_count}",
        (20, 72),
        (255, 255, 255),
    )
    return display


def put_text(
    image: np.ndarray,
    text: str,
    origin: Tuple[int, int],
    color: Tuple[int, int, int],
) -> None:
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)


def _replace_result(result: BookSpineMatchResult, **updates: Any) -> BookSpineMatchResult:
    return replace(result, **updates)


__all__ = [
    "BookSpineMatchConfig",
    "BookSpineMatcher",
    "BookSpineMatchResult",
    "CenterAlignment",
    "FeatureBackend",
    "Rect",
    "build_feature_backends",
    "clamp_rect",
    "draw_book_spine_overlay",
    "evaluate_center_alignment",
    "load_template_image",
    "match_target",
    "match_with_fallback",
    "polygon_to_bbox",
    "polygon_to_roi",
    "resize_keep_aspect",
    "update_stable_state",
]
