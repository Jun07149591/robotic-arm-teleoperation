"""SIFT-based book-spine localization for MotorStudio."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


DEFAULT_TEMPLATE_PATH = (
    Path(__file__).resolve().parents[2] / "assets" / "book_photos" / "net2.png"
)
FEATURE_SCALES = (1.0, 1.5)
TEMPLATE_MAX_HEIGHT = 1600


@dataclass(frozen=True)
class BookSpinePick:
    pixel_uv: tuple[int, int]
    corners: np.ndarray
    score: float
    good_matches: int
    inliers: int


@dataclass
class _TemplateData:
    image: np.ndarray
    gray: np.ndarray
    keypoints: list
    descriptors: np.ndarray
    size: tuple[int, int]


def _preprocess(image_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    return cv2.GaussianBlur(gray, (3, 3), 0)


def _resize_for_features(image_bgr: np.ndarray, scale: float) -> np.ndarray:
    if scale == 1.0:
        return image_bgr
    height, width = image_bgr.shape[:2]
    return cv2.resize(
        image_bgr,
        (max(1, int(width * scale)), max(1, int(height * scale))),
        interpolation=cv2.INTER_CUBIC,
    )


def _top_quarter_center(corners: np.ndarray, width: int, height: int) -> tuple[int, int]:
    tl, tr, br, bl = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    left = tl * 0.75 + bl * 0.25
    right = tr * 0.75 + br * 0.25
    point = (left + right) * 0.5
    u = int(round(float(np.clip(point[0], 0, width - 1))))
    v = int(round(float(np.clip(point[1], 0, height - 1))))
    return u, v


def _load_image_bgr(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"Failed to read image: {path}")
    return image


def _preprocess_for_features(image_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    return cv2.GaussianBlur(gray, (3, 3), 0)


def _resize_template_for_features(image_bgr: np.ndarray) -> np.ndarray:
    height, width = image_bgr.shape[:2]
    if height <= TEMPLATE_MAX_HEIGHT:
        return image_bgr
    scale = TEMPLATE_MAX_HEIGHT / height
    return cv2.resize(
        image_bgr,
        (max(1, int(width * scale)), TEMPLATE_MAX_HEIGHT),
        interpolation=cv2.INTER_AREA,
    )


def _resize_frame_for_features(image_bgr: np.ndarray, scale: float) -> np.ndarray:
    if scale == 1.0:
        return image_bgr
    height, width = image_bgr.shape[:2]
    return cv2.resize(
        image_bgr,
        (max(1, int(width * scale)), max(1, int(height * scale))),
        interpolation=cv2.INTER_CUBIC,
    )


class _BookSpineRecognizer:
    def __init__(
        self,
        template_path: Path | str = DEFAULT_TEMPLATE_PATH,
        threshold: float = 30.0,
        min_matches: int = 6,
        min_inliers: int = 5,
    ) -> None:
        self.template_path = Path(template_path)
        self.threshold = float(threshold)
        self.min_matches = int(min_matches)
        self.min_inliers = int(min_inliers)
        self._template_cache: _TemplateData | None = None
        self._detector = cv2.SIFT_create(
            nfeatures=4000,
            contrastThreshold=0.01,
            edgeThreshold=8,
            sigma=1.2,
        )
        self._matcher = cv2.BFMatcher(cv2.NORM_L2)
        self._ratio = 0.78

    def _load_template(self) -> _TemplateData:
        if self._template_cache is not None:
            return self._template_cache

        image = _resize_template_for_features(_load_image_bgr(self.template_path))
        gray = _preprocess_for_features(image)
        keypoints, descriptors = self._detector.detectAndCompute(gray, None)
        if descriptors is None or len(keypoints) < self.min_matches:
            raise RuntimeError(
                f"Template {self.template_path} did not produce enough features for SIFT."
            )

        data = _TemplateData(
            image=image,
            gray=gray,
            keypoints=keypoints,
            descriptors=descriptors,
            size=(image.shape[1], image.shape[0]),
        )
        self._template_cache = data
        return data

    def detect(self, frame_bgr: np.ndarray) -> BookSpinePick:
        template = self._load_template()
        best = None

        for scale in FEATURE_SCALES:
            scaled_frame = _resize_frame_for_features(frame_bgr, scale)
            gray = _preprocess_for_features(scaled_frame)
            keypoints_frame, descriptors_frame = self._detector.detectAndCompute(gray, None)
            if descriptors_frame is None or len(keypoints_frame) < self.min_matches:
                continue

            raw_matches = self._matcher.knnMatch(template.descriptors, descriptors_frame, k=2)
            matches_2 = [match for match in raw_matches if len(match) == 2]
            good_matches = [m for m, n in matches_2 if m.distance < self._ratio * n.distance]
            if len(good_matches) < self.min_matches:
                continue

            src_pts = np.float32([template.keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([keypoints_frame[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            homography, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 6.0)
            if homography is None or mask is None:
                continue

            inliers = int(mask.ravel().sum())
            score = 100.0 * inliers / max(1, len(good_matches))
            candidate = (score, len(good_matches), inliers, scale, homography)
            if best is None or candidate[:3] > best[:3]:
                best = candidate

        if best is None:
            raise RuntimeError("未识别到书脊：没有足够的 SIFT 匹配。")

        score, good_matches, inliers, scale, homography = best
        if score < self.threshold or inliers < self.min_inliers:
            raise RuntimeError(
                f"书脊识别置信度不足: score={score:.1f}%, inliers={inliers}"
            )

        template_h, template_w = template.image.shape[:2]
        template_corners = np.float32(
            [[0, 0], [template_w - 1, 0], [template_w - 1, template_h - 1], [0, template_h - 1]]
        ).reshape(-1, 1, 2)
        corners = cv2.perspectiveTransform(template_corners, homography).reshape(4, 2)
        if scale != 1.0:
            corners = corners / scale

        height, width = frame_bgr.shape[:2]
        pixel_uv = _top_quarter_center(corners, width, height)
        return BookSpinePick(
            pixel_uv=pixel_uv,
            corners=corners.astype(np.float32),
            score=float(score),
            good_matches=int(good_matches),
            inliers=int(inliers),
        )


def locate_book_spine_pick(
    frame_bgr: np.ndarray,
    *,
    template_path: Path | str = DEFAULT_TEMPLATE_PATH,
    threshold: float = 30.0,
    min_matches: int = 6,
    min_inliers: int = 5,
) -> BookSpinePick:
    """Locate a book spine and return the top-quarter horizontal center pixel."""
    recognizer = _BookSpineRecognizer(
        template_path=template_path,
        threshold=threshold,
        min_matches=min_matches,
        min_inliers=min_inliers,
    )
    return recognizer.detect(frame_bgr)


__all__ = ["BookSpinePick", "DEFAULT_TEMPLATE_PATH", "locate_book_spine_pick"]
