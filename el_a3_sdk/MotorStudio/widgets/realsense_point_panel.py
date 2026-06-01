"""RealSense point-cloud picking page for MotorStudio."""

from __future__ import annotations

import logging
import math
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from PyQt6.QtCore import QEvent, QSignalBlocker, QThread, Qt, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from MotorStudio.utils.i18n import tr
from MotorStudio.utils.style import SCENE_COLORS
from MotorStudio.utils.theme_manager import ThemeManager

try:
    import cv2
except Exception:
    cv2 = None

try:
    import pyvista as pv
    from pyvistaqt import QtInteractor
    from vtkmodules.vtkRenderingCore import vtkPointPicker

    HAS_PYVISTA = True
except Exception as _exc:
    pv = None
    QtInteractor = None
    vtkPointPicker = None
    HAS_PYVISTA = False
    logging.getLogger("MotorStudio.realsense_panel").warning(
        "pyvista / pyvistaqt 不可用，RealSense 点云页禁用: %s", _exc
    )


logger = logging.getLogger("MotorStudio.realsense_panel")

M_TO_CM = 100.0
DEFAULT_BOOK_TEMPLATE_PATH = (
    Path(__file__).resolve().parents[2] / "assets" / "book_photos" / "net2.png"
)


# Camera -> robot base extrinsic:
# camera +X -> robot +Y
# camera +Y -> robot -Z
# camera +Z -> robot -X
# camera origin in robot base: x=+5 cm, y=-28.5 cm, z=+15 cm.
CAMERA_TO_ROBOT_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "resources"
    / "config"
    / "camera_to_robot_transform.json"
)
DEFAULT_CAMERA_TO_ROBOT_MATRIX = np.array(
    [
        [0.0, 0.0, -1.0, 0.05],
        [1.0, 0.0, 0.0, -0.285],
        [0.0, -1.0, 0.0, 0.15],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)


def _validated_array(values, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != shape:
        raise ValueError(f"{name} shape must be {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite numbers")
    return array


def _load_camera_to_robot_transform(
    path: Path = CAMERA_TO_ROBOT_CONFIG_PATH,
) -> np.ndarray:
    """Load camera->robot extrinsic from JSON, falling back to built-in values."""

    matrix = DEFAULT_CAMERA_TO_ROBOT_MATRIX.copy()
    if not path.exists():
        return matrix

    try:
        with path.open("r", encoding="utf-8") as file:
            payload = json.load(file)
        if not isinstance(payload, dict):
            raise ValueError("JSON root must be an object")

        if "matrix" in payload:
            matrix = _validated_array(payload["matrix"], (4, 4), "matrix")
        else:
            rotation_values = payload.get(
                "rotation",
                payload.get("rotation_matrix", matrix[:3, :3]),
            )
            rotation = _validated_array(rotation_values, (3, 3), "rotation")

            translation = matrix[:3, 3]
            if "translation_m" in payload:
                translation = _validated_array(
                    payload["translation_m"],
                    (3,),
                    "translation_m",
                )
            elif "translation_cm" in payload:
                translation = (
                    _validated_array(payload["translation_cm"], (3,), "translation_cm")
                    / M_TO_CM
                )
            elif "xyz_m" in payload:
                translation = _validated_array(payload["xyz_m"], (3,), "xyz_m")

            matrix = np.eye(4, dtype=float)
            matrix[:3, :3] = rotation
            matrix[:3, 3] = translation

        return matrix.astype(float, copy=True)
    except Exception as exc:
        logger.warning(
            "加载相机到机械臂外参失败，使用内置默认值: %s (%s)",
            path,
            exc,
        )
        return matrix


CAMERA_TO_ROBOT_MATRIX = _load_camera_to_robot_transform()
CAMERA_TO_ROBOT_ROTATION = CAMERA_TO_ROBOT_MATRIX[:3, :3]
CAMERA_TO_ROBOT_TRANSLATION_M = CAMERA_TO_ROBOT_MATRIX[:3, 3]


def _rpy_to_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy, sy = math.cos(rz), math.sin(rz)
    rotation_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ],
        dtype=float,
    )
    rotation_y = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ],
        dtype=float,
    )
    rotation_z = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return rotation_z @ rotation_y @ rotation_x


def camera_point_to_robot_target(
    camera_point_m: Sequence[float],
) -> np.ndarray:
    """Map a RealSense camera-space point into robot base coordinates."""

    camera_point = np.asarray(camera_point_m, dtype=float).reshape(3)
    camera_point_h = np.ones(4, dtype=float)
    camera_point_h[:3] = camera_point
    return (CAMERA_TO_ROBOT_MATRIX @ camera_point_h)[:3]


def apply_tcp_offset_correction(
    robot_point_m: Sequence[float],
    tcp_offset: Sequence[float],
    rpy_rad: Sequence[float],
) -> np.ndarray:
    """Convert a picked TCP point into the underlying reference-frame target."""

    point = np.asarray(robot_point_m, dtype=float).reshape(3)
    offset = np.zeros(6, dtype=float)
    values = [] if tcp_offset is None else list(tcp_offset)
    for idx in range(min(6, len(values))):
        try:
            offset[idx] = float(values[idx])
        except (TypeError, ValueError):
            offset[idx] = 0.0
    rx, ry, rz = np.asarray(rpy_rad, dtype=float).reshape(3)
    tcp_rot = _rpy_to_matrix(offset[3], offset[4], offset[5])
    return point - _rpy_to_matrix(rx, ry, rz) @ tcp_rot.T @ offset[:3]


def _format_vec_m(values: Sequence[float]) -> str:
    vals = np.asarray(values, dtype=float).reshape(3)
    return f"X={vals[0]:.4f} m, Y={vals[1]:.4f} m, Z={vals[2]:.4f} m"


def _format_vec_cm(values: Sequence[float]) -> str:
    vals = np.asarray(values, dtype=float).reshape(3) * M_TO_CM
    return f"X={vals[0]:.2f} cm, Y={vals[1]:.2f} cm, Z={vals[2]:.2f} cm"


def _book_pick_point_from_polygon(polygon: np.ndarray) -> tuple[int, int]:
    points = np.asarray(polygon, dtype=float).reshape(-1, 2)
    if len(points) < 4:
        raise ValueError("书脊识别结果缺少四边形角点。")
    tl, tr, br, bl = points[:4]
    left = tl * (5.0 / 6.0) + bl * (1.0 / 6.0)
    right = tr * (5.0 / 6.0) + br * (1.0 / 6.0)
    point = (left + right) * 0.5
    return int(round(float(point[0]))), int(round(float(point[1])))


def _filter_depth_min(point_cloud, depth_min_m: float):
    if depth_min_m <= 0.0:
        return point_cloud

    points = np.asarray(point_cloud.points_xyz_m)
    keep = points[:, 2] >= depth_min_m
    return type(point_cloud)(
        points_xyz_m=points[keep],
        colors_rgb=(
            None
            if point_cloud.colors_rgb is None
            else np.asarray(point_cloud.colors_rgb)[keep]
        ),
        pixels_uv=np.asarray(point_cloud.pixels_uv)[keep],
        intrinsics=point_cloud.intrinsics,
        timestamp_ms=point_cloud.timestamp_ms,
        frame_number=point_cloud.frame_number,
    )


class RealSenseCaptureWorker(QThread):
    """Capture one RealSense RGB-D frame and convert it to a point cloud."""

    capture_finished = pyqtSignal(object, object)
    error_occurred = pyqtSignal(str)

    def __init__(
        self,
        *,
        serial: Optional[str],
        width: int,
        height: int,
        fps: int,
        warmup: int,
        timeout_ms: int,
        align_depth_to_color: bool,
        depth_min_m: float,
        depth_max_m: float,
        stride: int,
        include_color: bool,
        depth_width: int,
        depth_height: int,
        parent=None,
    ):
        super().__init__(parent)
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.warmup = warmup
        self.timeout_ms = timeout_ms
        self.align_depth_to_color = align_depth_to_color
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m
        self.stride = stride
        self.include_color = include_color
        self.depth_width = depth_width
        self.depth_height = depth_height

    def run(self):
        try:
            from el_a3_sdk.realsense import RealSenseD435

            with RealSenseD435(
                width=self.width,
                height=self.height,
                fps=self.fps,
                serial=self.serial or None,
                align_depth_to_color=self.align_depth_to_color,
                depth_width=self.depth_width,
                depth_height=self.depth_height,
            ) as camera:
                if self.isInterruptionRequested():
                    return
                camera.warmup(frame_count=self.warmup, timeout_ms=self.timeout_ms)
                if self.isInterruptionRequested():
                    return
                frame = camera.get_frame(timeout_ms=self.timeout_ms)
                if self.isInterruptionRequested():
                    return

            point_cloud = frame.to_point_cloud(
                max_depth_m=self.depth_max_m,
                stride=self.stride,
                include_color=self.include_color,
            )
            point_cloud = _filter_depth_min(point_cloud, self.depth_min_m)
            if point_cloud.size == 0:
                raise RuntimeError("点云为空，请调整深度范围或相机视角。")
            self.capture_finished.emit(frame, point_cloud)
        except Exception as exc:
            self.error_occurred.emit(str(exc))


def _make_book_match_config():
    from el_a3_sdk.realsense import BookSpineMatchConfig

    return BookSpineMatchConfig(
        match_confidence=0.65,
        center_tolerance_ratio=0.03,
        min_center_tolerance_px=45,
        frame_max_side=0,
        template_max_side=1200,
        min_good_matches=12,
        min_inliers=8,
        sift_ratio_test=0.72,
        sift_features=4000,
        acquire_match_confidence=0.50,
        acquire_min_good_matches=8,
        acquire_min_inliers=5,
        acquire_tile_columns=3,
        acquire_tile_overlap_ratio=0.20,
        search_scales=(1.0, 1.5, 2.0),
        max_scaled_frame_side=1800,
        use_clahe=True,
        clahe_clip_limit=2.0,
        clahe_tile_grid_size=8,
        roi_expand_ratio=0.45,
        roi_min_pad=60,
        roi_reacquire_after_misses=3,
        polygon_hold_frames=4,
        min_polygon_area_ratio=0.0002,
        max_polygon_area_ratio=0.75,
        min_polygon_fill_ratio=0.35,
        max_polygon_skew_ratio=3.0,
        max_polygon_jump_ratio=0.25,
        max_polygon_area_change_ratio=0.75,
        polygon_smoothing_alpha=0.25,
        keep_last_good_on_reject=True,
        center_smoothing_alpha=0.25,
        green_confirm_frames=2,
        red_confirm_frames=5,
    )


class BookSpineDetectWorker(QThread):
    """Keep capturing until the book spine is confidently detected."""

    detection_finished = pyqtSignal(object, object, object)
    status_message = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(
        self,
        *,
        serial: Optional[str],
        width: int,
        height: int,
        fps: int,
        warmup: int,
        timeout_ms: int,
        align_depth_to_color: bool,
        depth_min_m: float,
        depth_max_m: float,
        stride: int,
        include_color: bool,
        depth_width: int,
        depth_height: int,
        template_path: Path,
        config: object,
        parent=None,
    ):
        super().__init__(parent)
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.warmup = warmup
        self.timeout_ms = timeout_ms
        self.align_depth_to_color = align_depth_to_color
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m
        self.stride = stride
        self.include_color = include_color
        self.depth_width = depth_width
        self.depth_height = depth_height
        self.template_path = template_path
        self.config = config

    def run(self):
        try:
            from el_a3_sdk.realsense import BookSpineMatcher, RealSenseD435, match_with_fallback

            with RealSenseD435(
                width=self.width,
                height=self.height,
                fps=self.fps,
                serial=self.serial or None,
                align_depth_to_color=self.align_depth_to_color,
                depth_width=self.depth_width,
                depth_height=self.depth_height,
            ) as camera:
                if self.isInterruptionRequested():
                    return
                camera.warmup(frame_count=self.warmup, timeout_ms=self.timeout_ms)
                matcher = BookSpineMatcher.from_template_path(self.template_path, self.config)

                while not self.isInterruptionRequested():
                    frame = camera.get_frame(timeout_ms=self.timeout_ms)
                    if self.isInterruptionRequested():
                        return

                    point_cloud = frame.to_point_cloud(
                        max_depth_m=self.depth_max_m,
                        stride=self.stride,
                        include_color=self.include_color,
                    )
                    point_cloud = _filter_depth_min(point_cloud, self.depth_min_m)
                    if point_cloud.size == 0:
                        self.status_message.emit("点云为空，继续重采集...")
                        continue

                    result = match_with_fallback(
                        frame.color_bgr,
                        matcher.backends,
                        frame_max_side=self.config.frame_max_side,
                        search_rect=None,
                        match_confidence=self.config.match_confidence,
                        search_scales=self.config.search_scales,
                        max_scaled_frame_side=self.config.max_scaled_frame_side,
                        use_clahe=self.config.use_clahe,
                        clahe_clip_limit=self.config.clahe_clip_limit,
                        clahe_tile_grid_size=self.config.clahe_tile_grid_size,
                        min_good_matches=self.config.min_good_matches,
                        min_inliers=self.config.min_inliers,
                        polygon_validator=lambda _polygon, _shape: True,
                    )
                    if result.polygon is None:
                        self.status_message.emit(
                            f"识别未达标，重采集中: score={result.match_confidence:.2f} "
                            f"good={result.good_count} inliers={result.inlier_count}"
                        )
                        continue

                    self.detection_finished.emit(frame, point_cloud, result)
                    return
        except Exception as exc:
            self.error_occurred.emit(str(exc))


class RealSensePointPanel(QWidget):
    """Point cloud capture, target picking, and MoveL confirmation page."""

    DEFAULT_SERIAL = None
    # D435-class devices cannot usually run depth+color both at 1920x1080.
    # 1280x720 is the highest common RGB-D mode used for aligned point clouds.
    DEFAULT_WIDTH = 1920
    DEFAULT_HEIGHT = 1080
    DEFAULT_FPS = 30
    DEFAULT_WARMUP = 30
    DEFAULT_TIMEOUT_MS = 5000
    DEFAULT_DEPTH_MIN_M = 0.0
    DEFAULT_DEPTH_MAX_M = 2.0
    DEFAULT_STRIDE = 1
    DEFAULT_MAX_POINTS = 150000
    DEFAULT_POINT_SIZE = 2.0
    DEFAULT_ALIGN_DEPTH_TO_COLOR = True
    DEFAULT_INCLUDE_COLOR = True
    DEFAULT_FLIP_VIEW = True
    DEFAULT_DEPTH_WIDTH = 1280
    DEFAULT_DEPTH_HEIGHT = 720

    move_l_requested = pyqtSignal(list, float)
    log_message = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._capture_worker: Optional[RealSenseCaptureWorker] = None
        self._detect_worker: Optional[BookSpineDetectWorker] = None
        self._frame = None
        self._point_cloud = None
        self._display_indices: Optional[np.ndarray] = None
        self._display_points: Optional[np.ndarray] = None
        self._cloud_actor = None
        self._selected_actor = None
        self._picker = vtkPointPicker() if HAS_PYVISTA else None
        self._filter_installed = False
        self._selected_display_index: Optional[int] = None
        self._selected_display_point_m: Optional[np.ndarray] = None
        self._selected_raw_index: Optional[int] = None
        self._selected_camera_point_m: Optional[np.ndarray] = None
        self._selected_robot_target_raw_m: Optional[np.ndarray] = None
        self._target_robot_point_m: Optional[np.ndarray] = None
        self._book_spine_pick: Optional[object] = None
        self._tcp_offset = np.zeros(6, dtype=float)
        self._current_end_pose = None
        self._rpy_initialized = False
        self._arm_enabled = False
        self._viewer_visible = False
        self._viewer_widget = None
        self._serial = self.DEFAULT_SERIAL
        self._width = self.DEFAULT_WIDTH
        self._height = self.DEFAULT_HEIGHT
        self._depth_width = self.DEFAULT_DEPTH_WIDTH
        self._depth_height = self.DEFAULT_DEPTH_HEIGHT
        self._fps = self.DEFAULT_FPS
        self._warmup = self.DEFAULT_WARMUP
        self._timeout_ms = self.DEFAULT_TIMEOUT_MS
        self._depth_min_m = self.DEFAULT_DEPTH_MIN_M
        self._depth_max_m = self.DEFAULT_DEPTH_MAX_M
        self._stride = self.DEFAULT_STRIDE
        self._max_points = self.DEFAULT_MAX_POINTS
        self._point_size = self.DEFAULT_POINT_SIZE
        self._align_depth_to_color = self.DEFAULT_ALIGN_DEPTH_TO_COLOR
        self._include_color = self.DEFAULT_INCLUDE_COLOR
        self._flip_view = self.DEFAULT_FLIP_VIEW
        self._init_ui()

    def _init_ui(self):
        self._viewer_widget = self._create_viewer_widget()
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(6)

        controls_layout = QVBoxLayout()
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setSpacing(6)
        controls_layout.addWidget(self._create_capture_group())
        controls_layout.addWidget(self._create_book_group())
        controls_layout.addWidget(self._create_result_group())
        controls_layout.addWidget(self._create_move_group())
        controls_layout.addStretch()
        root.addLayout(controls_layout, 1)

        self.status_label = QLabel(tr("pc.ready"))
        root.addWidget(self.status_label)
        self._update_move_button_state()

    def _create_viewer_widget(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        self.rgb_preview_label = QLabel(container)
        self.rgb_preview_label.setFixedSize(300, 210)
        self.rgb_preview_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.rgb_preview_label.setScaledContents(False)
        self.rgb_preview_label.setStyleSheet(
            "QLabel { background: rgba(15, 18, 24, 210); border: 1px solid rgba(255, 255, 255, 120); }"
        )
        self.rgb_preview_label.hide()
        container.installEventFilter(self)
        if HAS_PYVISTA:
            self._plotter = None
            self._plotter_placeholder = QLabel(tr("pc.viewer_loading"))
            self._plotter_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self._plotter_placeholder.setWordWrap(True)
            layout.addWidget(self._plotter_placeholder)
        else:
            self._plotter = None
            self.no_plot_label = QLabel(tr("pc.no_pyvista"))
            self.no_plot_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.no_plot_label.setWordWrap(True)
            layout.addWidget(self.no_plot_label)
        return container

    def _ensure_plotter(self):
        if not HAS_PYVISTA or self._viewer_widget is None or self._plotter is not None:
            return

        layout = self._viewer_widget.layout()
        if layout is None:
            layout = QVBoxLayout(self._viewer_widget)
            layout.setContentsMargins(0, 0, 0, 0)

        pv.global_theme.allow_empty_mesh = True
        self._plotter = QtInteractor(self._viewer_widget, multi_samples=4)
        self._plotter.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        sc = SCENE_COLORS[ThemeManager.instance().theme]
        self._plotter.set_background(sc["bg_bottom"], top=sc["bg_top"])

        if hasattr(self, "_plotter_placeholder") and self._plotter_placeholder is not None:
            layout.removeWidget(self._plotter_placeholder)
            self._plotter_placeholder.deleteLater()
            self._plotter_placeholder = None

        layout.addWidget(self._plotter.interactor)
        self._position_rgb_preview()
        self._install_event_filter()
        self._reset_scene()

    def viewer_widget(self):
        return self._viewer_widget

    def show_viewer(self):
        self._viewer_visible = True
        self._ensure_plotter()
        if self._point_cloud is not None and self._display_points is not None:
            self._display_cloud(self._point_cloud)
        else:
            self._reset_scene()
        self._update_pick_mode_state()

    def hide_viewer(self):
        self._viewer_visible = False
        self._update_pick_mode_state()

    def _create_capture_group(self):
        self.capture_group = QGroupBox(tr("pc.capture_group"))
        layout = QGridLayout(self.capture_group)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(6)

        self.pick_mode_btn = QPushButton(tr("pc.pick_mode"))
        self.pick_mode_btn.setCheckable(True)
        self.pick_mode_btn.setChecked(True)
        self.pick_mode_btn.toggled.connect(
            lambda _checked: self._update_pick_mode_state()
        )
        layout.addWidget(self.pick_mode_btn, 0, 0)

        self.capture_btn = QPushButton(tr("pc.capture"))
        self.capture_btn.setObjectName("enableBtn")
        self.capture_btn.clicked.connect(self._start_capture)
        layout.addWidget(self.capture_btn, 0, 1)
        layout.setColumnStretch(2, 1)

        return self.capture_group

    def _create_book_group(self):
        self.book_group = QGroupBox(tr("pc.book_group"))
        layout = QGridLayout(self.book_group)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(6)

        self.template_label = QLabel(tr("pc.template"))
        self.template_edit = QLineEdit(str(DEFAULT_BOOK_TEMPLATE_PATH))
        layout.addWidget(self.template_label, 0, 0)
        layout.addWidget(self.template_edit, 0, 1, 1, 4)

        self.detect_btn = QPushButton(tr("pc.book_detect"))
        self.detect_btn.clicked.connect(self._detect_book_target)
        layout.addWidget(self.detect_btn, 0, 5)

        self.book_status_label = QLabel(tr("pc.book_ready"))
        self.book_status_label.setWordWrap(True)
        layout.addWidget(self.book_status_label, 1, 0, 1, 6)
        return self.book_group

    def _create_result_group(self):
        self.result_group = QGroupBox(tr("pc.result_group"))
        layout = QFormLayout(self.result_group)
        layout.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        self.pixel_label = QLabel(tr("pc.pixel"))
        self.robot_point_cm_label = QLabel(tr("pc.move_target_point_cm"))
        self.target_point_cm_label = QLabel(tr("pc.target_point_cm"))

        self.pixel_value = QLabel("--")
        self.move_target_point_cm_value = QLabel("--")
        self.target_point_cm_value = QLabel("--")

        layout.addRow(self.pixel_label, self.pixel_value)
        layout.addRow(self.robot_point_cm_label, self.move_target_point_cm_value)
        layout.addRow(self.target_point_cm_label, self.target_point_cm_value)
        return self.result_group

    def _create_move_group(self):
        self.move_group = QGroupBox(tr("pc.move_group"))
        layout = QGridLayout(self.move_group)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(6)

        self.rpy_labels = []
        self.rpy_spins = []
        for col, key in enumerate(("pc.rx", "pc.ry", "pc.rz")):
            label = QLabel(tr(key))
            spin = self._make_float_spin(-180.0, 180.0, 0.0, 1.0, "°")
            spin.valueChanged.connect(self._on_rpy_changed)
            self.rpy_labels.append(label)
            self.rpy_spins.append(spin)
            layout.addWidget(label, 0, col * 2)
            layout.addWidget(spin, 0, col * 2 + 1)

        self.duration_label = QLabel(tr("pc.duration"))
        self.duration_spin = self._make_float_spin(0.5, 30.0, 2.0, 0.5, " s")
        layout.addWidget(self.duration_label, 1, 0)
        layout.addWidget(self.duration_spin, 1, 1)

        row = QHBoxLayout()
        self.read_rpy_btn = QPushButton(tr("pc.read_rpy"))
        self.read_rpy_btn.clicked.connect(self._fill_current_rpy)
        row.addWidget(self.read_rpy_btn)

        self.move_btn = QPushButton(tr("pc.confirm_move"))
        self.move_btn.setObjectName("enableBtn")
        self.move_btn.clicked.connect(self._on_confirm_move)
        row.addWidget(self.move_btn)
        row.addStretch()
        layout.addLayout(row, 1, 2, 1, 4)
        return self.move_group

    @staticmethod
    def _make_int_spin(lo: int, hi: int, value: int, step: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(lo, hi)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setFixedWidth(105)
        return spin

    @staticmethod
    def _make_float_spin(
        lo: float,
        hi: float,
        value: float,
        step: float,
        suffix: str,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(3 if abs(step) < 0.1 else 2)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setFixedWidth(115)
        return spin

    def _start_capture(self):
        if self._capture_worker is not None and self._capture_worker.isRunning():
            return
        if self._depth_max_m <= self._depth_min_m:
            self._set_error(tr("pc.depth_range_error"))
            return

        self.capture_btn.setEnabled(False)
        self.capture_btn.setText(tr("pc.capturing"))
        self.status_label.setText(tr("pc.capturing"))
        self._clear_selection()

        self._capture_worker = RealSenseCaptureWorker(
            serial=self._serial,
            width=self._width,
            height=self._height,
            fps=self._fps,
            warmup=self._warmup,
            timeout_ms=self._timeout_ms,
            align_depth_to_color=self._align_depth_to_color,
            depth_min_m=self._depth_min_m,
            depth_max_m=self._depth_max_m,
            stride=self._stride,
            include_color=self._include_color,
            depth_width=self._depth_width,
            depth_height=self._depth_height,
            parent=self,
        )
        self._capture_worker.capture_finished.connect(self._on_capture_finished)
        self._capture_worker.error_occurred.connect(self._on_capture_error)
        self._capture_worker.finished.connect(self._on_capture_thread_finished)
        self._capture_worker.start()

    def _on_capture_finished(self, frame, point_cloud):
        self._frame = frame
        self._point_cloud = point_cloud
        self._book_spine_pick = None
        self.rgb_preview_label.hide()
        self._selected_display_index = None
        self._selected_raw_index = None
        self._selected_camera_point_m = None
        self._selected_robot_target_raw_m = None
        self._target_robot_point_m = None
        self._display_cloud(point_cloud)
        self.pick_mode_btn.setChecked(True)
        self.status_label.setText(
            tr("pc.capture_done", n=point_cloud.size, frame=frame.frame_number)
        )
        self.log_message.emit(
            tr("pc.capture_done", n=point_cloud.size, frame=frame.frame_number)
        )
        self._update_move_button_state()

    def _detect_book_target(self):
        try:
            if self._detect_worker is not None and self._detect_worker.isRunning():
                return

            template = Path(self.template_edit.text().strip() or str(DEFAULT_BOOK_TEMPLATE_PATH))
            self._detect_worker = BookSpineDetectWorker(
                serial=self._serial,
                width=self._width,
                height=self._height,
                fps=self._fps,
                warmup=self._warmup,
                timeout_ms=self._timeout_ms,
                align_depth_to_color=self._align_depth_to_color,
                depth_min_m=self._depth_min_m,
                depth_max_m=self._depth_max_m,
                stride=self._stride,
                include_color=self._include_color,
                depth_width=self._depth_width,
                depth_height=self._depth_height,
                template_path=template,
                config=_make_book_match_config(),
                parent=self,
            )
            self._detect_worker.detection_finished.connect(self._on_book_detection_finished)
            self._detect_worker.status_message.connect(self._on_book_detection_status)
            self._detect_worker.error_occurred.connect(self._on_capture_error)
            self._detect_worker.finished.connect(self._on_book_detection_thread_finished)

            self.capture_btn.setEnabled(False)
            self.detect_btn.setEnabled(False)
            self.capture_btn.setText(tr("pc.capturing"))
            self.status_label.setText(tr("pc.capturing"))
            self.book_status_label.setText("识别中，未达标会自动重采集...")
            self._clear_selection()
            self._detect_worker.start()
        except Exception as exc:
            self._on_capture_error(str(exc))

    def _on_capture_error(self, message: str):
        self._set_error(tr("pc.capture_error", msg=message))

    def _on_capture_thread_finished(self):
        self.capture_btn.setEnabled(True)
        self.capture_btn.setText(tr("pc.capture"))

    def _on_book_detection_status(self, message: str):
        self.book_status_label.setText(message)

    def _on_book_detection_finished(self, frame, point_cloud, pick):
        self._frame = frame
        self._point_cloud = point_cloud
        self._book_spine_pick = pick
        self.rgb_preview_label.hide()
        self._display_cloud(point_cloud)

        polygon = np.asarray(pick.polygon, dtype=float).reshape(-1, 2)
        u, v = _book_pick_point_from_polygon(polygon)
        camera_point = np.asarray(
            self._frame.point_at_pixel(u, v, max_depth_m=self._depth_max_m),
            dtype=float,
        )
        robot_point = camera_point_to_robot_target(camera_point)
        pixels = np.asarray([u, v], dtype=int)
        raw_index = self._nearest_point_index_from_pixel(pixels)
        self.book_status_label.setText(
            f"score={pick.match_confidence:.2f}  good={pick.good_count}  inliers={pick.inlier_count}"
        )
        self._show_book_preview(pick)
        self._apply_book_pick(raw_index, camera_point, robot_point, pixels, polygon)
        self.status_label.setText(tr("pc.book_point_selected"))

    def _on_book_detection_thread_finished(self):
        self.capture_btn.setEnabled(True)
        self.detect_btn.setEnabled(True)
        self.capture_btn.setText(tr("pc.capture"))

    def _display_cloud(self, point_cloud):
        self._ensure_plotter()
        if not HAS_PYVISTA or self._plotter is None:
            return

        points = np.asarray(point_cloud.points_xyz_m, dtype=np.float64)
        count = len(points)
        max_points = max(1, self._max_points)
        if count > max_points:
            indices = np.linspace(0, count - 1, max_points, dtype=np.int64)
        else:
            indices = np.arange(count, dtype=np.int64)

        display_points = points[indices].copy()
        if self._flip_view:
            display_points[:, 1] *= -1.0
            display_points[:, 2] *= -1.0

        self._display_indices = indices
        self._display_points = display_points
        colors = None
        if point_cloud.colors_rgb is not None:
            colors = np.asarray(point_cloud.colors_rgb, dtype=np.uint8)[indices]

        self._reset_scene()
        cloud_poly = pv.PolyData(display_points)
        cloud_poly["point_id"] = np.arange(len(display_points), dtype=np.int32)
        cloud_poly.verts = np.column_stack(
            (
                np.ones(len(display_points), dtype=np.int64),
                np.arange(len(display_points), dtype=np.int64),
            )
        ).ravel()
        point_size = float(self._point_size)
        if colors is not None and len(colors) == len(display_points):
            cloud_poly["rgb"] = colors
            self._cloud_actor = self._plotter.add_mesh(
                cloud_poly,
                scalars="rgb",
                rgb=True,
                point_size=point_size,
                render_points_as_spheres=True,
                name="realsense_cloud",
            )
        else:
            self._cloud_actor = self._plotter.add_mesh(
                cloud_poly,
                color="#8bd5ff",
                point_size=point_size,
                render_points_as_spheres=True,
                name="realsense_cloud",
            )

        if self._picker is not None and self._cloud_actor is not None:
            try:
                self._picker.InitializePickList()
                self._picker.AddPickList(self._cloud_actor)
                self._picker.PickFromListOn()
                self._picker.SetTolerance(0.01)
            except Exception:
                pass

        try:
            self._plotter.reset_camera()
            self._apply_initial_view()
            if (
                self._selected_display_index is not None
                and self._selected_display_index < len(self._display_points)
            ):
                self._add_selection_marker(
                    self._selected_display_index,
                    self._display_points[self._selected_display_index],
                )
            self._plotter.render()
        except Exception:
            pass

    def _camera_point_to_display_point(self, camera_point: Sequence[float]) -> np.ndarray:
        point = np.asarray(camera_point, dtype=float).reshape(3)
        if self._flip_view:
            point = point.copy()
            point[1] *= -1.0
            point[2] *= -1.0
        return point

    def _redisplay_current_cloud(self):
        if self._point_cloud is not None:
            self._display_cloud(self._point_cloud)
        self._update_pick_mode_state()

    def _reset_scene(self):
        if not HAS_PYVISTA or self._plotter is None:
            return
        try:
            self._plotter.clear()
            sc = SCENE_COLORS[ThemeManager.instance().theme]
            self._plotter.set_background(sc["bg_bottom"], top=sc["bg_top"])
            self._plotter.add_axes()
        except Exception:
            pass
        self._cloud_actor = None
        self._selected_actor = None

    def _install_event_filter(self):
        if not HAS_PYVISTA or self._plotter is None or self._filter_installed:
            return
        self._plotter.interactor.installEventFilter(self)
        self._filter_installed = True

    def eventFilter(self, obj, event):
        if obj is self._viewer_widget and event.type() == QEvent.Type.Resize:
            self._position_rgb_preview()
            return False

        if (
            not HAS_PYVISTA
            or self._plotter is None
            or obj is not self._plotter.interactor
            or not hasattr(self, "pick_mode_btn")
            or not self.pick_mode_btn.isChecked()
        ):
            return False

        if (
            event.type() == QEvent.Type.MouseButtonPress
            and event.button() == Qt.MouseButton.LeftButton
            and self._display_points is not None
        ):
            picked = self._pick_display_point(event)
            if picked is not None:
                display_index, display_point = picked
                self._select_display_point(display_index, display_point)
                return True
        if event.type() == QEvent.Type.MouseButtonPress and event.button() == Qt.MouseButton.RightButton:
            return False
        return False

    def _position_rgb_preview(self):
        if not hasattr(self, "rgb_preview_label") or self._viewer_widget is None:
            return
        margin = 14
        x = max(margin, self._viewer_widget.width() - self.rgb_preview_label.width() - margin)
        self.rgb_preview_label.move(x, margin)
        self.rgb_preview_label.raise_()

    def _show_book_preview(self, pick):
        if self._frame is None or not hasattr(self, "rgb_preview_label") or cv2 is None:
            return

        image_bgr = self._frame.color_bgr.copy()
        corners = np.asarray(pick.polygon, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(image_bgr, [corners], True, (0, 0, 255), 5, cv2.LINE_AA)
        u, v = _book_pick_point_from_polygon(pick.polygon)
        cv2.circle(image_bgr, (int(u), int(v)), 12, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(image_bgr, (int(u), int(v)), 15, (0, 0, 255), 3, cv2.LINE_AA)

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        height, width, channels = image_rgb.shape
        bytes_per_line = channels * width
        qimage = QImage(
            image_rgb.data,
            width,
            height,
            bytes_per_line,
            QImage.Format.Format_RGB888,
        ).copy()
        pixmap = QPixmap.fromImage(qimage).scaled(
            self.rgb_preview_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.rgb_preview_label.setPixmap(pixmap)
        self.rgb_preview_label.show()
        self._position_rgb_preview()

    def _pick_display_point(self, event) -> Optional[tuple[int, np.ndarray]]:
        if self._display_points is None or self._plotter is None:
            return None

        vx, vy = self._qt_to_vtk_coords(event)
        picked = self._pick_nearest_screen_point(vx, vy)
        if picked is not None:
            return picked, np.asarray(self._display_points[picked], dtype=float)

        if self._picker is None:
            return None

        try:
            hit = self._picker.Pick(vx, vy, 0, self._plotter.renderer)
            if not hit:
                self.status_label.setText(tr("pc.pick_miss"))
                return None

            pick_position = np.asarray(self._picker.GetPickPosition(), dtype=float)
            point_id = int(self._picker.GetPointId())
            if (
                self._display_indices is not None
                and 0 <= point_id < len(self._display_indices)
            ):
                display_point = self._picked_display_point(point_id, pick_position)
                return point_id, display_point
            world_point = pick_position
        except Exception as exc:
            logger.debug("点云选点失败: %s", exc)
            self.status_label.setText(tr("pc.pick_miss"))
            return None

        if self._display_indices is None:
            self.status_label.setText(tr("pc.pick_miss"))
            return None
        if world_point is not None and self._display_points is not None:
            distances = np.linalg.norm(self._display_points - world_point.reshape(1, 3), axis=1)
            display_index = int(np.argmin(distances))
            display_point = self._picked_display_point(display_index, world_point)
            return display_index, display_point

        try:
            pick_point = np.asarray(self._plotter.picked_point, dtype=float).reshape(3)
            if self._display_points is not None:
                distances = np.linalg.norm(self._display_points - pick_point.reshape(1, 3), axis=1)
                display_index = int(np.argmin(distances))
                display_point = self._picked_display_point(display_index, pick_point)
                return display_index, display_point
        except Exception:
            pass

        self.status_label.setText(tr("pc.pick_miss"))
        return None

    def _qt_to_vtk_coords(self, event) -> tuple[float, float]:
        x = float(event.position().x())
        y = float(self._plotter.interactor.height() - event.position().y() - 1)
        try:
            render_width, render_height = self._plotter.ren_win.GetSize()
            widget_width = max(1, self._plotter.interactor.width())
            widget_height = max(1, self._plotter.interactor.height())
            x *= float(render_width) / float(widget_width)
            y *= float(render_height) / float(widget_height)
        except Exception:
            pass
        return x, y

    def _picked_display_point(
        self,
        display_index: int,
        pick_position: np.ndarray,
    ) -> np.ndarray:
        if pick_position.shape == (3,) and np.all(np.isfinite(pick_position)):
            return pick_position.astype(float, copy=True)
        return np.asarray(self._display_points[display_index], dtype=float)

    def _pick_nearest_screen_point(self, x: float, y: float) -> Optional[int]:
        if self._display_points is None or len(self._display_points) == 0:
            return None

        try:
            renderer = self._plotter.renderer
            render_width, render_height = self._plotter.ren_win.GetSize()
            best_index = None
            best_dist_sq = float("inf")
            best_depth = float("inf")

            for index, point in enumerate(self._display_points):
                renderer.SetWorldPoint(
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                    1.0,
                )
                renderer.WorldToDisplay()
                sx, sy, sz = renderer.GetDisplayPoint()
                if (
                    not np.isfinite(sx)
                    or not np.isfinite(sy)
                    or not np.isfinite(sz)
                    or sx < 0.0
                    or sx >= render_width
                    or sy < 0.0
                    or sy >= render_height
                    or sz < 0.0
                    or sz > 1.0
                ):
                    continue

                dist_sq = (float(sx) - x) ** 2 + (float(sy) - y) ** 2
                if dist_sq < best_dist_sq or (
                    math.isclose(dist_sq, best_dist_sq) and float(sz) < best_depth
                ):
                    best_index = index
                    best_dist_sq = dist_sq
                    best_depth = float(sz)

            if best_index is None:
                return None

            point_size = max(1.0, float(self._point_size))
            threshold_px = max(12.0, point_size * 5.0)
            if best_dist_sq > threshold_px * threshold_px:
                logger.debug(
                    "屏幕最近点超过阈值: distance=%.2f px threshold=%.2f px",
                    math.sqrt(best_dist_sq),
                    threshold_px,
                )
                return None
            return int(best_index)
        except Exception as exc:
            logger.debug("屏幕空间点云选点失败: %s", exc)
            return None

    def _select_display_point(self, display_index: int, display_point: np.ndarray):
        if self._point_cloud is None or self._display_indices is None:
            return

        raw_index = int(self._display_indices[display_index])
        camera_point = np.asarray(self._point_cloud.points_xyz_m[raw_index], dtype=float)
        self._selected_display_index = int(display_index)
        self._selected_display_point_m = np.asarray(display_point, dtype=float).reshape(3)
        self._selected_raw_index = raw_index
        self._selected_camera_point_m = camera_point
        self._selected_robot_target_raw_m = camera_point_to_robot_target(camera_point)
        self._target_robot_point_m = np.asarray(
            self._selected_robot_target_raw_m,
            dtype=float,
        ).reshape(3)
        pixels = np.asarray(self._point_cloud.pixels_uv[raw_index], dtype=int)

        if pixels.shape == (2,) and np.all(pixels >= 0):
            self.pixel_value.setText(f"u={int(pixels[0])}, v={int(pixels[1])}")
        else:
            self.pixel_value.setText("--")
        self.move_target_point_cm_value.setText(_format_vec_cm(self._target_robot_point_m))

        self._add_selection_marker(display_index, self._selected_display_point_m)
        self._recompute_target_from_selection()
        self.status_label.setText(tr("pc.point_selected"))

    def _nearest_point_index_from_pixel(self, pixel_uv: np.ndarray) -> int:
        if self._point_cloud is None:
            return -1
        pixels = np.asarray(self._point_cloud.pixels_uv, dtype=int)
        if pixels.size == 0:
            return -1
        diffs = pixels - pixel_uv.reshape(1, 2)
        distances = np.sum(diffs.astype(float) ** 2, axis=1)
        return int(np.argmin(distances))

    def _apply_book_pick(
        self,
        raw_index: int,
        camera_point: np.ndarray,
        robot_point: np.ndarray,
        pixels: np.ndarray,
        corners: np.ndarray,
    ):
        if self._point_cloud is None:
            return

        if raw_index < 0:
            raw_index = self._nearest_point_index_from_pixel(pixels)
        if raw_index >= 0 and self._display_indices is not None:
            display_candidates = np.where(self._display_indices == raw_index)[0]
            if len(display_candidates) > 0:
                display_index = int(display_candidates[0])
                self._select_display_point(
                    display_index,
                    self._display_points[display_index],
                )
                return

        self._selected_display_index = None
        self._selected_display_point_m = self._camera_point_to_display_point(camera_point)
        self._selected_raw_index = int(raw_index)
        self._selected_camera_point_m = np.asarray(camera_point, dtype=float).reshape(3)
        self._selected_robot_target_raw_m = np.asarray(robot_point, dtype=float).reshape(3)
        self._target_robot_point_m = np.asarray(robot_point, dtype=float).reshape(3)
        self.pixel_value.setText(f"u={int(pixels[0])}, v={int(pixels[1])}")
        self.move_target_point_cm_value.setText(_format_vec_cm(robot_point))
        corrected_target = apply_tcp_offset_correction(
            self._selected_robot_target_raw_m,
            self._tcp_offset,
            np.array(
                [
                    math.radians(self.rpy_spins[0].value()),
                    math.radians(self.rpy_spins[1].value()),
                    math.radians(self.rpy_spins[2].value()),
                ],
                dtype=float,
            ),
        )
        self.target_point_cm_value.setText(_format_vec_cm(corrected_target))
        self._add_book_marker(corners, self._selected_display_point_m)
        self._update_move_button_state()

    def _add_book_marker(self, corners: np.ndarray, display_point: np.ndarray):
        if not HAS_PYVISTA or self._plotter is None:
            return
        try:
            if self._selected_actor is not None:
                self._plotter.remove_actor(self._selected_actor)
                self._selected_actor = None
            self._selected_actor = self._plotter.add_points(
                np.asarray(display_point, dtype=float).reshape(1, 3),
                color="#ffcc00",
                point_size=max(10.0, float(self._point_size) * 6.0),
                render_points_as_spheres=True,
                name="book_detect_point",
            )
            if corners is not None and len(corners) == 4:
                world = []
                for u, v in np.asarray(corners, dtype=float):
                    if self._frame is None:
                        continue
                    try:
                        world_point = self._frame.point_at_pixel(
                            int(round(u)),
                            int(round(v)),
                            max_depth_m=self._depth_max_m,
                        )
                        world.append(self._camera_point_to_display_point(world_point))
                    except Exception:
                        continue
                if len(world) >= 2:
                    self._plotter.add_lines(
                        np.asarray(world, dtype=float),
                        color="#ff8800",
                        width=3,
                        name="book_detect_box",
                    )
            self._plotter.render()
        except Exception:
            pass

    def _add_selection_marker(
        self,
        display_index: int,
        display_point: Optional[np.ndarray] = None,
    ):
        if not HAS_PYVISTA or self._plotter is None or self._display_points is None:
            return
        if display_point is None:
            display_point = self._display_points[display_index]
        point = np.asarray(display_point, dtype=float)
        if self._selected_actor is not None:
            try:
                self._plotter.remove_actor(self._selected_actor)
            except Exception:
                pass
            self._selected_actor = None

        try:
            marker_points = np.asarray(point, dtype=float).reshape(1, 3)
            marker_size = max(10.0, float(self._point_size) * 6.0)
            self._selected_actor = self._plotter.add_points(
                marker_points,
                color="#ffcc00",
                point_size=marker_size,
                render_points_as_spheres=True,
                name="selected_realsense_point",
            )
            self._plotter.render()
        except Exception:
            pass

    def _recompute_target_from_selection(self):
        if self._selected_robot_target_raw_m is None:
            return

        target_rpy = np.array(
            [
                math.radians(self.rpy_spins[0].value()),
                math.radians(self.rpy_spins[1].value()),
                math.radians(self.rpy_spins[2].value()),
            ],
            dtype=float,
        )
        if self._target_robot_point_m is not None:
            self.move_target_point_cm_value.setText(_format_vec_cm(self._target_robot_point_m))
        corrected_target = apply_tcp_offset_correction(
            self._selected_robot_target_raw_m,
            self._tcp_offset,
            target_rpy,
        )
        self.target_point_cm_value.setText(_format_vec_cm(corrected_target))
        self._update_move_button_state()

    def _apply_initial_view(self):
        if not HAS_PYVISTA or self._plotter is None:
            return
        try:
            self._plotter.view_xy()
            self._plotter.camera.zoom(1.2)
        except Exception:
            pass

    def _update_pick_mode_state(self):
        if not HAS_PYVISTA or self._plotter is None:
            return
        if not hasattr(self, "pick_mode_btn"):
            return
        try:
            if self._picker is not None:
                if self.pick_mode_btn.isChecked() and self._viewer_visible:
                    self._picker.PickFromListOn()
                else:
                    self._picker.PickFromListOff()
        except Exception as exc:
            logger.debug("更新点选模式失败: %s", exc)

    def set_tcp_offset(self, tcp_offset):
        source = [] if tcp_offset is None else list(tcp_offset)
        values = np.asarray(source, dtype=float)
        if values.shape != (6,):
            normalized = np.zeros(6, dtype=float)
            for idx in range(min(6, len(values))):
                normalized[idx] = float(values[idx])
            values = normalized
        self._tcp_offset = values
        if self._selected_robot_target_raw_m is not None:
            self._recompute_target_from_selection()

    def update_current_end_pose(self, end_pose):
        self._current_end_pose = end_pose
        if not self._rpy_initialized:
            self._set_rpy_from_pose(end_pose)

    def set_arm_enabled(self, enabled: bool):
        self._arm_enabled = bool(enabled)
        self._update_move_button_state()

    def _fill_current_rpy(self):
        if self._current_end_pose is None:
            self.status_label.setText(tr("pc.no_pose"))
            return
        self._set_rpy_from_pose(self._current_end_pose)
        self._rpy_initialized = True
        self.status_label.setText(tr("pc.rpy_loaded"))

    def _set_rpy_from_pose(self, pose):
        if pose is None:
            return
        values_deg = [
            math.degrees(float(pose.rx)),
            math.degrees(float(pose.ry)),
            math.degrees(float(pose.rz)),
        ]
        for spin, value in zip(self.rpy_spins, values_deg):
            blocker = QSignalBlocker(spin)
            spin.setValue(value)
            del blocker
        if self._selected_robot_target_raw_m is not None:
            self._recompute_target_from_selection()

    def _mark_rpy_initialized(self):
        self._rpy_initialized = True

    def _on_rpy_changed(self):
        self._mark_rpy_initialized()
        if self._selected_robot_target_raw_m is not None:
            self._recompute_target_from_selection()

    def _on_confirm_move(self):
        if self._target_robot_point_m is None:
            self.status_label.setText(tr("pc.no_target"))
            return
        pose = [
            float(self._target_robot_point_m[0]),
            float(self._target_robot_point_m[1]),
            float(self._target_robot_point_m[2]),
            math.radians(self.rpy_spins[0].value()),
            math.radians(self.rpy_spins[1].value()),
            math.radians(self.rpy_spins[2].value()),
        ]
        duration = float(self.duration_spin.value())
        self.move_l_requested.emit(pose, duration)
        self.status_label.setText(tr("pc.movel_sent"))
        self.log_message.emit(tr("pc.movel_sent"))

    def _clear_selection(self, keep_result: bool = False):
        if not keep_result:
            self._selected_display_index = None
            self._selected_display_point_m = None
            self._selected_raw_index = None
            self._selected_camera_point_m = None
            self._selected_robot_target_raw_m = None
            self._target_robot_point_m = None
            self.pixel_value.setText("--")
            self.move_target_point_cm_value.setText("--")
            self.target_point_cm_value.setText("--")
        if self._selected_actor is not None and self._plotter is not None:
            try:
                self._plotter.remove_actor(self._selected_actor)
                self._plotter.render()
            except Exception:
                pass
        self._selected_actor = None
        self._update_move_button_state()

    def _update_move_button_state(self):
        can_move = self._arm_enabled and self._target_robot_point_m is not None
        if hasattr(self, "move_btn"):
            self.move_btn.setEnabled(can_move)

    def _set_error(self, message: str):
        self.status_label.setText(message)
        self.error_occurred.emit(message)

    def apply_theme(self):
        if HAS_PYVISTA and self._plotter is not None:
            try:
                sc = SCENE_COLORS[ThemeManager.instance().theme]
                self._plotter.set_background(sc["bg_bottom"], top=sc["bg_top"])
                self._plotter.render()
            except Exception:
                pass
        elif HAS_PYVISTA and hasattr(self, "_plotter_placeholder") and self._plotter_placeholder is not None:
            try:
                sc = SCENE_COLORS[ThemeManager.instance().theme]
                self._plotter_placeholder.setStyleSheet(
                    f"color: {sc['subtext']}; font-size: 12px; padding: 12px;"
                )
            except Exception:
                pass

    def retranslate_ui(self):
        self.capture_group.setTitle(tr("pc.capture_group"))
        self.pick_mode_btn.setText(tr("pc.pick_mode"))
        self.capture_btn.setText(tr("pc.capture"))
        self.book_group.setTitle(tr("pc.book_group"))
        self.template_label.setText(tr("pc.template"))
        self.detect_btn.setText(tr("pc.book_detect"))
        if self._frame is None:
            self.book_status_label.setText(tr("pc.book_ready"))

        self.result_group.setTitle(tr("pc.result_group"))
        self.pixel_label.setText(tr("pc.pixel"))
        self.robot_point_cm_label.setText(tr("pc.move_target_point_cm"))
        self.target_point_cm_label.setText(tr("pc.target_point_cm"))
        self.move_group.setTitle(tr("pc.move_group"))
        for label, key in zip(self.rpy_labels, ("pc.rx", "pc.ry", "pc.rz")):
            label.setText(tr(key))
        self.duration_label.setText(tr("pc.duration"))
        self.read_rpy_btn.setText(tr("pc.read_rpy"))
        self.move_btn.setText(tr("pc.confirm_move"))
        if hasattr(self, "no_plot_label"):
            self.no_plot_label.setText(tr("pc.no_pyvista"))
        if hasattr(self, "_plotter_placeholder") and self._plotter_placeholder is not None:
            self._plotter_placeholder.setText(tr("pc.viewer_loading"))

    def cleanup(self):
        if self._capture_worker is not None and self._capture_worker.isRunning():
            self._capture_worker.requestInterruption()
            self._capture_worker.wait(1500)


__all__ = [
    "RealSensePointPanel",
    "camera_point_to_robot_target",
]
