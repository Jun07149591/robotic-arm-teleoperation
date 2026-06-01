"""Intel RealSense RGB-D camera and point cloud helpers.

The data returned by this module stays in the RealSense camera coordinate
system: x points right in the color image, y points down, and z points forward
from the camera. Viewer-only axis flips are kept separate from the stored data.
"""

from __future__ import annotations

from dataclasses import dataclass
import sys
from pathlib import Path
from typing import Any, Dict, Iterator, Literal, Optional, Tuple, Union

import numpy as np

ColorOrder = Literal["rgb", "bgr"]
DepthUnit = Literal["m", "raw"]


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for the stream used to deproject depth pixels."""

    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float


@dataclass(frozen=True)
class PointCloud:
    """Point cloud generated from an aligned RGB-D frame.

    ``points_xyz_m`` uses meters in the RealSense camera coordinate system.
    ``pixels_uv`` stores the source image pixel for every point when the point
    came directly from the depth map.
    """

    points_xyz_m: np.ndarray
    colors_rgb: Optional[np.ndarray]
    pixels_uv: np.ndarray
    intrinsics: CameraIntrinsics
    timestamp_ms: float
    frame_number: int

    @property
    def size(self) -> int:
        return int(self.points_xyz_m.shape[0])

    def save_npz(self, path: Union[str, Path]) -> Path:
        """Save points, colors, source pixels, and metadata as compressed NPZ."""

        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        payload: Dict[str, object] = {
            "points_xyz_m": self.points_xyz_m,
            "pixels_uv": self.pixels_uv,
            "timestamp_ms": np.array(self.timestamp_ms, dtype=np.float64),
            "frame_number": np.array(self.frame_number, dtype=np.int64),
            "intrinsics": np.array(
                [
                    self.intrinsics.width,
                    self.intrinsics.height,
                    self.intrinsics.fx,
                    self.intrinsics.fy,
                    self.intrinsics.ppx,
                    self.intrinsics.ppy,
                ],
                dtype=np.float64,
            ),
        }
        if self.colors_rgb is not None:
            payload["colors_rgb"] = self.colors_rgb

        np.savez_compressed(output_path, **payload)
        return output_path

    def save_ply(self, path: Union[str, Path]) -> Path:
        """Save the point cloud as a binary little-endian PLY file."""

        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        has_color = self.colors_rgb is not None
        header_lines = [
            "ply",
            "format binary_little_endian 1.0",
            f"element vertex {self.size}",
            "property float x",
            "property float y",
            "property float z",
        ]
        if has_color:
            header_lines.extend(
                [
                    "property uchar red",
                    "property uchar green",
                    "property uchar blue",
                ]
            )
        header_lines.append("end_header")
        header = "\n".join(header_lines) + "\n"

        if has_color:
            data = np.empty(
                self.size,
                dtype=[
                    ("x", "<f4"),
                    ("y", "<f4"),
                    ("z", "<f4"),
                    ("red", "u1"),
                    ("green", "u1"),
                    ("blue", "u1"),
                ],
            )
            data["red"] = self.colors_rgb[:, 0]
            data["green"] = self.colors_rgb[:, 1]
            data["blue"] = self.colors_rgb[:, 2]
        else:
            data = np.empty(
                self.size,
                dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")],
            )

        data["x"] = self.points_xyz_m[:, 0]
        data["y"] = self.points_xyz_m[:, 1]
        data["z"] = self.points_xyz_m[:, 2]

        with output_path.open("wb") as file:
            file.write(header.encode("ascii"))
            data.tofile(file)

        return output_path

    def to_open3d(
        self,
        *,
        flip_y_for_viewer: bool = False,
        flip_z_for_viewer: bool = False,
    ):
        """Convert this cloud to an Open3D ``PointCloud`` object.

        Flips only affect the returned Open3D geometry for display. The stored
        arrays and saved files remain in raw RealSense camera coordinates.
        """

        try:
            import open3d as o3d
        except ImportError as exc:
            raise RuntimeError(
                "缺少 open3d，无法进行 3D 点云可视化。请先运行：pip install open3d"
            ) from exc

        points = self.points_xyz_m.astype(np.float64, copy=True)
        if flip_y_for_viewer:
            points[:, 1] *= -1.0
        if flip_z_for_viewer:
            points[:, 2] *= -1.0

        open3d_cloud = o3d.geometry.PointCloud()
        open3d_cloud.points = o3d.utility.Vector3dVector(points)

        if self.colors_rgb is not None:
            colors = self.colors_rgb.astype(np.float64, copy=False) / 255.0
            open3d_cloud.colors = o3d.utility.Vector3dVector(colors)

        return open3d_cloud


@dataclass(frozen=True)
class RGBDFrame:
    """One synchronized RGB-D frame captured from a RealSense camera."""

    color_bgr: np.ndarray
    depth_raw: np.ndarray
    depth_m: np.ndarray
    depth_scale: float
    timestamp_ms: float
    frame_number: int
    intrinsics: CameraIntrinsics
    depth_aligned_to_color: bool = True

    @property
    def color_rgb(self) -> np.ndarray:
        """Return a copy of the color image in RGB channel order."""

        return self.color_bgr[:, :, ::-1].copy()

    def color(self, order: ColorOrder = "rgb") -> np.ndarray:
        """Return the color image in RGB or BGR channel order."""

        if order == "rgb":
            return self.color_rgb
        if order == "bgr":
            return self.color_bgr.copy()
        raise ValueError(f"不支持的彩色图通道顺序：{order}")

    def depth(self, unit: DepthUnit = "m") -> np.ndarray:
        """Return depth in meters or in raw sensor units."""

        if unit == "m":
            return self.depth_m.copy()
        if unit == "raw":
            return self.depth_raw.copy()
        raise ValueError(f"不支持的深度单位：{unit}")

    def valid_depth_mask(self, max_depth_m: Optional[float] = None) -> np.ndarray:
        """Return a mask of finite, positive depth pixels."""

        mask = np.isfinite(self.depth_m) & (self.depth_m > 0.0)
        if max_depth_m is not None:
            mask &= self.depth_m <= max_depth_m
        return mask

    def point_at_pixel(
        self,
        u: int,
        v: int,
        *,
        search_radius: int = 5,
        max_depth_m: Optional[float] = None,
    ) -> np.ndarray:
        """Return the camera-space 3D point for one pixel, in meters.

        If the clicked pixel has invalid depth, the nearest valid pixel within
        ``search_radius`` is used.
        """

        height, width = self.depth_m.shape
        if not (0 <= u < width and 0 <= v < height):
            raise ValueError(f"像素坐标超出图像范围：u={u}, v={v}, size={width}x{height}")

        valid_mask = self.valid_depth_mask(max_depth_m=max_depth_m)
        if valid_mask[v, u]:
            depth = float(self.depth_m[v, u])
            return self._deproject_pixel(u, v, depth)

        if search_radius < 0:
            raise ValueError("search_radius 必须大于等于 0。")
        if search_radius == 0:
            raise ValueError(f"点击像素没有有效深度：u={u}, v={v}")

        u_min = max(0, u - search_radius)
        u_max = min(width, u + search_radius + 1)
        v_min = max(0, v - search_radius)
        v_max = min(height, v + search_radius + 1)
        local_mask = valid_mask[v_min:v_max, u_min:u_max]
        if not np.any(local_mask):
            raise ValueError(
                f"点击像素附近没有有效深度：u={u}, v={v}, search_radius={search_radius}"
            )

        local_v, local_u = np.nonzero(local_mask)
        candidate_u = local_u + u_min
        candidate_v = local_v + v_min
        distances = (candidate_u - u) ** 2 + (candidate_v - v) ** 2
        best_index = int(np.argmin(distances))
        best_u = int(candidate_u[best_index])
        best_v = int(candidate_v[best_index])
        depth = float(self.depth_m[best_v, best_u])
        return self._deproject_pixel(best_u, best_v, depth)

    def _deproject_pixel(self, u: int, v: int, depth_m: float) -> np.ndarray:
        x = (float(u) - self.intrinsics.ppx) * depth_m / self.intrinsics.fx
        y = (float(v) - self.intrinsics.ppy) * depth_m / self.intrinsics.fy
        return np.array([x, y, depth_m], dtype=float)

    def depth_colormap_bgr(
        self,
        max_depth_m: float = 2.0,
        invalid_color_bgr: Tuple[int, int, int] = (0, 0, 0),
    ) -> np.ndarray:
        """Convert depth to an OpenCV BGR pseudo-color image."""

        if max_depth_m <= 0:
            raise ValueError("max_depth_m 必须为正数。")

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "保存或预览深度伪彩色图需要 opencv-python。请先安装它。"
            ) from exc

        depth_normalized = np.clip(self.depth_m / max_depth_m, 0.0, 1.0)
        depth_u8 = (depth_normalized * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
        depth_color[~self.valid_depth_mask()] = invalid_color_bgr
        return depth_color

    def to_point_cloud(
        self,
        *,
        max_depth_m: Optional[float] = None,
        stride: int = 1,
        include_color: bool = True,
    ) -> PointCloud:
        """Deproject the depth image to a point cloud."""

        if stride < 1:
            raise ValueError("stride 必须大于等于 1。")

        depth = self.depth_m[::stride, ::stride]
        height, width = depth.shape
        v_grid, u_grid = np.indices((height, width), dtype=np.float32)
        u_grid *= stride
        v_grid *= stride

        valid = np.isfinite(depth) & (depth > 0.0)
        if max_depth_m is not None:
            valid &= depth <= max_depth_m

        z = depth[valid].astype(np.float32, copy=False)
        u = u_grid[valid]
        v = v_grid[valid]
        x = (u - self.intrinsics.ppx) * z / self.intrinsics.fx
        y = (v - self.intrinsics.ppy) * z / self.intrinsics.fy
        points_xyz_m = np.column_stack((x, y, z)).astype(np.float32, copy=False)
        pixels_uv = np.column_stack((u, v)).astype(np.int32, copy=False)

        colors_rgb: Optional[np.ndarray] = None
        if (
            include_color
            and self.depth_aligned_to_color
            and self.color_bgr.shape[:2] == self.depth_m.shape
        ):
            color_rgb = self.color_rgb[::stride, ::stride]
            colors_rgb = color_rgb[valid].reshape(-1, 3).astype(np.uint8, copy=True)

        return PointCloud(
            points_xyz_m=points_xyz_m,
            colors_rgb=colors_rgb,
            pixels_uv=pixels_uv,
            intrinsics=self.intrinsics,
            timestamp_ms=self.timestamp_ms,
            frame_number=self.frame_number,
        )

    def save_images(
        self,
        directory: Union[str, Path],
        *,
        prefix: str = "realsense",
        depth_vis_max_m: float = 2.0,
    ) -> Dict[str, Path]:
        """Save color, raw depth, and depth visualization images."""

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "保存图像需要 opencv-python。请先安装它，例如 pip install opencv-python。"
            ) from exc

        output_dir = Path(directory)
        output_dir.mkdir(parents=True, exist_ok=True)

        paths = {
            "color": output_dir / f"{prefix}_color.png",
            "depth_raw": output_dir / f"{prefix}_depth_raw.png",
            "depth_vis": output_dir / f"{prefix}_depth_vis.png",
        }
        cv2.imwrite(str(paths["color"]), self.color_bgr)
        cv2.imwrite(str(paths["depth_raw"]), self.depth_raw)
        cv2.imwrite(
            str(paths["depth_vis"]),
            self.depth_colormap_bgr(max_depth_m=depth_vis_max_m),
        )
        return paths


def import_pyrealsense2():
    """Import ``pyrealsense2`` with a readable error message."""

    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError(
            "RealSense 相机需要 pyrealsense2。当前 Python 是 "
            f"{sys.executable}。请确认已进入 conda 环境 lingarm，或执行 "
            "`conda run -n lingarm python scripts/camera_test/pick_realsense_point.py`。"
        ) from exc
    return rs


class RealSenseD435:
    """Control an Intel RealSense D435-compatible RGB-D camera."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        serial: Optional[str] = None,
        align_depth_to_color: bool = True,
        depth_width: Optional[int] = None,
        depth_height: Optional[int] = None,
    ) -> None:
        self.width = width
        self.height = height
        self.depth_width = int(depth_width if depth_width is not None else width)
        self.depth_height = int(depth_height if depth_height is not None else height)
        self.fps = fps
        self.serial = serial
        self.align_depth_to_color = align_depth_to_color

        self._rs = import_pyrealsense2()
        self._pipeline = self._rs.pipeline()
        self._config = self._rs.config()
        if serial:
            self._config.enable_device(serial)

        self._config.enable_stream(
            self._rs.stream.depth,
            self.depth_width,
            self.depth_height,
            self._rs.format.z16,
            fps,
        )
        self._config.enable_stream(
            self._rs.stream.color,
            width,
            height,
            self._rs.format.bgr8,
            fps,
        )

        self._align = (
            self._rs.align(self._rs.stream.color) if align_depth_to_color else None
        )
        self._profile: Optional[Any] = None
        self._depth_scale: Optional[float] = None

    @property
    def is_started(self) -> bool:
        return self._profile is not None

    @property
    def depth_scale(self) -> float:
        if self._depth_scale is None:
            raise RuntimeError("相机尚未启动，无法读取 depth scale。")
        return self._depth_scale

    def start(self) -> None:
        """Start color and depth streams."""

        if self._profile is not None:
            return

        try:
            self._profile = self._pipeline.start(self._config)
        except Exception as exc:
            message = str(exc)
            if "Couldn't resolve requests" in message:
                raise RuntimeError(
                    "RealSense 不支持当前 RGB-D 流配置："
                    f"color={self.width}x{self.height}@{self.fps}, "
                    f"depth={self.depth_width}x{self.depth_height}@{self.fps}。"
                    "请改用相机同时支持的深度和彩色分辨率。"
                ) from exc
            raise
        depth_sensor = self._profile.get_device().first_depth_sensor()
        self._depth_scale = float(depth_sensor.get_depth_scale())

    def stop(self) -> None:
        """Stop the RealSense pipeline."""

        if self._profile is None:
            return

        self._pipeline.stop()
        self._profile = None
        self._depth_scale = None

    def warmup(self, frame_count: int = 30, timeout_ms: int = 5000) -> None:
        """Drop startup frames so auto exposure can settle."""

        if frame_count <= 0:
            return
        self._ensure_started()
        for _ in range(frame_count):
            self._pipeline.wait_for_frames(timeout_ms)

    def get_frame(self, timeout_ms: int = 5000) -> RGBDFrame:
        """Capture one synchronized RGB-D frame."""

        self._ensure_started()
        frames = self._pipeline.wait_for_frames(timeout_ms)
        if self._align is not None:
            frames = self._align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            raise RuntimeError("未能同时获取 RGB 和深度帧，请检查 RealSense 连接状态。")

        color_bgr = np.asanyarray(color_frame.get_data()).copy()
        depth_raw = np.asanyarray(depth_frame.get_data()).copy()
        depth_scale = self.depth_scale
        depth_m = depth_raw.astype(np.float32) * depth_scale
        intrinsics = self._get_frame_intrinsics(depth_frame, color_frame)

        return RGBDFrame(
            color_bgr=color_bgr,
            depth_raw=depth_raw,
            depth_m=depth_m,
            depth_scale=depth_scale,
            timestamp_ms=float(color_frame.get_timestamp()),
            frame_number=int(color_frame.get_frame_number()),
            intrinsics=intrinsics,
            depth_aligned_to_color=self.align_depth_to_color,
        )

    def iter_frames(self, timeout_ms: int = 5000) -> Iterator[RGBDFrame]:
        """Yield RGB-D frames until the caller stops iteration."""

        self._ensure_started()
        while True:
            yield self.get_frame(timeout_ms=timeout_ms)

    def get_rgbd_frame(self, timeout_ms: int = 5000) -> RGBDFrame:
        """Alias for ``get_frame``."""

        return self.get_frame(timeout_ms=timeout_ms)

    def get_color_image(
        self,
        *,
        timeout_ms: int = 5000,
        order: ColorOrder = "rgb",
    ) -> np.ndarray:
        """Capture one frame and return only the color image."""

        return self.get_frame(timeout_ms=timeout_ms).color(order=order)

    def get_depth_image(
        self,
        *,
        timeout_ms: int = 5000,
        unit: DepthUnit = "m",
    ) -> np.ndarray:
        """Capture one frame and return only the depth image."""

        return self.get_frame(timeout_ms=timeout_ms).depth(unit=unit)

    def get_point_cloud(
        self,
        *,
        timeout_ms: int = 5000,
        max_depth_m: Optional[float] = None,
        stride: int = 1,
        include_color: bool = True,
    ) -> PointCloud:
        """Capture one RGB-D frame and return its point cloud."""

        frame = self.get_frame(timeout_ms=timeout_ms)
        return frame.to_point_cloud(
            max_depth_m=max_depth_m,
            stride=stride,
            include_color=include_color,
        )

    def _ensure_started(self) -> None:
        if self._profile is None:
            self.start()

    def _get_frame_intrinsics(self, depth_frame, color_frame) -> CameraIntrinsics:
        if self.align_depth_to_color:
            video_profile = color_frame.profile.as_video_stream_profile()
        else:
            video_profile = depth_frame.profile.as_video_stream_profile()

        intrinsics = video_profile.intrinsics
        return CameraIntrinsics(
            width=int(intrinsics.width),
            height=int(intrinsics.height),
            fx=float(intrinsics.fx),
            fy=float(intrinsics.fy),
            ppx=float(intrinsics.ppx),
            ppy=float(intrinsics.ppy),
        )

    def __enter__(self) -> "RealSenseD435":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()


__all__ = [
    "CameraIntrinsics",
    "ColorOrder",
    "DepthUnit",
    "PointCloud",
    "RGBDFrame",
    "RealSenseD435",
    "import_pyrealsense2",
]
