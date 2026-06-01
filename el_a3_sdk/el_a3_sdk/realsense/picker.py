"""Open3D point picking helpers for RealSense point clouds."""

from __future__ import annotations

from dataclasses import dataclass
import copy
import json
import os
from pathlib import Path
import sys
from typing import Dict, Optional, Union

import numpy as np

from .camera import PointCloud
from .geometry import RigidTransform


@dataclass(frozen=True)
class SelectedPoint:
    """Point selected from a point cloud."""

    picked_index: int
    camera_point_m: np.ndarray
    pixel_uv: Optional[np.ndarray] = None

    def to_dict(self) -> Dict[str, object]:
        return {
            "picked_index": int(self.picked_index),
            "camera_point_m": self.camera_point_m.astype(float).tolist(),
            **(
                {"pixel_uv": self.pixel_uv.astype(int).tolist()}
                if self.pixel_uv is not None
                else {}
            ),
        }


@dataclass(frozen=True)
class SelectedTarget:
    """Point selected from a point cloud plus an optional transform."""

    picked_index: int
    camera_point_m: np.ndarray
    pixel_uv: Optional[np.ndarray] = None
    target_point_m: Optional[np.ndarray] = None

    def to_dict(self) -> Dict[str, object]:
        payload = {
            "picked_index": int(self.picked_index),
            "camera_point_m": self.camera_point_m.astype(float).tolist(),
        }
        if self.pixel_uv is not None:
            payload["pixel_uv"] = self.pixel_uv.astype(int).tolist()
        if self.target_point_m is not None:
            payload["target_point_m"] = self.target_point_m.astype(float).tolist()
        return payload


def import_open3d():
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError(
            "缺少 open3d，无法打开点云选点窗口。当前 Python 是 "
            f"{sys.executable}。请确认已进入 conda 环境 lingarm，或执行 "
            "`conda run -n lingarm python scripts/camera_test/pick_realsense_point.py`。"
        ) from exc
    return o3d


def prepare_view_cloud(point_cloud, flip_view: bool):
    if not flip_view:
        return point_cloud

    view_cloud = copy.deepcopy(point_cloud)
    view_cloud.transform(
        [
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1],
        ]
    )
    return view_cloud


def point_cloud_to_open3d(point_cloud: PointCloud, flip_view: bool = True):
    return point_cloud.to_open3d(
        flip_y_for_viewer=flip_view,
        flip_z_for_viewer=flip_view,
    )


def pick_point_index(
    point_cloud: PointCloud,
    *,
    point_size: float = 1.0,
    flip_view: bool = True,
    window_name: str = "Pick Point",
    width: int = 1280,
    height: int = 720,
) -> int:
    o3d = import_open3d()
    display_cloud = point_cloud_to_open3d(point_cloud, flip_view=flip_view)
    if len(display_cloud.points) == 0:
        raise RuntimeError("点云为空，请调整深度范围或相机视角。")

    print("Open3D 选点操作：")
    print("  Shift + 左键：选择一个点")
    print("  Shift + 右键：撤销上一次选择")
    print("  Q 或 Esc：结束选点")

    visualizer = o3d.visualization.VisualizerWithEditing()
    if not visualizer.create_window(window_name, width=width, height=height):
        raise RuntimeError("创建 Open3D 选点窗口失败。")

    try:
        visualizer.add_geometry(display_cloud)
        render_options = visualizer.get_render_option()
        render_options.point_size = point_size
        render_options.background_color = np.asarray([0.02, 0.02, 0.02])
        visualizer.run()
        picked_points = visualizer.get_picked_points()
    finally:
        visualizer.destroy_window()

    if not picked_points:
        raise RuntimeError("没有选择任何点。")
    if len(picked_points) > 1:
        print(f"选中了 {len(picked_points)} 个点，将使用第一个。")
    return int(picked_points[0])


def pick_point(
    point_cloud: PointCloud,
    *,
    point_size: float = 1.0,
    flip_view: bool = True,
    window_name: str = "Pick Point",
    width: int = 1280,
    height: int = 720,
) -> SelectedPoint:
    """Pick a point from a point cloud and return the 3D camera-space point."""

    picked_index = pick_point_index(
        point_cloud,
        point_size=point_size,
        flip_view=flip_view,
        window_name=window_name,
        width=width,
        height=height,
    )
    camera_point_m = np.asarray(point_cloud.points_xyz_m)[picked_index]
    pixel_uv = None
    if hasattr(point_cloud, "pixels_uv"):
        try:
            candidate_uv = np.asarray(point_cloud.pixels_uv[picked_index], dtype=int)
            if candidate_uv.shape == (2,) and np.all(candidate_uv >= 0):
                pixel_uv = candidate_uv
        except Exception:
            pixel_uv = None
    return SelectedPoint(
        picked_index=picked_index,
        camera_point_m=np.asarray(camera_point_m, dtype=float),
        pixel_uv=pixel_uv,
    )


def pick_target(
    point_cloud: PointCloud,
    *,
    transform: Optional[RigidTransform] = None,
    point_size: float = 1.0,
    flip_view: bool = True,
    window_name: str = "Pick Target Point",
    width: int = 1280,
    height: int = 720,
) -> SelectedTarget:
    """Pick a point and optionally transform it to another coordinate frame."""

    picked = pick_point(
        point_cloud,
        point_size=point_size,
        flip_view=flip_view,
        window_name=window_name,
        width=width,
        height=height,
    )
    target_point_m = None if transform is None else transform.transform_point(picked.camera_point_m)
    return SelectedTarget(
        picked_index=picked.picked_index,
        camera_point_m=picked.camera_point_m,
        pixel_uv=picked.pixel_uv,
        target_point_m=target_point_m,
    )


def make_display_cloud(point_cloud: PointCloud, *, flip_view: bool = True):
    return point_cloud.to_open3d(
        flip_y_for_viewer=flip_view,
        flip_z_for_viewer=flip_view,
    )


def save_point_cloud(point_cloud: PointCloud, path: Union[str, bytes, os.PathLike]):
    o3d = import_open3d()
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if hasattr(point_cloud, "to_open3d"):
        point_cloud = point_cloud.to_open3d()
    if not o3d.io.write_point_cloud(str(output_path), point_cloud):
        raise RuntimeError(f"保存点云失败：{output_path}")
    print(f"已保存点云：{output_path}")


def save_selected_point(path: Union[str, bytes, os.PathLike], selected: SelectedPoint) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(selected.to_dict(), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"已保存选点结果：{output_path}")
    return output_path


def save_selected_target(path: Union[str, bytes, os.PathLike], selected: SelectedTarget) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(selected.to_dict(), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"已保存目标点结果：{output_path}")
    return output_path


__all__ = [
    "import_open3d",
    "make_display_cloud",
    "pick_point",
    "pick_point_index",
    "pick_target",
    "point_cloud_to_open3d",
    "prepare_view_cloud",
    "save_point_cloud",
    "save_selected_point",
    "save_selected_target",
    "SelectedPoint",
    "SelectedTarget",
]
