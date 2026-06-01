"""单关节拖拽控制器 — 参考 robot_viewer JointDragControls.js

核心算法：点击连杆 → 找到父 revolute 关节 → 鼠标拖动时将屏幕位移
投影到关节旋转平面，计算有符号角度增量 → 更新目标关节角（含限位）。
"""

import numpy as np
from typing import Dict, Optional, Tuple

from MotorStudio.utils.urdf_loader import UrdfModel, UrdfJoint, _make_transform


class JointDragController:
    """无 UI 依赖的拖拽状态机 + 运动学计算。"""

    def __init__(self, model: UrdfModel, joint_names_ordered: list[str]):
        self._model = model
        self._joint_names = joint_names_ordered
        self.target_angles: Dict[str, float] = {n: 0.0 for n in joint_names_ordered}

        self._dragging = False
        self._active_joint: Optional[UrdfJoint] = None
        self._hit_depth: float = 0.0
        self._prev_world_pt: Optional[np.ndarray] = None

    @property
    def is_dragging(self) -> bool:
        return self._dragging

    @property
    def active_joint(self) -> Optional[UrdfJoint]:
        return self._active_joint

    # ------------------------------------------------------------------
    # 拖拽生命周期
    # ------------------------------------------------------------------

    def begin_drag(self, link_name: str, world_point: np.ndarray, depth: float) -> bool:
        """尝试开始拖拽。返回 True 表示找到了可拖拽的 revolute 关节。"""
        joint = self._model.find_ancestor_revolute_joint(link_name)
        if joint is None or joint.name not in self.target_angles:
            return False
        self._active_joint = joint
        self._hit_depth = depth
        self._prev_world_pt = world_point.copy()
        self._dragging = True
        return True

    def update_drag(self, world_point: np.ndarray, view_dir: np.ndarray) -> Optional[float]:
        """鼠标移动时调用。返回更新后的关节角度值，或 None。"""
        if not self._dragging or self._active_joint is None or self._prev_world_pt is None:
            return None

        joint = self._active_joint
        transforms = self._model.compute_link_transforms(self.target_angles)
        axis_world, pivot_world = self._get_joint_world_info(joint, transforms)

        delta = compute_revolute_delta(
            axis_world, pivot_world,
            self._prev_world_pt, world_point,
            view_dir,
        )

        new_angle = self.target_angles[joint.name] + delta
        new_angle = float(np.clip(new_angle, joint.lower, joint.upper))
        self.target_angles[joint.name] = new_angle
        self._prev_world_pt = world_point.copy()
        return new_angle

    def end_drag(self):
        """结束拖拽。"""
        self._dragging = False
        self._active_joint = None
        self._prev_world_pt = None

    def sync_from_feedback(self, angles_dict: Dict[str, float]):
        """将目标角度重置为机械臂反馈的实际角度。"""
        for name in self._joint_names:
            if name in angles_dict:
                self.target_angles[name] = angles_dict[name]

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------

    def _get_joint_world_info(
        self, joint: UrdfJoint, transforms: Dict[str, np.ndarray]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (axis_world, pivot_world)。"""
        parent_T = transforms.get(joint.parent_link, np.eye(4))
        joint_T = _make_transform(joint.origin_xyz, joint.origin_rpy)
        world_joint_T = parent_T @ joint_T
        axis_world = world_joint_T[:3, :3] @ joint.axis
        axis_world = axis_world / (np.linalg.norm(axis_world) + 1e-12)
        pivot_world = world_joint_T[:3, 3]
        return axis_world, pivot_world


# ======================================================================
# 纯函数：旋转增量计算（对标 robot_viewer getRevoluteDelta）
# ======================================================================

def compute_revolute_delta(
    joint_axis_world: np.ndarray,
    pivot_world: np.ndarray,
    prev_point: np.ndarray,
    curr_point: np.ndarray,
    view_dir: np.ndarray,
) -> float:
    """在关节旋转平面上计算两个世界坐标点之间的有符号角度增量。

    当视线与旋转平面几乎共面时 (|viewDir·normal| <= 0.3)，
    退化为沿相机辅助向量的线性映射以避免抖动。
    """
    normal = joint_axis_world
    cos_view = abs(np.dot(view_dir, normal))

    if cos_view > 0.3:
        return _revolute_delta_plane(normal, pivot_world, prev_point, curr_point)

    # 退化情况：视线与旋转平面几乎平行，用相机右向量做线性映射
    up_guess = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(view_dir, up_guess)) > 0.95:
        up_guess = np.array([0.0, 1.0, 0.0])
    cam_right = np.cross(view_dir, up_guess)
    cam_right /= np.linalg.norm(cam_right) + 1e-12
    cam_up = np.cross(cam_right, view_dir)
    cam_up /= np.linalg.norm(cam_up) + 1e-12

    diff = curr_point - prev_point
    screen_delta = np.dot(diff, cam_right) + np.dot(diff, cam_up)
    sign = 1.0 if np.dot(np.cross(view_dir, normal), cam_right) >= 0 else -1.0
    return float(sign * screen_delta * 2.0)


def _revolute_delta_plane(
    normal: np.ndarray,
    pivot: np.ndarray,
    p0: np.ndarray,
    p1: np.ndarray,
) -> float:
    """将两点投影到关节旋转平面并计算有符号夹角。"""
    v0 = p0 - pivot - np.dot(p0 - pivot, normal) * normal
    v1 = p1 - pivot - np.dot(p1 - pivot, normal) * normal

    n0 = np.linalg.norm(v0)
    n1 = np.linalg.norm(v1)
    if n0 < 1e-10 or n1 < 1e-10:
        return 0.0

    v0 /= n0
    v1 /= n1

    cross = np.cross(v0, v1)
    dot = float(np.clip(np.dot(v0, v1), -1.0, 1.0))
    return float(np.arctan2(np.dot(cross, normal), dot))
