#!/usr/bin/env python3
"""Run a no-build Windows RViz simulation for the EL-A3 robot."""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from tkinter import BOTH, LEFT, RIGHT, W, X, Tk, DoubleVar, Frame, Label, Scale, Button, StringVar
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


ALL_JOINTS = [f"L{i}_joint" for i in range(1, 8)]
IK_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
IK_TARGET_LINK = "end_effector"


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis_xyz: Tuple[float, float, float]
    lower: float
    upper: float


def _parse_vector(value: str | None, default: Sequence[float]) -> Tuple[float, float, float]:
    if not value:
        return (float(default[0]), float(default[1]), float(default[2]))
    parts = [float(part) for part in value.split()]
    if len(parts) != 3:
        raise ValueError(f"Expected a 3D vector, got: {value!r}")
    return (parts[0], parts[1], parts[2])


def _load_urdf_joints(urdf_path: Path) -> List[UrdfJoint]:
    root = ET.parse(urdf_path).getroot()
    joints: List[UrdfJoint] = []
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue

        origin = joint.find("origin")
        axis = joint.find("axis")
        joints.append(
            UrdfJoint(
                name=joint.attrib.get("name", ""),
                joint_type=joint.attrib.get("type", "fixed"),
                parent_link=parent.attrib.get("link", ""),
                child_link=child.attrib.get("link", ""),
                origin_xyz=_parse_vector(origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0)),
                origin_rpy=_parse_vector(origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0)),
                axis_xyz=_parse_vector(axis.attrib.get("xyz") if axis is not None else None, (0, 0, 1)),
                lower=float(joint.find("limit").attrib.get("lower", "-3.14159")) if joint.find("limit") is not None else -3.14159,
                upper=float(joint.find("limit").attrib.get("upper", "3.14159")) if joint.find("limit") is not None else 3.14159,
            )
        )
    return joints


def _rpy_to_matrix(rpy: Sequence[float]) -> Tuple[Tuple[float, float, float], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _axis_angle_matrix(axis: Sequence[float], angle: float) -> Tuple[Tuple[float, float, float], ...]:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    x, y, z = x / norm, y / norm, z / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return (
        (c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s),
        (y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s),
        (z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c),
    )


def _matmul3(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> Tuple[Tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(left[row][idx] * right[idx][col] for idx in range(3)) for col in range(3))
        for row in range(3)
    )


def _matvec3(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> Tuple[float, float, float]:
    return tuple(sum(matrix[row][idx] * vector[idx] for idx in range(3)) for row in range(3))


def _matrix_to_quaternion(matrix: Sequence[Sequence[float]]) -> Tuple[float, float, float, float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2][1] - matrix[1][2]) / scale
        qy = (matrix[0][2] - matrix[2][0]) / scale
        qz = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        qw = (matrix[2][1] - matrix[1][2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0][1] + matrix[1][0]) / scale
        qz = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        qw = (matrix[0][2] - matrix[2][0]) / scale
        qx = (matrix[0][1] + matrix[1][0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        qw = (matrix[1][0] - matrix[0][1]) / scale
        qx = (matrix[0][2] + matrix[2][0]) / scale
        qy = (matrix[1][2] + matrix[2][1]) / scale
        qz = 0.25 * scale

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def _quaternion_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3)

    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=float,
    )


def _rotation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    error_matrix = target @ current.T
    trace = float(np.trace(error_matrix))
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    angle = math.acos(cos_angle)
    if angle < 1e-6:
        return np.zeros(3)

    axis = np.array(
        [
            error_matrix[2, 1] - error_matrix[1, 2],
            error_matrix[0, 2] - error_matrix[2, 0],
            error_matrix[1, 0] - error_matrix[0, 1],
        ],
        dtype=float,
    )
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        return np.zeros(3)
    return axis / axis_norm * angle


def _transform_from_joint(joint: UrdfJoint, angle: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = np.array(_rpy_to_matrix(joint.origin_rpy), dtype=float)
    transform[:3, 3] = np.array(joint.origin_xyz, dtype=float)

    if joint.joint_type in {"revolute", "continuous"}:
        rotation = np.eye(4)
        rotation[:3, :3] = np.array(_axis_angle_matrix(joint.axis_xyz, angle), dtype=float)
        return transform @ rotation

    if joint.joint_type == "prismatic":
        translation = np.eye(4)
        translation[:3, 3] = np.array(joint.axis_xyz, dtype=float) * angle
        return transform @ translation

    return transform


class IKSolver:
    def __init__(self, joints: List[UrdfJoint], target_link: str = IK_TARGET_LINK) -> None:
        self._joints = joints
        self._target_link = target_link
        self._children: Dict[str, List[UrdfJoint]] = {}
        self._joint_map = {joint.name: joint for joint in joints}
        for joint in joints:
            self._children.setdefault(joint.parent_link, []).append(joint)
        self._chain = self._build_chain("base_link", target_link)
        self._ik_joint_names = [name for name in IK_JOINTS if name in {joint.name for joint in self._chain}]
        self._lower = np.array([self._joint_map[name].lower for name in self._ik_joint_names], dtype=float)
        self._upper = np.array([self._joint_map[name].upper for name in self._ik_joint_names], dtype=float)

    @property
    def joint_names(self) -> List[str]:
        return list(self._ik_joint_names)

    def _build_chain(self, root_link: str, target_link: str) -> List[UrdfJoint]:
        found = self._find_chain(root_link, target_link, [])
        if found is None:
            raise RuntimeError(f"Could not find kinematic chain from {root_link} to {target_link}")
        return found

    def _find_chain(self, current_link: str, target_link: str, chain: List[UrdfJoint]) -> List[UrdfJoint] | None:
        if current_link == target_link:
            return chain
        for joint in self._children.get(current_link, []):
            result = self._find_chain(joint.child_link, target_link, chain + [joint])
            if result is not None:
                return result
        return None

    def forward(self, positions: Sequence[float]) -> np.ndarray:
        joint_positions = dict(zip(ALL_JOINTS, positions))
        transform = np.eye(4)
        for joint in self._chain:
            transform = transform @ _transform_from_joint(joint, joint_positions.get(joint.name, 0.0))
        return transform

    def solve(
        self,
        seed_positions: Sequence[float],
        target_xyz: Sequence[float],
        target_quaternion: Sequence[float],
        max_iterations: int = 120,
    ) -> Tuple[List[float], bool, float, float, int]:
        current = np.array(list(seed_positions), dtype=float)
        q = np.array([current[ALL_JOINTS.index(name)] for name in self._ik_joint_names], dtype=float)
        q = np.clip(q, self._lower, self._upper)
        target_pos = np.array(target_xyz, dtype=float)
        target_rot = _quaternion_to_matrix(target_quaternion)
        damping = 0.06
        step_limit = 0.16
        pos_error_norm = float("inf")
        rot_error_norm = float("inf")

        for iteration in range(max_iterations):
            current = self._positions_from_q(current, q)
            transform = self.forward(current)
            pos_error = target_pos - transform[:3, 3]
            rot_error = _rotation_error(transform[:3, :3], target_rot)
            error = np.concatenate([pos_error, 0.45 * rot_error])
            pos_error_norm = float(np.linalg.norm(pos_error))
            rot_error_norm = float(np.linalg.norm(rot_error))
            if pos_error_norm < 0.004 and rot_error_norm < 0.035:
                return current.tolist(), True, pos_error_norm, rot_error_norm, iteration + 1

            jacobian = self._numeric_jacobian(current, target_rot)
            jacobian[3:, :] *= 0.45
            lhs = jacobian @ jacobian.T + (damping * damping) * np.eye(6)
            delta = jacobian.T @ np.linalg.solve(lhs, error)
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > step_limit:
                delta *= step_limit / delta_norm

            q = np.clip(q + delta, self._lower, self._upper)

        current = self._positions_from_q(current, q)
        return current.tolist(), False, pos_error_norm, rot_error_norm, max_iterations

    def _positions_from_q(self, positions: np.ndarray, q: np.ndarray) -> np.ndarray:
        updated = positions.copy()
        for value, name in zip(q, self._ik_joint_names):
            updated[ALL_JOINTS.index(name)] = value
        return updated

    def _numeric_jacobian(self, positions: np.ndarray, target_rot: np.ndarray) -> np.ndarray:
        base_transform = self.forward(positions)
        base_pos = base_transform[:3, 3]
        base_rot = base_transform[:3, :3]
        jacobian = np.zeros((6, len(self._ik_joint_names)), dtype=float)
        eps = 1e-4

        for col, name in enumerate(self._ik_joint_names):
            perturbed = positions.copy()
            idx = ALL_JOINTS.index(name)
            perturbed[idx] = min(self._joint_map[name].upper, max(self._joint_map[name].lower, perturbed[idx] + eps))
            actual_eps = perturbed[idx] - positions[idx]
            if abs(actual_eps) < 1e-8:
                perturbed[idx] = min(self._joint_map[name].upper, max(self._joint_map[name].lower, positions[idx] - eps))
                actual_eps = perturbed[idx] - positions[idx]
            if abs(actual_eps) < 1e-8:
                continue

            perturbed_transform = self.forward(perturbed)
            jacobian[:3, col] = (perturbed_transform[:3, 3] - base_pos) / actual_eps
            rot_delta = _rotation_error(base_rot, perturbed_transform[:3, :3])
            jacobian[3:, col] = rot_delta / actual_eps

        return jacobian


class JointStateMotion(Node):
    def __init__(self, speed_scale: float, loop: bool, robot_description: str, joints: List[UrdfJoint]) -> None:
        super().__init__("el_a3_windows_joint_state_motion")
        self._joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        description_qos = QoSProfile(depth=1)
        description_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        description_qos.reliability = ReliabilityPolicy.RELIABLE
        self._description_publisher = self.create_publisher(String, "/robot_description", description_qos)
        self._description_msg = String(data=robot_description)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._joints = joints
        self._description_timer = self.create_timer(1.0, self._publish_description)
        self._timer = self.create_timer(1.0 / 60.0, self._tick)
        self._start = time.monotonic()
        self._speed = max(0.05, speed_scale)
        self._loop = loop
        self._manual_mode = False
        self.done = False
        self._positions_lock = threading.Lock()
        self._manual_positions = [0.0] * len(ALL_JOINTS)
        self._sequence = [
            (0.0, [0.0, 0.75, -0.85, 0.0, 0.35, 0.0, 0.20]),
            (4.0, [0.45, 1.05, -1.25, -0.35, 0.55, 0.30, 0.55]),
            (8.0, [-0.45, 0.85, -1.05, 0.35, -0.35, -0.30, 0.05]),
            (12.0, [0.20, 1.30, -1.60, 0.60, 0.80, -0.45, 0.35]),
            (16.0, [0.0, 0.75, -0.85, 0.0, 0.35, 0.0, 0.20]),
        ]
        self._period = self._sequence[-1][0]
        self._home_positions = list(self._sequence[0][1])
        self._manual_positions = list(self._home_positions)
        self._publish_description()

    def _publish_description(self) -> None:
        self._description_publisher.publish(self._description_msg)

    def set_manual_mode(self, enabled: bool) -> None:
        self._manual_mode = enabled

    def set_manual_positions(self, positions: Sequence[float]) -> None:
        with self._positions_lock:
            self._manual_positions = list(positions[: len(ALL_JOINTS)])

    def get_manual_positions(self) -> List[float]:
        with self._positions_lock:
            return list(self._manual_positions)

    def reset_home(self) -> None:
        self.set_manual_positions(self._home_positions)

    def _tick(self) -> None:
        if self._manual_mode:
            pos = self.get_manual_positions()
        else:
            elapsed = (time.monotonic() - self._start) * self._speed
            if self._loop:
                t = elapsed % self._period
            else:
                t = min(elapsed, self._period)
                if elapsed >= self._period:
                    self.done = True
            pos = self._sample(t)

        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ALL_JOINTS
        msg.position = pos
        msg.velocity = [0.0] * len(pos)
        msg.effort = [0.0] * len(pos)
        self._joint_state_publisher.publish(msg)
        self._publish_transforms(stamp, dict(zip(ALL_JOINTS, pos)))

    def _publish_transforms(self, stamp, joint_positions: Dict[str, float]) -> None:
        transforms: List[TransformStamped] = []
        for joint in self._joints:
            angle = joint_positions.get(joint.name, 0.0)
            origin_rotation = _rpy_to_matrix(joint.origin_rpy)
            rotation = origin_rotation
            translation = joint.origin_xyz

            if joint.joint_type in {"revolute", "continuous"}:
                rotation = _matmul3(origin_rotation, _axis_angle_matrix(joint.axis_xyz, angle))
            elif joint.joint_type == "prismatic":
                axis_offset = _matvec3(origin_rotation, joint.axis_xyz)
                translation = (
                    translation[0] + axis_offset[0] * angle,
                    translation[1] + axis_offset[1] * angle,
                    translation[2] + axis_offset[2] * angle,
                )

            qx, qy, qz, qw = _matrix_to_quaternion(rotation)
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = joint.parent_link
            transform.child_frame_id = joint.child_link
            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)

        self._tf_broadcaster.sendTransform(transforms)

    def _sample(self, t: float) -> List[float]:
        for idx in range(len(self._sequence) - 1):
            t0, p0 = self._sequence[idx]
            t1, p1 = self._sequence[idx + 1]
            if t <= t1:
                alpha = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                alpha = 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, alpha)))
                return [a + (b - a) * alpha for a, b in zip(p0, p1)]
        return list(self._sequence[-1][1])


class JointControlUI:
    def __init__(self, node: JointStateMotion, joints: List[UrdfJoint]) -> None:
        self._node = node
        self._joints = [joint for joint in joints if joint.joint_type in {"revolute", "continuous"}]
        self._vars: List[DoubleVar] = []
        self._root = Tk()
        self._root.title("EL-A3 关节控制")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        header = Frame(self._root)
        header.pack(fill=X, padx=10, pady=(10, 4))
        Label(header, text="拖动滑块控制机械臂关节，RViz 会实时跟随。").pack(anchor=W)

        body = Frame(self._root)
        body.pack(fill=BOTH, expand=True, padx=10, pady=4)

        for index, joint in enumerate(self._joints):
            row = Frame(body)
            row.pack(fill=X, pady=2)
            Label(row, text=joint.name, width=14, anchor=W).pack(side=LEFT)
            value = DoubleVar(value=node.get_manual_positions()[index])
            self._vars.append(value)
            scale = Scale(
                row,
                from_=joint.lower,
                to=joint.upper,
                orient="horizontal",
                resolution=0.001,
                length=360,
                variable=value,
                command=lambda _val, i=index: self._on_slider(i),
            )
            scale.pack(side=LEFT, fill=X, expand=True, padx=6)
            Label(row, textvariable=value, width=10, anchor=W).pack(side=RIGHT)

        button_row = Frame(self._root)
        button_row.pack(fill=X, padx=10, pady=(4, 10))
        Button(button_row, text="回零位", command=self._home).pack(side=LEFT, padx=(0, 6))
        Button(button_row, text="关闭", command=self._on_close).pack(side=RIGHT)

        self._node.set_manual_mode(True)
        self._node.set_manual_positions(self._collect_positions())

    def _collect_positions(self) -> List[float]:
        return [var.get() for var in self._vars]

    def _on_slider(self, _index: int) -> None:
        self._node.set_manual_positions(self._collect_positions())

    def _home(self) -> None:
        self._node.reset_home()
        for var, value in zip(self._vars, self._node.get_manual_positions()):
            var.set(value)

    def _on_close(self) -> None:
        self._root.quit()
        self._root.destroy()

    def mainloop(self) -> None:
        self._root.mainloop()


class IKControlUI:
    def __init__(self, node: JointStateMotion, joints: List[UrdfJoint]) -> None:
        self._node = node
        self._solver = IKSolver(joints)
        self._solving = False
        self._vars: Dict[str, DoubleVar] = {}

        self._root = Tk()
        self._root.title("EL-A3 末端位姿 IK 控制")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._status = StringVar(value="Ready")
        self._joint_status = StringVar(value="")

        header = Frame(self._root)
        header.pack(fill=X, padx=10, pady=(10, 4))
        Label(header, text="拖动 x/y/z 和四元数滑块，后台 IK 会求解 L1-L6 并驱动 RViz。").pack(anchor=W)

        body = Frame(self._root)
        body.pack(fill=BOTH, expand=True, padx=10, pady=4)

        start_transform = self._solver.forward(self._node.get_manual_positions())
        start_quat = _matrix_to_quaternion(start_transform[:3, :3])
        defaults = {
            "x": float(start_transform[0, 3]),
            "y": float(start_transform[1, 3]),
            "z": float(start_transform[2, 3]),
            "qx": start_quat[0],
            "qy": start_quat[1],
            "qz": start_quat[2],
            "qw": start_quat[3],
            "L7": self._node.get_manual_positions()[6],
        }
        specs = [
            ("x", -0.30, 0.35, 0.001),
            ("y", -0.35, 0.35, 0.001),
            ("z", -0.15, 0.45, 0.001),
            ("qx", -1.0, 1.0, 0.001),
            ("qy", -1.0, 1.0, 0.001),
            ("qz", -1.0, 1.0, 0.001),
            ("qw", -1.0, 1.0, 0.001),
            ("L7", -1.5708, 1.5708, 0.001),
        ]

        for key, lower, upper, resolution in specs:
            row = Frame(body)
            row.pack(fill=X, pady=2)
            Label(row, text=key, width=8, anchor=W).pack(side=LEFT)
            value = DoubleVar(value=defaults[key])
            self._vars[key] = value
            scale = Scale(
                row,
                from_=lower,
                to=upper,
                orient="horizontal",
                resolution=resolution,
                length=420,
                variable=value,
                command=lambda _val: self._request_solve(),
            )
            scale.pack(side=LEFT, fill=X, expand=True, padx=6)
            Label(row, textvariable=value, width=10, anchor=W).pack(side=RIGHT)

        status_row = Frame(self._root)
        status_row.pack(fill=X, padx=10, pady=(4, 2))
        Label(status_row, textvariable=self._status, anchor=W).pack(fill=X)
        Label(status_row, textvariable=self._joint_status, anchor=W).pack(fill=X)

        button_row = Frame(self._root)
        button_row.pack(fill=X, padx=10, pady=(4, 10))
        Button(button_row, text="应用当前目标", command=self._request_solve).pack(side=LEFT, padx=(0, 6))
        Button(button_row, text="读取当前末端", command=self._read_current_pose).pack(side=LEFT, padx=(0, 6))
        Button(button_row, text="回零位", command=self._home).pack(side=LEFT, padx=(0, 6))
        Button(button_row, text="关闭", command=self._on_close).pack(side=RIGHT)

        self._node.set_manual_mode(True)
        self._request_solve()

    def _target(self) -> Tuple[List[float], List[float]]:
        quat = [self._vars[key].get() for key in ("qx", "qy", "qz", "qw")]
        norm = math.sqrt(sum(value * value for value in quat))
        if norm < 1e-9:
            quat = [0.0, 0.0, 0.0, 1.0]
        else:
            quat = [value / norm for value in quat]
        return [self._vars[key].get() for key in ("x", "y", "z")], quat

    def _request_solve(self) -> None:
        if self._solving:
            return
        self._solving = True
        self._root.after(1, self._solve)

    def _solve(self) -> None:
        try:
            target_xyz, target_quat = self._target()
            seed = self._node.get_manual_positions()
            positions, ok, pos_err, rot_err, iterations = self._solver.solve(seed, target_xyz, target_quat)
            positions[6] = self._vars["L7"].get()
            self._node.set_manual_positions(positions)
            prefix = "IK OK" if ok else "IK 未完全收敛"
            self._status.set(f"{prefix} | pos_err={pos_err:.4f} m | rot_err={rot_err:.4f} rad | iter={iterations}")
            joint_text = " ".join(f"L{i + 1}={positions[i]:+.3f}" for i in range(7))
            self._joint_status.set(joint_text)
        finally:
            self._solving = False

    def _read_current_pose(self) -> None:
        transform = self._solver.forward(self._node.get_manual_positions())
        quat = _matrix_to_quaternion(transform[:3, :3])
        values = {
            "x": float(transform[0, 3]),
            "y": float(transform[1, 3]),
            "z": float(transform[2, 3]),
            "qx": quat[0],
            "qy": quat[1],
            "qz": quat[2],
            "qw": quat[3],
            "L7": self._node.get_manual_positions()[6],
        }
        for key, value in values.items():
            self._vars[key].set(value)
        self._request_solve()

    def _home(self) -> None:
        self._node.reset_home()
        self._read_current_pose()

    def _on_close(self) -> None:
        self._root.quit()
        self._root.destroy()

    def mainloop(self) -> None:
        self._root.mainloop()


def _sanitize_environment(prefix: Path) -> dict:
    env = dict(os.environ)
    env["PYTHONUTF8"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    env["ROS_DOMAIN_ID"] = env.get("ROS_DOMAIN_ID", "23")
    env["RMW_IMPLEMENTATION"] = env.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

    path_entries = [
        str(prefix / "Library" / "opt" / "rviz_ogre_vendor" / "bin"),
        str(prefix),
        str(prefix / "Scripts"),
        str(prefix / "Library" / "bin"),
        str(prefix / "Library" / "usr" / "bin"),
        str(Path(os.environ.get("SystemRoot", "C:\\Windows")) / "system32"),
        str(Path(os.environ.get("SystemRoot", "C:\\Windows"))),
        str(Path(os.environ.get("SystemRoot", "C:\\Windows")) / "System32" / "Wbem"),
    ]
    env["PATH"] = os.pathsep.join(path_entries)
    env["AMENT_PREFIX_PATH"] = str(prefix / "Library")
    env["CMAKE_PREFIX_PATH"] = str(prefix / "Library")
    env["COLCON_PREFIX_PATH"] = str(prefix / "Library")
    return env


def _write_windows_urdf(repo_root: Path) -> Path:
    urdf_path = repo_root / "el_a3_ros" / "el_a3_description" / "urdf" / "el_a3.urdf"
    mesh_dir = repo_root / "el_a3_ros" / "el_a3_description" / "meshes"

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        prefix = "package://el_a3_description/meshes/"
        if filename.startswith(prefix):
            local = mesh_dir / filename[len(prefix):]
            mesh.set("filename", local.as_uri())

    out_dir = Path(tempfile.gettempdir()) / "el_a3_windows_sim"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "el_a3_windows.urdf"
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path


def _start_process(args: List[str], env: dict, cwd: Path) -> subprocess.Popen:
    return subprocess.Popen(
        args,
        cwd=str(cwd),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )


def _pipe_output(name: str, proc: subprocess.Popen) -> None:
    assert proc.stdout is not None
    for line in proc.stdout:
        print(f"[{name}] {line.rstrip()}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run EL-A3 RViz simulation on native Windows.")
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--conda-prefix", required=True)
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--manual-control", action="store_true")
    parser.add_argument("--ik-control", action="store_true")
    parser.add_argument("--speed-scale", type=float, default=1.0)
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    prefix = Path(args.conda_prefix).resolve()
    env = _sanitize_environment(prefix)
    os.environ.update(env)
    urdf = _write_windows_urdf(repo_root)
    robot_description = urdf.read_text(encoding="utf-8")
    joints = _load_urdf_joints(urdf)

    rviz_exe = prefix / "Library" / "bin" / "rviz2.exe"

    rviz_config = repo_root / "el_a3_ros" / "el_a3_description" / "config" / "el_a3_view.rviz"
    processes: List[subprocess.Popen] = []
    if not args.no_rviz:
        processes.append(_start_process([str(rviz_exe), "-d", str(rviz_config)], env, repo_root))

    for idx, proc in enumerate(processes):
        threading.Thread(
            target=_pipe_output,
            args=(f"proc{idx}", proc),
            daemon=True,
        ).start()

    stop = False

    def _stop(_signum=None, _frame=None):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    rclpy.init(args=None)
    node = JointStateMotion(
        speed_scale=args.speed_scale,
        loop=not args.once and not args.manual_control and not args.ik_control,
        robot_description=robot_description,
        joints=joints,
    )
    ui = None
    if args.ik_control:
        ui = IKControlUI(node, joints)
    elif args.manual_control:
        ui = JointControlUI(node, joints)

    spin_thread = None
    if ui is not None:
        spin_thread = threading.Thread(
            target=lambda: _spin_until_done(node, lambda: stop),
            daemon=True,
        )
        spin_thread.start()

    try:
        print("EL-A3 Windows RViz simulation is running. Press Ctrl+C to stop.")
        if ui is not None:
            ui.mainloop()
            stop = True
        else:
            _spin_until_done(node, lambda: stop)
    finally:
        stop = True
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        for proc in processes:
            if proc.poll() is None:
                proc.terminate()
        for proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    return 0


def _spin_until_done(node: JointStateMotion, should_stop) -> None:
    while rclpy.ok() and not should_stop() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)


if __name__ == "__main__":
    raise SystemExit(main())
