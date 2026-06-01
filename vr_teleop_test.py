#!/usr/bin/env python3
"""
EL-A3 VR 遥操作：Pico WebXR → 机械臂增量 IK 控制

坐标映射:
  VR 空间:    +X 右,    +Y 上,  -Z 前
  Robot 空间: +X 右,    +Z 上,  -Y 前

  Robot_X = +VR_X      (VR 右 → Robot 右 / VR 左 → Robot 左)
  Robot_Y = +VR_Z      (VR 前(-Z) → Robot 前(-Y))
  Robot_Z = +VR_Y      (VR 上 → Robot 上)

模式:
  --sim   仿真模式: 仅用运动学验证坐标变换，不连接真实机械臂
  --ros   仿真模式下发布 /joint_states 到 ROS 2，在 RViz 中可视化
  默认    真实模式: 连接 CAN 总线控制机械臂
"""

import argparse
import asyncio
import json
import math
import os
import ssl
import struct
import time
import sys
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np

# ROS 2 (可选)
try:
    import rclpy
    from sensor_msgs.msg import JointState
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ─── VR → Robot 坐标对齐矩阵 ─────────────────────────────────────
# Robot = R_align @ VR
R_ALIGN = np.array([
    [1.0,  0.0,  0.0],
    [0.0,  0.0,  1.0],
    [0.0,  1.0,  0.0],
], dtype=float)


# ─── 四元数工具 ──────────────────────────────────────────────────

def quat_inverse(q):
    x, y, z, w = q
    return np.array([-x, -y, -z, w])


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def quat_to_matrix(q):
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)],
    ])


def matrix_to_rpy(R):
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0
    return np.array([rx, ry, rz])


# ─── VR 数据提取 ─────────────────────────────────────────────────

def extract_pose(source: dict, pose_type: str = "grip"):
    pose = source.get(pose_type)
    if not pose:
        return None, None
    p = pose.get("position", {})
    o = pose.get("orientation", {})
    pos = np.array([p.get("x", 0), p.get("y", 0), p.get("z", 0)], dtype=float)
    quat = np.array([o.get("x", 0), o.get("y", 0), o.get("z", 0), o.get("w", 1)], dtype=float)
    return pos, quat


# ─── 文件轮询客户端 ────────────────────────────────────────────

class VRPoseClient:
    """从 pico server 写入的 JSON 文件读取 VR 数据 (零网络开销)"""

    def __init__(self, filepath: str = "/tmp/pico_latest_pose.json"):
        self.filepath = filepath
        self.latest_packet: dict = {}
        self.seq: int = 0

    async def poll_loop(self):
        """轮询文件变化"""
        import os as _os
        first = True
        last_mtime = 0
        while True:
            try:
                mtime = _os.path.getmtime(self.filepath)
                if mtime == last_mtime:
                    await asyncio.sleep(0.005)
                    continue
                last_mtime = mtime

                with open(self.filepath, "r") as f:
                    body = json.load(f)
                if body and body.get("type") == "pose":
                    if first:
                        print(f"[VR] 文件读取已连接: {self.filepath}, seq={body.get('seq','?')}")
                        first = False
                    self.latest_packet = body
                    self.seq = body.get("seq", self.seq)
                await asyncio.sleep(0.005)
            except (FileNotFoundError, json.JSONDecodeError):
                await asyncio.sleep(0.1)
            except Exception as e:
                if first:
                    print(f"[VR] 读取失败: {e}, 等待 {self.filepath}...")
                await asyncio.sleep(0.5)


# ─── ROS 2 关节状态发布 ──────────────────────────────────────────

# URDF 关节名 (与 el_a3.urdf 一致)
JOINT_NAMES_ROS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]


class ROSSimSender:
    """向 ROS 2 仿真环境发送 JointTrajectory 命令"""

    def __init__(self, topic: str = "/arm_controller/joint_trajectory"):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy) 不可用")

        if not rclpy.ok():
            rclpy.init(args=sys.argv)

        self._node = rclpy.create_node("vr_teleop_sim")
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        self._publisher = self._node.create_publisher(JointTrajectory, topic, 10)
        self._lock = threading.Lock()
        self._q = [0.0] * 6
        self._dirty = False
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def set_joints(self, q: list):
        with self._lock:
            self._q = list(q[:6])
            self._dirty = True

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True, name="ros_sim")
        self._thread.start()
        print(f"[ROS-Sim] 发送 JointTrajectory → /arm_controller")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._node:
            self._node.destroy_node()

    def _spin(self):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        while self._running and rclpy.ok():
            with self._lock:
                if not self._dirty:
                    time.sleep(0.01)
                    continue
                q = list(self._q)
                self._dirty = False

            msg = JointTrajectory()
            msg.joint_names = JOINT_NAMES_ROS
            point = JointTrajectoryPoint()
            point.positions = q
            point.velocities = [0.0] * 6
            point.time_from_start = Duration(sec=0, nanosec=100000000)  # 100ms，更平滑
            msg.points = [point]
            self._publisher.publish(msg)
            time.sleep(0.03)  # ~30Hz


class ROSJointPublisher:
    """在独立线程中发布 /joint_states 到 ROS 2"""

    def __init__(self, node_name: str = "vr_teleop", topic: str = "/joint_states",
                 rate_hz: float = 30.0):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy) 不可用")

        self._topic = topic
        self._rate_hz = rate_hz
        self._lock = threading.Lock()
        self._q = [0.0] * 6
        self._seq = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # 在调用线程初始化 rclpy (非 ROS 标准但单节点可行)
        if not rclpy.ok():
            rclpy.init(args=sys.argv)

        self._node = rclpy.create_node(node_name)
        self._publisher = self._node.create_publisher(JointState, topic, 10)

    def set_joints(self, q: list):
        """线程安全地更新关节角"""
        with self._lock:
            self._q = list(q)
            self._seq += 1

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True, name="ros_pub")
        self._thread.start()
        print(f"[ROS] 发布 /joint_states @ {self._rate_hz}Hz")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _spin(self):
        period = 1.0 / self._rate_hz
        while self._running and rclpy.ok():
            t_start = time.time()

            with self._lock:
                q = list(self._q)
                seq = self._seq

            msg = JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = ""
            msg.name = JOINT_NAMES_ROS
            msg.position = q
            msg.velocity = [0.0] * 6
            msg.effort = [0.0] * 6

            self._publisher.publish(msg)

            elapsed = time.time() - t_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


# ─── 变换管线数据结构 ────────────────────────────────────────────

@dataclass
class TransformDebug:
    """记录一次变换管线的所有中间结果"""
    vr_pos: np.ndarray = None
    vr_quat: np.ndarray = None
    vr_pos_init: np.ndarray = None
    vr_quat_init: np.ndarray = None
    delta_pos_vr: np.ndarray = None
    delta_quat_vr: np.ndarray = None
    delta_pos_robot: np.ndarray = None
    R_vr_delta: np.ndarray = None
    R_robot_delta: np.ndarray = None
    delta_rpy_robot: np.ndarray = None
    target_pos: np.ndarray = None
    target_rpy: np.ndarray = None
    q_target: list = None
    fk_pos: np.ndarray = None
    fk_rpy: np.ndarray = None


# ─── 遥操作控制器 (Sim 模式) ──────────────────────────────────────

class VRTeleopSim:
    """仿真模式: 纯运动学验证，不连接真实机械臂"""

    def __init__(self, scale_pos=1.0, scale_rot=1.0, dead_zone=0.02,
                 hand="right", pose_type="grip"):
        self.scale_pos = scale_pos
        self.scale_rot = scale_rot
        self.dead_zone = dead_zone
        self.hand = hand
        self.pose_type = pose_type

        # 非奇异初始姿态: 臂弯曲 30°, 远离 workspace 边界和奇异点
        self._q_default = [
            0.0,           # L1: 中立
            math.radians(45),   # L2: 前倾
            math.radians(-90),  # L3: 弯曲
            0.0,           # L4: 中立
            math.radians(45),   # L5: 弯曲
            0.0,           # L6: 中立
        ]

        self._active = False
        self._init_vr_pos: Optional[np.ndarray] = None
        self._init_vr_quat: Optional[np.ndarray] = None
        self._init_ee_pos: Optional[np.ndarray] = None   # 激活时的末端位置
        self._init_ee_rpy: Optional[np.ndarray] = None   # 激活时的末端姿态
        self._q_current = list(self._q_default)
        self._last_seq = 0
        self._kin = None
        self._last_debug: TransformDebug = None
        self._frame_count = 0
        self._print_interval = 10  # 每 N 帧打印一次详细信息
        self._ik_fail_count = 0

    @property
    def active(self):
        return self._active

    def _get_kin(self):
        if self._kin is None:
            from el_a3_sdk.kinematics import ELA3Kinematics
            self._kin = ELA3Kinematics()
        return self._kin

    def _pad_q(self, q6):
        """6 关节 → 7 DOF (pad gripper=0)"""
        return list(q6) + [0.0]

    def update(self, packet: dict) -> Optional[TransformDebug]:
        sources = packet.get("inputSources") or []
        source = next((s for s in sources if s.get("handedness") == self.hand), None)
        if source is None:
            return None

        gamepad = source.get("gamepad", {})
        buttons = gamepad.get("buttons", [])
        trigger = buttons[0].get("value", 0) if buttons else 0

        vr_pos, vr_quat = extract_pose(source, self.pose_type)
        if vr_pos is None or vr_quat is None:
            return None

        vr_quat = vr_quat / np.linalg.norm(vr_quat)

        # ── 扳机释放 ──
        if trigger < 0.3:
            if self._active:
                self._active = False
                print("\n[Teleop] 释放 — 增量控制停止\n")
            return None

        # ── 扳机首次按下 → 记录初始姿态 ──
        if not self._active:
            self._active = True
            self._init_vr_pos = vr_pos.copy()
            self._init_vr_quat = vr_quat.copy()
            self._frame_count = 0
            # 清除平滑状态，重新开始
            self._smooth_delta_pos = None
            self._smooth_delta_rpy = None
            self._last_target_pos = None
            self._last_target_rpy = None

            kin = self._get_kin()
            ee = kin.forward_kinematics(self._q_current)
            # 记录激活时的末端位姿作为后续增量的固定基准
            self._init_ee_pos = np.array([ee.x, ee.y, ee.z])
            self._init_ee_rpy = np.array([ee.rx, ee.ry, ee.rz])

            print(f"\n{'='*72}")
            print(f"[Teleop] 激活 — 记录初始姿态")
            print(f"  初始 VR 位置:    ({self._init_vr_pos[0]:+.4f}, {self._init_vr_pos[1]:+.4f}, {self._init_vr_pos[2]:+.4f}) m")
            print(f"  初始 VR 四元数:  ({self._init_vr_quat[0]:+.4f}, {self._init_vr_quat[1]:+.4f}, "
                  f"{self._init_vr_quat[2]:+.4f}, {self._init_vr_quat[3]:+.4f})")
            print(f"  初始末端位置:    ({ee.x:+.4f}, {ee.y:+.4f}, {ee.z:+.4f}) m")
            print(f"  初始末端 RPY:    ({math.degrees(ee.rx):+.1f}, {math.degrees(ee.ry):+.1f}, "
                  f"{math.degrees(ee.rz):+.1f}) deg")
            print(f"  初始关节角:      {[f'{math.degrees(v):+.1f}°' for v in self._q_current]}")
            print(f"{'='*72}")
            return None

        # ── 检查新数据 ──
        seq = packet.get("seq", 0)
        if seq == self._last_seq:
            return None
        self._last_seq = seq
        self._frame_count += 1

        # ── Step 1: VR 空间增量 ──
        delta_pos_vr = vr_pos - self._init_vr_pos
        if np.linalg.norm(delta_pos_vr) < self.dead_zone:
            delta_pos_vr = np.zeros(3)

        delta_quat_vr = quat_multiply(vr_quat, quat_inverse(self._init_vr_quat))

        # ── Step 2: 坐标变换 ──
        delta_pos_robot = R_ALIGN @ delta_pos_vr * self.scale_pos

        R_vr_delta = quat_to_matrix(delta_quat_vr)
        R_robot_delta = R_ALIGN @ R_vr_delta @ R_ALIGN.T
        delta_rpy_robot = matrix_to_rpy(R_robot_delta) * self.scale_rot

        # ── Step 2.5: EMA 平滑 (抑制手抖) ──
        if not hasattr(self, '_smooth_delta_pos'):
            self._smooth_delta_pos = delta_pos_robot.copy()
            self._smooth_delta_rpy = delta_rpy_robot.copy()
        else:
            alpha = 0.3  # 越小越平滑 (0~1, 0=不动, 1=直通)
            self._smooth_delta_pos = alpha * delta_pos_robot + (1 - alpha) * self._smooth_delta_pos
            self._smooth_delta_rpy = alpha * delta_rpy_robot + (1 - alpha) * self._smooth_delta_rpy

        # ── Step 2.6: 阈值过滤 (变化小于阈值不更新，消除静止抖动) ──
        if not hasattr(self, '_last_target_pos'):
            self._last_target_pos = self._init_ee_pos.copy()
            self._last_target_rpy = self._init_ee_rpy.copy()

        target_pos_raw = self._init_ee_pos + self._smooth_delta_pos
        target_rpy_raw = self._init_ee_rpy + self._smooth_delta_rpy

        pos_change = np.linalg.norm(target_pos_raw - self._last_target_pos)
        rot_change = np.linalg.norm(target_rpy_raw - self._last_target_rpy)

        # 只有位置变化 > 3mm 或旋转 > 0.5° 时才更新
        if pos_change < 0.003 and rot_change < math.radians(0.5):
            return None

        self._last_target_pos = target_pos_raw.copy()
        self._last_target_rpy = target_rpy_raw.copy()

        # ── Step 3: 增量应用到激活时记录的固定基准 ──
        target_pos = target_pos_raw
        target_rpy = target_rpy_raw

        # ── Step 4: IK (实时 ik_step, 自适应阻尼 + 限步) ──
        kin = self._get_kin()
        from el_a3_sdk.data_types import ArmEndPose
        target_pose = ArmEndPose(
            x=float(target_pos[0]), y=float(target_pos[1]), z=float(target_pos[2]),
            rx=float(target_rpy[0]), ry=float(target_rpy[1]), rz=float(target_rpy[2]),
        )

        try:
            q_new, ik_err = kin.ik_step(
                target_pose, q_current=self._q_current,
                damping=5e-3, max_step=0.3, max_iter=5, converge_eps=1e-4,
            )
        except Exception as e:
            self._ik_fail_count += 1
            if self._ik_fail_count <= 3:
                print(f"[Teleop] IK 异常: {e}")
            return None

        if q_new is None:
            self._ik_fail_count += 1
            return None

        self._ik_fail_count = 0
        q_target = q_new[:6]
        self._q_current = q_target

        # ── Step 5: FK 验证 ──
        ee_result = kin.forward_kinematics(q_target[:6])

        # ── 组装 debug 结构 ──
        dbg = TransformDebug(
            vr_pos=vr_pos, vr_quat=vr_quat,
            vr_pos_init=self._init_vr_pos, vr_quat_init=self._init_vr_quat,
            delta_pos_vr=delta_pos_vr, delta_quat_vr=delta_quat_vr,
            delta_pos_robot=delta_pos_robot,
            R_vr_delta=R_vr_delta, R_robot_delta=R_robot_delta,
            delta_rpy_robot=delta_rpy_robot,
            target_pos=target_pos, target_rpy=target_rpy,
            q_target=q_target[:6],
            fk_pos=np.array([ee_result.x, ee_result.y, ee_result.z]),
            fk_rpy=np.array([ee_result.rx, ee_result.ry, ee_result.rz]),
        )
        self._last_debug = dbg

        # 定期打印详细变换管线
        if self._frame_count % self._print_interval == 1:
            self._print_pipeline(dbg, trigger)

        return dbg

    def _print_pipeline(self, dbg: TransformDebug, trigger: float):
        """打印完整的变换管线"""
        dpv = dbg.delta_pos_vr
        dpr = dbg.delta_pos_robot
        drpy = dbg.delta_rpy_robot

        print(f"\n{'─'*72}")
        print(f"  变换管线 (frame={self._frame_count}, trigger={trigger:.2f})")
        print(f"{'─'*72}")

        # VR 当前 vs 初始
        print(f"  VR 当前位置:  ({dbg.vr_pos[0]:+.4f}, {dbg.vr_pos[1]:+.4f}, {dbg.vr_pos[2]:+.4f})")
        print(f"  VR 初始位置:  ({dbg.vr_pos_init[0]:+.4f}, {dbg.vr_pos_init[1]:+.4f}, {dbg.vr_pos_init[2]:+.4f})")

        # Step 1: VR 增量
        print(f"\n  [1] VR 空间增量:")
        print(f"      Δpos (VR):  ({dpv[0]:+.4f}, {dpv[1]:+.4f}, {dpv[2]:+.4f}) m  "
              f"|Δ|={np.linalg.norm(dpv):.4f}m")
        q_axis = _quat_to_axis_angle(dbg.delta_quat_vr)
        print(f"      Δquat (VR): ({dbg.delta_quat_vr[0]:+.4f}, {dbg.delta_quat_vr[1]:+.4f}, "
              f"{dbg.delta_quat_vr[2]:+.4f}, {dbg.delta_quat_vr[3]:+.4f})")
        print(f"                   轴=({q_axis[0]:+.3f},{q_axis[1]:+.3f},{q_axis[2]:+.3f}) "
              f"角度={math.degrees(q_axis[3]):.1f}°")

        # Step 2: 坐标变换后
        print(f"\n  [2] 坐标变换 (VR→Robot, scale_pos={self.scale_pos}, scale_rot={self.scale_rot}):")
        print(f"      Δpos (Robot): ({dpr[0]:+.4f}, {dpr[1]:+.4f}, {dpr[2]:+.4f}) m  "
              f"|Δ|={np.linalg.norm(dpr):.4f}m")
        print(f"      Δrpy (Robot): roll={math.degrees(drpy[0]):+.1f}°  "
              f"pitch={math.degrees(drpy[1]):+.1f}°  yaw={math.degrees(drpy[2]):+.1f}°")

        # Step 3: 目标末端
        print(f"\n  [3] 目标末端位姿:")
        print(f"      target pos: ({dbg.target_pos[0]:+.4f}, {dbg.target_pos[1]:+.4f}, "
              f"{dbg.target_pos[2]:+.4f}) m")
        print(f"      target rpy: roll={math.degrees(dbg.target_rpy[0]):+.1f}°  "
              f"pitch={math.degrees(dbg.target_rpy[1]):+.1f}°  "
              f"yaw={math.degrees(dbg.target_rpy[2]):+.1f}°")

        # Step 4: IK 结果
        print(f"\n  [4] IK 结果 (关节角):")
        print(f"      q = [{', '.join(f'{math.degrees(v):+.2f}°' for v in dbg.q_target)}]")

        # Step 5: FK 验证
        fk_err = np.linalg.norm(dbg.fk_pos - dbg.target_pos)
        print(f"\n  [5] FK 验证 (回算末端):")
        print(f"      FK pos:  ({dbg.fk_pos[0]:+.4f}, {dbg.fk_pos[1]:+.4f}, {dbg.fk_pos[2]:+.4f}) m")
        print(f"      FK rpy:  roll={math.degrees(dbg.fk_rpy[0]):+.1f}°  "
              f"pitch={math.degrees(dbg.fk_rpy[1]):+.1f}°  yaw={math.degrees(dbg.fk_rpy[2]):+.1f}°")
        print(f"      FK 误差: {fk_err:.4f}m  "
              f"({'✓ 收敛' if fk_err < 0.01 else '✗ 未收敛' if fk_err > 0.05 else '⚠ 边界'})")
        print(f"{'─'*72}")


def _quat_to_axis_angle(q):
    """四元数 → 轴角表示 (axis_x, axis_y, axis_z, angle_rad)"""
    x, y, z, w = q
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    if abs(angle) < 1e-10:
        return np.array([0.0, 0.0, 1.0, 0.0])
    s = math.sqrt(1.0 - w*w)
    if s < 1e-10:
        return np.array([0.0, 0.0, 1.0, 0.0])
    return np.array([x/s, y/s, z/s, angle])


# ─── 文件 IPC 可视化 ─────────────────────────────────────────────

class FileVizWriter:
    """将关节角 + VR 数据写入 JSON 文件，供 arm_viz.py 读取"""

    def __init__(self, path: str):
        self.path = path
        self._q = [0.0] * 7
        self._vr_pos = None
        self._target_pos = None
        self._active = False
        self._frame = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def update(self, q: list, vr_pos_robot=None, target_pos=None, active=False):
        self._q = list(q) + [0.0]  # pad to 7 joints (含夹爪)
        self._vr_pos = list(vr_pos_robot) if vr_pos_robot is not None else None
        self._target_pos = list(target_pos) if target_pos is not None else None
        self._active = active
        self._frame += 1

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._write_loop, daemon=True, name="viz_writer")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _write_loop(self):
        while self._running:
            data = {
                "q": self._q,
                "vr_pos": self._vr_pos,
                "target_pos": self._target_pos,
                "active": self._active,
                "frame": self._frame,
            }
            try:
                with open(self.path, "w") as f:
                    json.dump(data, f)
            except Exception:
                pass
            time.sleep(0.033)  # ~30Hz


# ─── 主入口 ──────────────────────────────────────────────────────

async def run_sim(args):
    """仿真模式: 连接 VR，打印变换管线"""
    from el_a3_sdk.kinematics import ELA3Kinematics

    # ── 连接 VR (文件轮询, pico server 写入 /tmp/pico_latest_pose.json) ──
    vr_client = VRPoseClient()

    teleop = VRTeleopSim(
        scale_pos=args.scale_pos,
        scale_rot=args.scale_rot,
        dead_zone=args.dead_zone,
        hand=args.hand,
        pose_type=args.pose_type,
    )

    print(f"\n{'='*72}")
    print(f"VR 遥操作 — 仿真模式 (坐标变换验证)")
    print(f"  VR: /tmp/pico_latest_pose.json (pico server 自动写入)")
    print(f"  手柄: {args.hand} / {args.pose_type}")
    print(f"  位置缩放: {args.scale_pos:.1f}x    姿态缩放: {args.scale_rot:.1f}x")
    print(f"  死区: {args.dead_zone:.3f}m")
    print(f"  初始关节角 (非奇异): {[f'{math.degrees(v):.0f}°' for v in teleop._q_default]}")
    print(f"\n  操作: 握紧扳机 → 移动手柄 → 观察变换管线")
    print(f"        松开扳机 → 停止, 再次握紧 → 重新记录初始姿态")
    print(f"{'='*72}")

    print(f"\n  坐标系映射:")
    print(f"    VR:   +X右      +Y上      -Z前")
    print(f"    Robot: -X左      -Y前      +Z上")
    print(f"    R_align = [[ 1, 0, 0],")
    print(f"               [ 0, 0, 1],")
    print(f"               [ 0, 1, 0]]")

    # ── 可视化 (可选): 写文件供 arm_viz.py 读取 ──
    viz = None
    if args.viz:
        viz = FileVizWriter(args.viz_file)
        viz.update(teleop._q_default)  # 写入初始姿态
        viz.start()
        print(f"[Viz] 关节数据写入 {args.viz_file}")

    # ── ROS 发布 (可选) ──
    ros_pub = None
    if args.ros:
        if not ROS_AVAILABLE:
            print("[ROS] rclpy 不可用，忽略 --ros")
        else:
            ros_pub = ROSJointPublisher(node_name="vr_teleop", rate_hz=30.0)

    # ── ROS Sim 控制 (可选) ──
    ros_sim = None
    if args.ros_sim:
        if not ROS_AVAILABLE:
            print("[ROS-Sim] rclpy 不可用，忽略 --ros-sim")
        else:
            ros_sim = ROSSimSender()
            ros_sim.set_joints(teleop._q_default)
            ros_sim.start()
            print("[ROS-Sim] 仿真机械臂将跟随 VR 手柄运动")

    vr_task = asyncio.create_task(vr_client.poll_loop())

    # 等待 VR 数据到达
    print(f"\n  等待 VR 数据...")
    while not vr_client.latest_packet:
        await asyncio.sleep(0.1)
    print(f"  VR 数据已到达 (seq={vr_client.latest_packet.get('seq','?')})")

    # 启动 ROS 发布线程
    if ros_pub:
        ros_pub.set_joints(teleop._q_default)
        ros_pub.start()

    try:
        while True:
            await asyncio.sleep(0.01)
            packet = vr_client.latest_packet
            if not packet:
                continue
            teleop.update(packet)
            # 发布当前关节角到 ROS
            if ros_pub:
                ros_pub.set_joints(teleop._q_current)
            # ROS Sim: 发送关节轨迹
            if ros_sim and teleop.active:
                ros_sim.set_joints(teleop._q_current)
            # 更新可视化 (始终写入当前 VR 位置 + 关节角)
            if viz:
                # 提取当前 VR 手柄位置
                sources = packet.get("inputSources", [])
                src = next((s for s in sources if s.get("handedness") == args.hand), None)
                vr_pos_robot = None
                target_pos = None
                if src:
                    vr_pos_raw, _ = extract_pose(src, args.pose_type)
                    if vr_pos_raw is not None:
                        vr_pos_robot = R_ALIGN @ vr_pos_raw * teleop.scale_pos
                if teleop._last_debug:
                    target_pos = teleop._last_debug.target_pos
                viz.update(
                    teleop._q_current,
                    vr_pos_robot=vr_pos_robot,
                    target_pos=target_pos,
                    active=teleop.active,
                )
    except KeyboardInterrupt:
        print("\n[Teleop] 中断")
    finally:
        vr_task.cancel()
        try:
            await vr_task
        except asyncio.CancelledError:
            pass
        if ros_pub:
            ros_pub.stop()
        if ros_sim:
            ros_sim.stop()
        if viz:
            viz.stop()
        print("[Teleop] 已停止")

    return 0


async def run_real(args):
    """真实模式: 连接机械臂 CAN 总线"""
    from el_a3_sdk import ELA3Interface

    print(f"[Arm] 连接机械臂: {args.can}")
    arm = ELA3Interface(
        can_name=args.can,
        backend=args.backend,
        serial_port=args.serial_port,
        serial_baudrate=args.serial_baudrate,
        can_bitrate=args.can_bitrate,
    )

    if not arm.ConnectPort():
        print("[Arm] 连接失败!")
        return 1

    print("[Arm] 已连接, 使能电机...")
    arm.EnableArm()
    arm.start_control_loop(rate_hz=200.0)

    vr_client = VRPoseClient()

    teleop = VRTeleopSim(
        scale_pos=args.scale_pos,
        scale_rot=args.scale_rot,
        dead_zone=args.dead_zone,
        hand=args.hand,
        pose_type=args.pose_type,
    )

    # 真实模式下必须用真实机械臂当前关节角初始化，避免第一次遥操时跳变。
    try:
        q_real = arm.GetArmJointMsgs().to_list(include_gripper=False)
        if q_real and len(q_real) >= 6:
            teleop._q_current = list(q_real[:6])
            teleop._q_default = list(q_real[:6])
            print(f"[Arm] 使用当前真实关节角作为遥操初始值: "
                  f"{[f'{math.degrees(v):+.1f}°' for v in teleop._q_current]}")
    except Exception as e:
        print(f"[Arm] 读取真实关节角失败，继续使用默认姿态: {e}")

    print(f"\n{'='*60}")
    print(f"VR 遥操作已就绪 (真实模式)")
    print(f"  VR: 文件读取  机械臂: {args.can}")
    print(f"  握紧扳机开始, 松开停止")
    print(f"{'='*60}\n")

    vr_task = asyncio.create_task(vr_client.poll_loop())

    last_print = 0
    try:
        while True:
            await asyncio.sleep(0.005)

            packet = vr_client.latest_packet
            if not packet:
                continue

            dbg = teleop.update(packet)
            if dbg is not None and teleop.active and dbg.q_target:
                arm.JointCtrlList(dbg.q_target)

            now = time.time()
            if now - last_print > 1.0:
                if teleop.active:
                    ee = arm.GetArmEndPoseMsgs()
                    print(f"[Teleop] 末端: ({ee.x:.3f},{ee.y:.3f},{ee.z:.3f}) "
                          f"rpy=({math.degrees(ee.rx):.0f},{math.degrees(ee.ry):.0f},"
                          f"{math.degrees(ee.rz):.0f})°")
                last_print = now

    except KeyboardInterrupt:
        print("\n[Teleop] 中断")
    finally:
        vr_task.cancel()
        try:
            await vr_task
        except asyncio.CancelledError:
            pass
        arm.stop_control_loop()
        arm.DisableArm()
        arm.DisconnectPort()
        print("[Teleop] 已停止")

    return 0


def parse_args():
    p = argparse.ArgumentParser(description="EL-A3 VR 遥操作 (Pico WebXR → 机械臂)")

    # VR
    p.add_argument("--vr-host", default="192.168.3.5", help="Pico WebXR server IP")
    p.add_argument("--vr-port", type=int, default=8765, help="Pico WebXR server port")

    # 模式
    p.add_argument("--sim", action="store_true",
                   help="仿真模式: 纯运动学验证，不连接真实机械臂")
    p.add_argument("--ros", action="store_true",
                   help="仿真模式下发布 /joint_states 到 ROS 2 (需 rclpy)")
    p.add_argument("--ros-sim", action="store_true",
                   help="向 ROS 2 仿真环境 (/arm_controller/joint_trajectory) 发送关节角")
    p.add_argument("--viz", action="store_true",
                   help="仿真模式下将关节角写入临时文件，供 arm_viz.py 读取")
    p.add_argument("--viz-file", default="/tmp/el_a3_vr_joints.json",
                   help="可视化数据文件路径")

    # 机械臂 (真实模式)
    p.add_argument("--can", default="can0", help="CAN 接口名")
    p.add_argument("--backend", default="socketcan", choices=["socketcan", "slcan"])
    p.add_argument("--serial-port", default="/dev/ttyUSB0")
    p.add_argument("--serial-baudrate", type=int, default=2000000)
    p.add_argument("--can-bitrate", type=int, default=1000000)

    # 遥操作参数
    p.add_argument("--scale-pos", type=float, default=0.5,
                   help="位置缩放 (VR m → 机械臂 m), 默认 0.5")
    p.add_argument("--scale-rot", type=float, default=0.5,
                   help="姿态缩放, 默认 0.5")
    p.add_argument("--dead-zone", type=float, default=0.02,
                   help="位置死区 (m), 默认 0.02")
    p.add_argument("--hand", default="right", choices=["left", "right"])
    p.add_argument("--pose-type", default="grip", choices=["grip", "targetRay"])

    return p.parse_args()


def main():
    args = parse_args()
    try:
        if args.sim:
            return asyncio.run(run_sim(args))
        else:
            return asyncio.run(run_real(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
