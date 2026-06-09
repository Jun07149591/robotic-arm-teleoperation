#!/usr/bin/env python3
"""
EL-A3 Pico VR teleoperation node for ROS 2.

Control pipeline (incremental delta):
  1. hand_delta = current_hand - prev_hand         (per-frame increment)
  2. deadband (< 2mm → 0)                           (jitter removal)
  3. VR → robot coordinate transform (R_ALIGN)
  4. scaled_delta = robot_delta * position_scale
  5. ee_target_raw += scaled_delta                  (accumulate)
  6. workspace clamp
  7. low-pass filter on target                      (smoothing)
  8. per-frame max_step limit                       (acceleration guard)
  9. IK solve
 10. joint velocity limit                           (before publish)
"""

import json
import math
import os as _os
import sys as _sys
# Ensure el_a3_sdk is importable when running as a ROS 2 node
_sdk_dir = _os.path.realpath(_os.path.join(_os.path.dirname(__file__), "..", "..", "..", "el_a3_sdk"))
if _sdk_dir not in _sys.path:
    _sys.path.insert(0, _sdk_dir)
import threading
import time
from typing import Dict, List, Optional, Tuple, Any

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64

from el_a3_sdk.kinematics import ELA3Kinematics
from el_a3_sdk.data_types import ArmEndPose

ARM_JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]
GRIPPER_JOINT = "L7_joint"

SPEED_LEVELS = [
    ("T1", 0.10), ("T2", 0.25), ("T3", 0.50), ("T4", 0.75), ("T5", 1.00),
]
HOME_POSITIONS = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
ZERO_POSITIONS = [0.0] * 6

POSE_FILE = "/tmp/pico_latest_pose.json"

BTN_TRIGGER = 0
BTN_GRIP = 1
BTN_THUMBSTICK = 3
BTN_XA = 4
BTN_YB = 5

# Gripper control constants (matches SDK pico_control.py)
GRIP_SPEED = 3.0       # gripper movement speed (rad/s)
STICK_DEADZONE = 0.2   # thumbstick deadzone for gripper
GRIP_MAX = 2.0         # max gripper close angle (rad)
GRIP_LONG_PRESS_SEC = 0.3          # grip button long-press threshold
TRIGGER_HOLD_SEC = 0.3             # trigger hold for fine yaw mode
THUMB_ESTOP_HOLD_SEC = 0.5         # right thumbstick hold for estop
THUMB_ZT_HOLD_SEC = 0.3            # left thumbstick hold for zero torque
A_LONG_PRESS_SEC = 1.0             # A/X long press for motor disable/enable


def _read_pico_pose() -> Optional[Dict[str, Any]]:
    try:
        with open(POSE_FILE, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def _extract_grip_pose(source: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    grip = source.get("grip")
    if not grip:
        return None
    pos = grip.get("position")
    ori = grip.get("orientation")
    if not pos or not ori:
        return None
    return {
        "x": pos.get("x", 0.0), "y": pos.get("y", 0.0), "z": pos.get("z", 0.0),
        "qx": ori.get("x", 0.0), "qy": ori.get("y", 0.0),
        "qz": ori.get("z", 0.0), "qw": ori.get("w", 1.0),
    }


def quat_multiply(a: Tuple[float, ...], b: Tuple[float, ...]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_inverse(q: Tuple[float, ...]) -> Tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def quat_to_rpy(q: Tuple[float, ...]) -> Tuple[float, float, float]:
    x, y, z, w = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class PicoTeleopNode(Node):
    """Pico VR teleoperation node — incremental delta control pipeline."""

    def __init__(self) -> None:
        super().__init__("pico_teleop_node")

        self.declare_parameter("hand", "right")
        self.declare_parameter("arm_controller_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("gripper_controller_topic", "/gripper_controller/joint_trajectory")
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("position_scale", 0.05)
        self.declare_parameter("deadband", 0.002)
        self.declare_parameter("pos_filter_alpha", 0.3)
        self.declare_parameter("target_filter_alpha", 0.2)
        self.declare_parameter("filter_omega", 14.0)
        self.declare_parameter("max_ik_jump", 0.5)
        self.declare_parameter("max_ee_speed", 0.05)
        self.declare_parameter("max_joint_velocity", 1.0)
        self.declare_parameter("trajectory_time_from_start", 0.08)
        self.declare_parameter("workspace_radius", 0.35)
        self.declare_parameter("yaw_scale", 0.5)
        self.declare_parameter("yaw_deadband_deg", 2.0)
        self.declare_parameter("yaw_stick_scale", 0.5)

        self._rate = float(self.get_parameter("update_rate").value)
        self._pos_scale = float(self.get_parameter("position_scale").value)
        self._deadband = float(self.get_parameter("deadband").value)
        self._pos_filter_alpha = float(self.get_parameter("pos_filter_alpha").value)
        self._target_filter_alpha = float(self.get_parameter("target_filter_alpha").value)
        self._filter_omega = float(self.get_parameter("filter_omega").value)
        self._max_ik_jump = float(self.get_parameter("max_ik_jump").value)
        self._max_ee_speed = float(self.get_parameter("max_ee_speed").value)
        self._max_joint_vel = float(self.get_parameter("max_joint_velocity").value)
        self._traj_dt = float(self.get_parameter("trajectory_time_from_start").value)
        self._workspace_radius = float(self.get_parameter("workspace_radius").value)
        self._yaw_scale = float(self.get_parameter("yaw_scale").value)
        self._yaw_deadband_deg = float(self.get_parameter("yaw_deadband_deg").value)
        self._yaw_stick_scale = float(self.get_parameter("yaw_stick_scale").value)
        self._stick_x_ema = 0.0

        self._kin = ELA3Kinematics()
        self.get_logger().info("Pinocchio kinematics initialized")

        cb_group = ReentrantCallbackGroup()
        _arm_topic = str(self.get_parameter("arm_controller_topic").value)
        _gripper_topic = str(self.get_parameter("gripper_controller_topic").value)
        self._arm_pub = self.create_publisher(JointTrajectory, _arm_topic, 10)
        self._gripper_pub = self.create_publisher(JointTrajectory, _gripper_topic, 10)
        self._gripper_torque_pub = self.create_publisher(
            Float64, "/gripper_controller/torque_limit", 10)
        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10,
            callback_group=cb_group)
        self._switch_ctrl_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller",
            callback_group=cb_group)

        # Control state
        self._dt = 1.0 / self._rate
        self._speed_idx = 2
        self._speed_factor = SPEED_LEVELS[self._speed_idx][1]
        self._max_step = self._max_ee_speed * self._dt  # per-frame position limit

        self._zero_torque = False
        self._is_moving = False
        self._estop = False

        # Calibration
        self._ref_quat: Optional[Tuple[float, float, float, float]] = None
        self._ref_hand_rpy: Optional[List[float]] = None  # [roll, pitch, yaw] in VR space
        self._ref_robot_rpy: Optional[List[float]] = None  # [rx, ry, rz] in robot space
        self._calibrated = False
        self._tracking_engaged = False

        # Per-frame hand delta tracking
        self._filtered_hand_pos: Optional[List[float]] = None   # EMA filtered
        self._prev_hand_pos: Optional[List[float]] = None

        # End-effector target (raw accumulated + filtered)
        self._ee_target_raw: Optional[List[float]] = None       # [x, y, z]
        self._ee_target_rpy_raw: Optional[List[float]] = None   # [rx, ry, rz]
        self._ee_target_filtered: Optional[List[float]] = None   # [x, y, z, rx, ry, rz]
        self._target_pose: Optional[ArmEndPose] = None

        # IK state
        self._ik_seed: Optional[List[float]] = None
        self._ik_filter_pos: Optional[List[float]] = None
        self._ik_filter_vel: Optional[List[float]] = None
        self._ik_raw: Optional[List[float]] = None
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0
        self._seed_just_init = False
        self._resync_cooldown = 0

        # Gripper
        self._gripper_angle = 0.0
        self._grip_active = False

        # Joint state
        self._current_q: Optional[List[float]] = None
        self._joint_state_received = False
        self._q_buffer: List[List[float]] = []
        self._q_buffer_size = 5

        # Button state
        self._a_hold_start: Optional[float] = None
        self._trigger_hold_start: Optional[float] = None   # trigger hold timer for fine yaw
        self._fine_yaw_mode = False                        # freeze position, only track wrist yaw
        self._last_tp_val: float = 0.0                     # last trigger analog value
        self._thumb_hold_start: Optional[float] = None     # right thumbstick hold timer (estop)
        self._zt_hold_start: Optional[float] = None        # left thumbstick hold timer (zero torque)
        self._grip_press_start: Optional[float] = None     # grip button hold timer (tracking)
        # Gripper state (controlled by thumbstick Y)
        self._gripper_angle: float = 0.0
        self._gripper_last_sent: float = -1.0
        self._gripper_hold_active: bool = False
        self._gripper_last_hold_time: float = 0.0
        self._gripper_hold_interval: float = 0.25
        self._axes_detected: bool = False

        # Pico data
        self._last_pose_seq = -1
        self._prev_btn_left = [0] * 8
        self._prev_btn_right = [0] * 8
        self._initialized = False
        self._diag_tick = 0

        timer_period = 1.0 / self._rate
        self._timer = self.create_timer(timer_period, self._control_tick, callback_group=cb_group)
        self.get_logger().info(
            f"Pico teleop (incremental): {self._rate:.0f}Hz scale={self._pos_scale:.3f} "
            f"deadband={self._deadband*1000:.1f}mm filter_alpha={self._target_filter_alpha:.2f} "
            f"max_ee={self._max_ee_speed:.3f}m/s max_jv={self._max_joint_vel:.1f}rad/s "
            f"max_step={self._max_step*1000:.1f}mm/tick ws={self._workspace_radius:.2f}m")

    # ---- Joint state callback ----

    def _joint_state_callback(self, msg: JointState) -> None:
        q = [0.0] * 6
        found = 0
        for i, name in enumerate(ARM_JOINTS):
            if name in msg.name:
                idx = list(msg.name).index(name)
                q[i] = msg.position[idx]
                found += 1
        if found == 6:
            self._current_q = q
            self._q_buffer.append(list(q))
            if len(self._q_buffer) > self._q_buffer_size:
                self._q_buffer.pop(0)
            if not self._joint_state_received:
                self._joint_state_received = True
                self.get_logger().info(
                    f"Joint states received: [{', '.join(f'{v:.3f}' for v in q)}]")

    # ---- Main control tick ----

    def _control_tick(self) -> None:
        if not self._joint_state_received or self._current_q is None:
            return
        if not self._initialized:
            self._initialize()

        packet = _read_pico_pose()
        if packet is None:
            return

        seq = packet.get("seq", -1)
        if seq == self._last_pose_seq:
            if self._ik_filter_pos is not None and not self._is_moving:
                self._send_filtered()
            return
        self._last_pose_seq = seq

        _track_hand = str(self.get_parameter("hand").value)
        sources = packet.get("inputSources") or []
        left_src, right_src = None, None
        for s in sources:
            h = s.get("handedness", "")
            if h == "left":
                left_src = s
            elif h == "right":
                right_src = s

        left_pose = _extract_grip_pose(left_src) if left_src else None
        track_pose = _extract_grip_pose(right_src) if right_src else None
        track_pose = left_pose if _track_hand == "left" else track_pose

        left_btns, right_btns = [], []
        left_axes, right_axes = [], []
        if left_src:
            gp = left_src.get("gamepad")
            if gp:
                left_btns = gp.get("buttons", [])
                left_axes = gp.get("axes", [])
        if right_src:
            gp = right_src.get("gamepad")
            if gp:
                right_btns = gp.get("buttons", [])
                right_axes = gp.get("axes", [])
        track_axes = left_axes if _track_hand == "left" else right_axes

        # Make controller pose headset-relative
        viewer = packet.get("viewer", {})
        if viewer:
            vpos = viewer.get("position", {})
            hx, hy, hz = vpos.get("x", 0), vpos.get("y", 0), vpos.get("z", 0)
            if left_pose:
                left_pose["x"] -= hx; left_pose["y"] -= hy; left_pose["z"] -= hz
            if track_pose:
                track_pose["x"] -= hx; track_pose["y"] -= hy; track_pose["z"] -= hz

        def _btn_val(btns, idx):
            if idx < len(btns) and btns[idx] is not None:
                return 1 if btns[idx].get("pressed") else 0
            return 0

        def _btn_edge(cur, prev):
            return cur == 1 and prev == 0

        # ---- Button handling ----
        # A/X: short press=zero, long press 1s=disable motors
        lx = _btn_val(left_btns, BTN_XA)
        ra = _btn_val(right_btns, BTN_XA)
        a_pressed = (lx == 1) or (ra == 1)
        a_was = (self._prev_btn_left[BTN_XA] == 1) or (self._prev_btn_right[BTN_XA] == 1)
        now_a = time.monotonic()
        if self._a_hold_start is None:
            if a_pressed and not a_was:
                self._a_hold_start = now_a
        if a_pressed and self._a_hold_start is not None:
            if now_a - self._a_hold_start >= 1.0:
                self._a_hold_start = None
                if self._estop:
                    self.get_logger().info("A long press: re-enabling motors...")
                    self._enable_motors()
                else:
                    self.get_logger().info("A long press: disabling motors...")
                    self._disable_motors()
        elif not a_pressed and self._a_hold_start is not None:
            if 0.05 < now_a - self._a_hold_start < 1.0:
                self._async_move(ZERO_POSITIONS, "zero")
            self._a_hold_start = None

        ly = _btn_val(left_btns, BTN_YB)
        if _btn_edge(ly, self._prev_btn_left[BTN_YB]):
            self._async_move(HOME_POSITIONS, "Home")
        rb = _btn_val(right_btns, BTN_YB)
        if _btn_edge(rb, self._prev_btn_right[BTN_YB]):
            self._async_move(HOME_POSITIONS, "Home")

        # Zero torque toggle: left thumbstick hold 0.3s (matches SDK)
        lt = _btn_val(left_btns, BTN_THUMBSTICK)
        if lt == 1:
            if self._zt_hold_start is None:
                self._zt_hold_start = time.monotonic()
            elif time.monotonic() - self._zt_hold_start >= THUMB_ZT_HOLD_SEC:
                self._toggle_zero_torque()
                self._zt_hold_start = None
        else:
            self._zt_hold_start = None

        # Emergency stop: right thumbstick hold 0.5s (matches SDK, prevents accidental trigger)
        rt = _btn_val(right_btns, BTN_THUMBSTICK)
        if rt == 1:
            if self._thumb_hold_start is None:
                self._thumb_hold_start = time.monotonic()
            elif time.monotonic() - self._thumb_hold_start >= THUMB_ESTOP_HOLD_SEC:
                self._emergency_stop()
                self._thumb_hold_start = None
        else:
            self._thumb_hold_start = None

        # ---- Grip: hold 1s to start tracking, release to stop ----
        l_grip = _btn_val(left_btns, BTN_GRIP)
        r_grip = _btn_val(right_btns, BTN_GRIP)
        any_grip = (l_grip == 1) or (r_grip == 1)

        if any_grip and not self._tracking_engaged:
            now = time.monotonic()
            if self._grip_press_start is None:
                self._grip_press_start = now
            elif now - self._grip_press_start >= 1.0:
                self._tracking_engaged = True
                self._grip_press_start = None
                # Reset per-frame delta + hand RPY reference on engagement
                if track_pose is not None:
                    hp = [track_pose["x"], track_pose["y"], track_pose["z"]]
                    self._filtered_hand_pos = hp
                    self._prev_hand_pos = hp
                    self._ref_hand_rpy = list(quat_to_rpy(
                        (track_pose["qx"], track_pose["qy"],
                         track_pose["qz"], track_pose["qw"])))
                # Snap robot RPY reference to current FK
                if self._current_q is not None:
                    fk = self._kin.forward_kinematics(list(self._current_q))
                    self._ref_robot_rpy = [fk.rx, fk.ry, fk.rz]
                self.get_logger().info("Tracking ON")
        elif any_grip and self._tracking_engaged:
            pass
        elif not any_grip:
            self._grip_press_start = None
            if self._tracking_engaged:
                self._tracking_engaged = False
                self._filtered_hand_pos = None
                self._prev_hand_pos = None
                if self._ik_filter_pos is not None:
                    self._ik_raw = list(self._ik_filter_pos)
                    self._ik_filter_vel = [0.0] * 6
                self.get_logger().info("Tracking OFF")

        # ---- Trigger (btn[0]): hold 0.3s → fine yaw mode, release → exit (matches SDK) ----
        def _trigger_val(btns):
            if 0 < len(btns) and btns[0] is not None:
                return float(btns[0].get("value", 0.0))
            return 0.0

        track_btns = left_btns if _track_hand == "left" else right_btns
        trigger_val = _trigger_val(track_btns)
        tp = trigger_val > 0.5
        if abs(trigger_val - self._last_tp_val) > 0.05:
            self._last_tp_val = trigger_val
            self.get_logger().debug(f"TRIGGER val={trigger_val:.3f} tp={tp} fine_yaw={self._fine_yaw_mode}")
        if tp and self._trigger_hold_start is None:
            self._trigger_hold_start = time.monotonic()
        elif tp and self._trigger_hold_start is not None:
            if not self._fine_yaw_mode and (time.monotonic() - self._trigger_hold_start) >= TRIGGER_HOLD_SEC:
                self._fine_yaw_mode = True
                # Init hand RPY reference from current hand orientation
                if track_pose is not None:
                    self._ref_hand_rpy = list(quat_to_rpy(
                        (track_pose["qx"], track_pose["qy"],
                         track_pose["qz"], track_pose["qw"])))
                # Snapshot FK for orientation only (rx/ry/rz = actual arm state),
                # keep x/y/z from _ee_target_filtered (no position jump).
                if self._kin is not None and self._current_q is not None:
                    fk = self._kin.forward_kinematics(list(self._current_q))
                    if fk is not None:
                        self._ref_robot_rpy = [fk.rx, fk.ry, fk.rz]
                        # Update ee_target_filtered orientation to match actual arm
                        self._ee_target_filtered[3] = fk.rx
                        self._ee_target_filtered[4] = fk.ry
                        self._ee_target_filtered[5] = fk.rz
                        if self._target_pose is not None:
                            self._target_pose.rx = fk.rx
                            self._target_pose.ry = fk.ry
                            self._target_pose.rz = fk.rz
                self.get_logger().info("Fine Yaw ON (position frozen, RPY tracking)")
        elif not tp:
            self._trigger_hold_start = None
            if self._fine_yaw_mode:
                self._fine_yaw_mode = False
                self.get_logger().info("Fine Yaw OFF")

        def _save(btns, prev):
            for i in range(min(len(btns), len(prev))):
                b = btns[i]
                if b is None:
                    prev[i] = 0
                elif i <= 1:
                    # analog buttons: use value > 0.5 threshold
                    prev[i] = 1 if float(b.get("value", 0)) > 0.5 else 0
                else:
                    prev[i] = 1 if b.get("pressed") else 0
        if left_btns:
            _save(left_btns, self._prev_btn_left)
        if right_btns:
            _save(right_btns, self._prev_btn_right)

        # ---- Thumbstick Y: gripper velocity control (up=close, down=open, matches SDK) ----
        stick_y = 0.0
        if len(track_axes) >= 4:
            stick_y = track_axes[3]   # thumbstick Y
        elif len(track_axes) >= 2:
            stick_y = track_axes[1]   # fallback
        stick_x_for_grip = track_axes[2] if len(track_axes) >= 3 else 0.0
        # Only activate gripper when stick is primarily vertical
        if abs(stick_y) > STICK_DEADZONE and abs(stick_y) > abs(stick_x_for_grip):
            speed = (-stick_y) * GRIP_SPEED
            self._gripper_angle += speed * self._dt
            self._gripper_angle = max(0.0, min(GRIP_MAX, self._gripper_angle))
            if abs(self._gripper_angle - self._gripper_last_sent) > 0.01:
                self._gripper_last_sent = self._gripper_angle
                self._send_gripper(self._gripper_angle)
                self._gripper_hold_active = self._gripper_angle > 0.03
                self._gripper_last_hold_time = time.monotonic()
                self.get_logger().debug(
                    f"Gripper: stick_y={stick_y:.3f} speed={speed:.2f} → angle={self._gripper_angle:.3f}")
        elif self._gripper_hold_active:
            if self._gripper_angle <= 0.03:
                self._gripper_hold_active = False
            elif time.monotonic() - self._gripper_last_hold_time >= self._gripper_hold_interval:
                self._send_gripper(self._gripper_angle)
                self._gripper_last_hold_time = time.monotonic()

        # ---- Calibration ----
        if not self._calibrated and track_pose is not None and not self._is_moving:
            self._ref_quat = (track_pose["qx"], track_pose["qy"],
                              track_pose["qz"], track_pose["qw"])
            hp = [track_pose["x"], track_pose["y"], track_pose["z"]]
            self._filtered_hand_pos = hp
            self._prev_hand_pos = hp
            self._calibrated = True
            self.get_logger().info(
                f"Calibrated: hand_pos=({track_pose['x']:.3f}, "
                f"{track_pose['y']:.3f}, {track_pose['z']:.3f})")
            return

        # ---- Motion guard ----
        if self._zero_torque or self._is_moving or self._estop:
            self._periodic_status()
            return
        # Fine yaw mode: allow yaw-only tracking even without calibration or grip tracking
        if self._fine_yaw_mode:
            if track_pose is None or self._target_pose is None:
                self._send_filtered()
                self._periodic_status()
                return
        else:
            if self._ee_target_filtered is None or track_pose is None:
                self._periodic_status()
                return
            if not self._calibrated:
                self._periodic_status()
                return

        # ---- Stick base yaw: XBOX-style velocity-form → integrate → IK → filter → publish ----
        STICK_EMA = 0.3
        stick_active = False
        if len(track_axes) >= 3:
            stick_x_raw = track_axes[2]
            if abs(stick_x_raw) < 0.1:
                decay = min(STICK_EMA * 3.0, 1.0)
                self._stick_x_ema = (1 - decay) * self._stick_x_ema
            else:
                self._stick_x_ema = STICK_EMA * stick_x_raw + (1 - STICK_EMA) * self._stick_x_ema
        if abs(self._stick_x_ema) > STICK_DEADZONE and self._target_pose is not None:
            stick_active = True
            if self._tracking_engaged and not getattr(self, '_stick_conflict_warned', False):
                self._stick_conflict_warned = True
                self.get_logger().warn("摇杆Yaw优先，手部追踪已暂停（松开摇杆恢复）")
            ang_vel = -self._stick_x_ema * self._yaw_stick_scale * self._speed_factor
            angle = ang_vel * self._dt
            x, y = self._ee_target_filtered[0], self._ee_target_filtered[1]
            c, s = math.cos(angle), math.sin(angle)
            self._ee_target_filtered[0] = x * c - y * s
            self._ee_target_filtered[1] = x * s + y * c
            self._ee_target_filtered[5] += angle
            self._target_pose = ArmEndPose(
                x=float(self._ee_target_filtered[0]),
                y=float(self._ee_target_filtered[1]),
                z=float(self._ee_target_filtered[2]),
                rx=float(self._ee_target_filtered[3]),
                ry=float(self._ee_target_filtered[4]),
                rz=float(self._ee_target_filtered[5]),
            )
            try:
                q_sol, _ = self._kin.ik_step(
                    self._target_pose, self._ik_seed,
                    damping=5e-3, max_step=self._max_ik_jump)
                if q_sol is not None and self._accept_ik(q_sol):
                    self._ik_raw = q_sol
                    self._ik_seed = list(q_sol)
            except Exception:
                pass

        else:
            self._stick_conflict_warned = False

        if stick_active:
            self._send_filtered()
            self._periodic_status()
            return

        # Fine yaw mode: allow yaw-only tracking even without grip tracking engaged
        if not self._tracking_engaged and not self._fine_yaw_mode:
            self._send_filtered()
            self._periodic_status()
            return
        if self._resync_cooldown > 0:
            self._resync_cooldown -= 1
            self._send_filtered()
            self._periodic_status()
            return

        # ================================================================
        #  Incremental delta control pipeline
        # ================================================================

        sf = self._speed_factor
        cur_hand_raw = [track_pose["x"], track_pose["y"], track_pose["z"]]

        # Hand position EMA filter (reduces tremor before computing delta)
        if self._filtered_hand_pos is None:
            self._filtered_hand_pos = cur_hand_raw
        else:
            a = self._pos_filter_alpha
            self._filtered_hand_pos = [
                a * cur_hand_raw[i] + (1 - a) * self._filtered_hand_pos[i]
                for i in range(3)]
        cur_hand = self._filtered_hand_pos

        if self._prev_hand_pos is None:
            self._prev_hand_pos = cur_hand
            self._send_filtered()
            return

        # Fine yaw mode: freeze position, only track wrist yaw (matches SDK trigger behavior)
        if not self._fine_yaw_mode:
            # Step 1: per-frame hand delta (from filtered position)
            hand_delta = [cur_hand[i] - self._prev_hand_pos[i] for i in range(3)]
            self._prev_hand_pos = cur_hand

            # Step 2: deadband (2mm default)
            dist = math.sqrt(sum(d * d for d in hand_delta))
            if dist < self._deadband:
                hand_delta = [0.0, 0.0, 0.0]

            # Step 3: VR → robot coordinate transform
            # VR→robot coord transform
            # Pico local-floor: +X right, +Y up, +Z forward
            # Robot: -X forward, -Y left, +Z up
            robot_delta = [-hand_delta[2], hand_delta[0], hand_delta[1]]

            # Step 4: scale + speed factor
            scaled_delta = [d * self._pos_scale * sf for d in robot_delta]

            # Step 5: accumulate to raw target
            if self._ee_target_raw is None:
                self._ee_target_raw = [self._ee_target_filtered[0],
                                       self._ee_target_filtered[1],
                                       self._ee_target_filtered[2]]
            for i in range(3):
                self._ee_target_raw[i] += scaled_delta[i]

            # Step 6: workspace limit (spherical radius from origin)
            raw = self._ee_target_raw
            r = math.sqrt(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2])
            if r > self._workspace_radius:
                s = self._workspace_radius / r
                self._ee_target_raw = [raw[0] * s, raw[1] * s, raw[2] * s]

            # Step 7: low-pass filter on position target (RPY stays at calibrated)
            if self._ee_target_filtered is None:
                self._ee_target_filtered = [0.0] * 6
            alpha = self._target_filter_alpha
            for i in range(3):
                if self._ee_target_filtered[i] == 0.0 and self._ee_target_raw[i] != 0.0:
                    self._ee_target_filtered[i] = self._ee_target_raw[i]  # init on first use
                else:
                    self._ee_target_filtered[i] = (alpha * self._ee_target_raw[i] +
                                                   (1 - alpha) * self._ee_target_filtered[i])
        else:
            # Fine yaw mode: skip position update, but still track hand for yaw reference
            self._prev_hand_pos = cur_hand

        # Build target pose (position: filtered, orientation: full RPY tracking in fine-yaw mode)
        if self._fine_yaw_mode:
            cur_quat = (track_pose["qx"], track_pose["qy"],
                         track_pose["qz"], track_pose["qw"])
            cur_roll, cur_pitch, cur_yaw = quat_to_rpy(cur_quat)
            droll  = cur_roll  - self._ref_hand_rpy[0]
            dpitch = cur_pitch - self._ref_hand_rpy[1]
            dyaw   = cur_yaw   - self._ref_hand_rpy[2]
            DEADBAND_DEG = 2.0
            if abs(droll)  < math.radians(DEADBAND_DEG): droll  = 0.0
            if abs(dpitch) < math.radians(DEADBAND_DEG): dpitch = 0.0
            if abs(dyaw)   < math.radians(DEADBAND_DEG): dyaw   = 0.0
            # VR → robot rotation mapping (1:1, matches SDK):
            #   VR roll  (X) → robot rx
            #   VR pitch (Y) → robot rz
            #   VR yaw   (Z) → robot ry
            self._ee_target_filtered[3] = self._ref_robot_rpy[0] + droll  * self._yaw_scale
            self._ee_target_filtered[4] = self._ref_robot_rpy[1] + dyaw   * self._yaw_scale
            self._ee_target_filtered[5] = self._ref_robot_rpy[2] + dpitch * self._yaw_scale

        self._target_pose = ArmEndPose(
            x=float(self._ee_target_filtered[0]),
            y=float(self._ee_target_filtered[1]),
            z=float(self._ee_target_filtered[2]),
            rx=float(self._ee_target_filtered[3]),
            ry=float(self._ee_target_filtered[4]),
            rz=float(self._ee_target_filtered[5]),
        )

        # ---- Step 8: IK with step-jump protection ----
        try:
            q_sol, ik_err = self._kin.ik_step(
                self._target_pose, self._ik_seed,
                damping=5e-3, max_step=self._max_ik_jump)
            if q_sol is not None and self._accept_ik(q_sol):
                self._ik_raw = q_sol
                self._ik_seed = list(q_sol)
                self._consecutive_ik_fails = 0
            else:
                self._consecutive_ik_fails += 1
                if self._consecutive_ik_fails >= 50:
                    self._resync_ik()
        except Exception as e:
            self.get_logger().error(f"IK exception: {e}")
            self._consecutive_ik_fails += 1

        # ---- Step 9: joint velocity limit → publish ----
        self._send_filtered()
        self._periodic_status()

    # ---- Initialization ----

    def _initialize(self) -> None:
        q = list(self._current_q)
        self._ik_seed = list(q)
        self._ik_filter_pos = list(q)
        self._ik_filter_vel = [0.0] * 6
        self._ik_raw = None
        self._seed_just_init = True
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0

        fk = self._kin.forward_kinematics(q)
        self._target_pose = fk
        self._ee_target_raw = [fk.x, fk.y, fk.z]
        self._ee_target_rpy_raw = [fk.rx, fk.ry, fk.rz]
        self._ee_target_filtered = [fk.x, fk.y, fk.z, fk.rx, fk.ry, fk.rz]
        self._ref_robot_rpy = [fk.rx, fk.ry, fk.rz]

        self.get_logger().info(
            f"Initialized: ee=({fk.x:.3f}, {fk.y:.3f}, {fk.z:.3f})m "
            f"rpy=({fk.rx:.2f}, {fk.ry:.2f}, {fk.rz:.2f})rad")
        self.get_logger().info(
            f"Joints: [{', '.join(f'{v:.3f}' for v in q)}]")
        self._initialized = True

    # ---- IK helpers ----

    def _accept_ik(self, q_new: List[float]) -> bool:
        ref = self._ik_seed
        if ref is None:
            return True
        max_diff = max(abs(q_new[i] - ref[i]) for i in range(6))
        if max_diff <= self._max_ik_jump:
            if self._consecutive_rejects > 0:
                self._consecutive_rejects = 0
            self._seed_just_init = False
            return True
        if self._seed_just_init:
            self._seed_just_init = False
            return True
        self._consecutive_rejects += 1
        if self._consecutive_rejects >= 5:
            self.get_logger().warn(
                f"IK jump: {max_diff:.3f}rad, rejected {self._consecutive_rejects}")
        if self._consecutive_rejects >= 50:
            self._resync_ik()
        return False

    def _get_averaged_q(self) -> Optional[List[float]]:
        if not self._q_buffer:
            return list(self._current_q) if self._current_q else None
        n = len(self._q_buffer)
        return [sum(self._q_buffer[s][i] for s in range(n)) / n for i in range(6)]

    def _resync_ik(self) -> None:
        q_avg = self._get_averaged_q()
        if q_avg is None:
            return
        self._ik_seed = list(q_avg)
        self._ik_filter_pos = list(q_avg)
        self._ik_raw = list(q_avg)
        for i in range(6):
            self._ik_filter_vel[i] *= 0.2
        self._seed_just_init = True
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0
        self._resync_cooldown = 5
        fk = self._kin.forward_kinematics(q_avg)
        self._target_pose = fk
        self._ee_target_raw = [fk.x, fk.y, fk.z]
        self._ee_target_rpy_raw = [fk.rx, fk.ry, fk.rz]
        self._ee_target_filtered = [fk.x, fk.y, fk.z, fk.rx, fk.ry, fk.rz]

    # ---- Joint limits & publish ----

    def _send_filtered(self) -> None:
        """2nd-order filter + joint velocity limit → publish."""
        if self._ik_raw is None and self._ik_filter_pos is None:
            return

        if self._ik_filter_pos is None and self._ik_raw is not None:
            self._ik_filter_pos = list(self._ik_raw)
            self._ik_filter_vel = [0.0] * 6

        if self._ik_raw is not None:
            omega = self._filter_omega
            dt = self._dt
            a = omega * dt
            ea = math.exp(-a)
            for i in range(6):
                err = self._ik_raw[i] - self._ik_filter_pos[i]
                vel = self._ik_filter_vel[i]
                err_new = ea * ((1.0 + a) * err - dt * vel)
                vel_new = ea * (omega * omega * dt * err + (1.0 - a) * vel)
                self._ik_filter_pos[i] = self._ik_raw[i] - err_new
                self._ik_filter_vel[i] = vel_new

        self._publish_arm_trajectory(self._ik_filter_pos, self._ik_filter_vel)

    def _apply_joint_velocity_limit(self) -> None:
        """Clamp per-frame joint position change to max_joint_velocity * dt."""
        if self._current_q is None or self._ik_filter_pos is None:
            return
        max_dq = self._max_joint_vel * self._dt
        for i in range(6):
            dq = self._ik_filter_pos[i] - self._current_q[i]
            if abs(dq) > max_dq:
                dq = math.copysign(max_dq, dq)
                self._ik_filter_pos[i] = self._current_q[i] + dq

    def _publish_arm_trajectory(
        self, positions: List[float], velocities: Optional[List[float]] = None
    ) -> None:
        if positions is None:
            return
        msg = JointTrajectory()
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        extrapolated = list(positions)
        if velocities is not None:
            for i in range(6):
                extrapolated[i] += velocities[i] * self._traj_dt
        pt.positions = extrapolated
        if velocities is not None:
            pt.velocities = list(velocities)
        secs = int(self._traj_dt)
        nsecs = int((self._traj_dt - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
        msg.points = [pt]
        self._arm_pub.publish(msg)

    def _send_gripper(self, angle: float = None, stop: bool = False) -> None:
        msg = JointTrajectory()
        msg.joint_names = [GRIPPER_JOINT]
        pt = JointTrajectoryPoint()
        if stop:
            pt.positions = [self._gripper_angle]
        elif angle is not None:
            pt.positions = [angle]
        if pt.positions:
            torque_msg = Float64()
            torque_msg.data = 0.65 if pt.positions[0] > 0.03 else 0.0
            self._gripper_torque_pub.publish(torque_msg)
        pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
        msg.points = [pt]
        self._gripper_pub.publish(msg)

    # ---- Button actions ----

    def _async_move(self, positions: List[float], name: str) -> None:
        if self._is_moving:
            self.get_logger().warn("Another move in progress")
            return
        self.get_logger().info(f"Moving to {name}...")
        self._is_moving = True
        threading.Thread(
            target=self._do_move, args=(positions, name), daemon=True).start()

    def _do_move(self, positions: List[float], name: str) -> None:
        try:
            if self._zero_torque:
                self._switch_to_trajectory_controller()
                self._zero_torque = False
                time.sleep(0.3)
            if self._estop:
                self._estop = False

            n_steps = 20
            if self._ik_filter_pos is not None:
                start = list(self._ik_filter_pos)
            elif self._current_q is not None:
                start = list(self._current_q)
            else:
                start = list(positions)

            for step in range(1, n_steps + 1):
                alpha = step / n_steps
                interp = [s + alpha * (t - s) for s, t in zip(start, positions)]
                self._ik_raw = list(interp)
                self._ik_filter_pos = list(interp)
                self._ik_filter_vel = [0.0] * 6
                msg = JointTrajectory()
                msg.joint_names = list(ARM_JOINTS)
                pt = JointTrajectoryPoint()
                pt.positions = list(interp)
                pt.velocities = [0.0] * 6
                pt.time_from_start = Duration(sec=0, nanosec=50_000_000)
                msg.points = [pt]
                self._arm_pub.publish(msg)
                time.sleep(0.05)

            time.sleep(0.2)
            self._ik_seed = list(positions)
            self._ik_filter_pos = list(positions)
            self._ik_filter_vel = [0.0] * 6
            self._ik_raw = list(positions)
            self._seed_just_init = True
            self._consecutive_rejects = 0
            self._consecutive_ik_fails = 0

            fk = self._kin.forward_kinematics(positions)
            self._target_pose = fk
            self._ee_target_raw = [fk.x, fk.y, fk.z]
            self._ee_target_rpy_raw = [fk.rx, fk.ry, fk.rz]
            self._ee_target_filtered = [fk.x, fk.y, fk.z, fk.rx, fk.ry, fk.rz]
            self._calibrated = False
            self._prev_hand_pos = None
            self.get_logger().info(f"Arrived at {name}")
        except Exception as e:
            self.get_logger().error(f"Move failed: {e}")
        finally:
            self._is_moving = False

    def _emergency_stop(self) -> None:
        self._estop = True
        if self._ik_filter_pos is not None:
            msg = JointTrajectory()
            msg.joint_names = list(ARM_JOINTS)
            pt = JointTrajectoryPoint()
            pt.positions = list(self._ik_filter_pos)
            pt.time_from_start = Duration(sec=1, nanosec=0)
            msg.points = [pt]
            self._arm_pub.publish(msg)
        self.get_logger().error("!!! ESTOP !!!")

    def _toggle_zero_torque(self) -> None:
        """Toggle zero-torque mode (matches SDK: left thumbstick hold 0.3s)."""
        new_state = not self._zero_torque
        self.get_logger().info("%s zero-torque mode...", "Enabling" if new_state else "Disabling")
        if new_state:
            # Switch to zero_torque_controller, deactivate arm + gripper
            self._call_switch_controller(
                activate=["zero_torque_controller"],
                deactivate=["arm_controller", "gripper_controller"])
            self._zero_torque = True
            self._tracking_engaged = False
            self.get_logger().info(">>> Zero-torque ON: arm can be moved by hand <<<")
        else:
            # Switch back to arm_controller + gripper_controller
            self._call_switch_controller(
                activate=["arm_controller", "gripper_controller"],
                deactivate=["zero_torque_controller"])
            self._zero_torque = False
            self._calibrated = False
            self._resync_ik()
            self.get_logger().info(">>> Zero-torque OFF: Pico control restored <<<")

    def _disable_motors(self) -> None:
        self._estop = True
        self._zero_torque = False
        self._tracking_engaged = False
        self._call_switch_controller(
            activate=[],
            deactivate=["arm_controller", "gripper_controller"])
        self.get_logger().error("Motors disabled (controllers deactivated)")

    def _enable_motors(self) -> None:
        self._estop = False
        self._tracking_engaged = False
        self._call_switch_controller(
            activate=["arm_controller", "gripper_controller"],
            deactivate=[])
        self._calibrated = False
        self.get_logger().info("Motors re-enabled")

    def _switch_to_trajectory_controller(self) -> None:
        self._call_switch_controller(
            activate=["arm_controller"],
            deactivate=["zero_torque_controller"])

    def _call_switch_controller(
        self, activate: List[str], deactivate: List[str]
    ) -> None:
        if not self._switch_ctrl_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("controller_manager switch service unavailable")
            return
        from controller_manager_msgs.srv import SwitchController
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self._switch_ctrl_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None and future.result().ok:
            self.get_logger().info(f"Controller switch: activate={activate}")
        else:
            self.get_logger().error("Controller switch failed")

    # ---- Diagnostics ----

    def _periodic_status(self) -> None:
        self._diag_tick += 1
        if self._diag_tick < int(self._rate * 5):
            return
        self._diag_tick = 0
        if self._current_q is None:
            return
        degs = [f"{v * 180 / math.pi:.1f}" for v in self._current_q]
        if self._zero_torque:
            mode = "zero_torque"
        elif self._estop:
            mode = "ESTOP"
        elif self._grip_active:
            mode = "grip"
        elif not self._calibrated:
            mode = "not_calib"
        elif self._tracking_engaged:
            mode = "tracking"
        else:
            mode = "idle"
        self.get_logger().info(f"[{mode}] joints(deg): [{', '.join(degs)}]")
        if self._ee_target_filtered is not None:
            e = self._ee_target_filtered
            self.get_logger().info(
                f"ee_target: ({e[0]:.4f}, {e[1]:.4f}, {e[2]:.4f})m "
                f"rpy=({e[3]:.2f}, {e[4]:.2f}, {e[5]:.2f})rad")


def main(args=None):
    rclpy.init(args=args)
    node = PicoTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
