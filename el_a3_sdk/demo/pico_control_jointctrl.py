#!/usr/bin/env python3
"""
EL-A3 Pico VR 手柄遥操作控制程序（纯 SDK，无 ROS 依赖）

支持双机械臂：左手柄控制左臂，右手柄控制右臂。
通过读取 pico3_webxr_pose_receiver.py 写入的 /tmp/pico_latest_pose.json
获取 Pico 手柄位姿数据，每只手柄的 grip 位姿增量独立映射到对应机械臂末端。

控制映射 (每臂独立):
  右手柄 A / 左手柄 X  → 对应机械臂回零位
  右手柄 B / 左手柄 Y  → 对应机械臂回 Home
  左手摇杆按下          → 切换零力矩模式
  右手摇杆按下          → 急停
  摇杆上推/下推         → 夹爪收紧/打开
  摇杆左推/右推         → 基座 yaw 旋转
  侧边按键(长按1秒)    → 开启跟踪，松开关闭

坐标系映射 (Pico WebXR local-floor → 机器人基座标):
  Pico 左 (-X) → 机器人 -Y
  Pico 上 (+Y) → 机器人 +Z
  Pico 前 (-Z) → 机器人 -X

用法:
  # 仿真模式（无需硬件，纯 FK/IK 模拟）
  python3 pico_control.py --sim

  # 双机械臂
  python3 pico_control.py --can-left can0 --can-right can1

  # 单机械臂（仅右手柄）
  python3 pico_control.py --can can0

  # 自定义参数
  python3 pico_control.py --can-left can0 --can-right can1 --pos-scale 0.3 --deadzone 0.03 --debug
"""

import os
import time
import threading
import math
import json
import signal
import argparse
import logging
from typing import List, Optional, Dict, Any, Tuple

from el_a3_sdk import ELA3Interface, ArmEndPose, LogLevel, ArmState
from el_a3_sdk.kinematics import ELA3Kinematics

logger = logging.getLogger("pico_control_jointctrl")

# ================================================================
# Simulated Arm (for --sim mode)
# ================================================================


class _JointStates:
    """Minimal joint states for simulation, compatible with GetArmJointMsgs()."""
    def __init__(self, positions: List[float]):
        self._positions = list(positions)

    def to_list(self) -> List[float]:
        return list(self._positions)


class SimArm:
    """Simulated arm that uses FK/IK without CAN hardware.

    Provides the same interface as ELA3Interface for the methods used by
    _ArmController and PicoArmController. All motion is computed via
    kinematics only, with simple interpolation for MoveJ.
    """

    def __init__(self, label: str):
        self._label = label
        self._kin: Optional[ELA3Kinematics] = None
        self._joints = [0.0] * 7
        self._enabled = False
        self._estop = False
        self._zero_torque = False
        self._gripper_angle = 0.0
        self._tick = 0

        try:
            self._kin = ELA3Kinematics()
        except Exception:
            pass

    def _get_kinematics(self):
        return self._kin

    def ConnectPort(self) -> bool:
        logger.info("[%s 模拟] 已连接", self._label)
        return True

    def DisconnectPort(self):
        logger.info("[%s 模拟] 已断开", self._label)

    def EnableArm(self, motor_num: int = 0xFF, **kwargs) -> bool:
        self._enabled = True
        return True

    def DisableArm(self, **kwargs) -> bool:
        self._enabled = False
        return True

    def start_control_loop(self, rate_hz: float = 200.0):
        pass

    def stop_control_loop(self):
        pass

    def MoveL(self, target_pose, duration: float = 2.0, n_waypoints: int = 50,
              block: bool = True, **kwargs) -> bool:
        if self._kin is None:
            return False
        q_sol = self._kin.inverse_kinematics(target_pose, q_init=list(self._joints[:6]),
                                              max_iter=120, damping=5e-3)
        if q_sol is None:
            logger.warning("[%s 模拟] MoveL IK 失败", self._label)
            return False
        return self.MoveJ(q_sol, duration=duration, block=block)

    def MoveJ(self, positions: List[float], duration: float = 2.0, block: bool = True) -> bool:
        if self._kin is None:
            return False

        start = list(self._joints[:6])
        target = list(positions[:6])
        steps = max(int(duration * 50), 1)

        if block:
            for i in range(steps + 1):
                t = i / max(steps, 1)
                # Smoothstep interpolation
                t_smooth = t * t * (3 - 2 * t)
                for j in range(6):
                    self._joints[j] = start[j] + (target[j] - start[j]) * t_smooth
                if i < steps:
                    time.sleep(duration / (steps + 1))
        else:
            for j in range(6):
                self._joints[j] = target[j]

        pose = self._kin.forward_kinematics(self._joints[:6])
        logger.info("[%s 模拟] MoveJ → (%.3f, %.3f, %.3f)", self._label, pose.x, pose.y, pose.z)
        return True

    def JointCtrl(self, *positions, velocities=None) -> bool:
        if len(positions) >= 6:
            for i in range(6):
                self._joints[i] = float(positions[i])
        self._tick += 1
        return True

    def GripperCtrl(self, gripper_angle: float = None, stop: bool = False, **kwargs) -> bool:
        if stop:
            logger.info("[%s 模拟] 夹爪停止", self._label)
        elif gripper_angle is not None:
            self._gripper_angle = gripper_angle
            logger.info("[%s 模拟] 夹爪: %.2f rad", self._label, gripper_angle)
        return True

    def GripperCurrentCtrl(self, iq_ref: float = 0.0, **kwargs) -> bool:
        logger.info("[%s 模拟] 夹爪电流保持: %.3f A", self._label, iq_ref)
        return True

    def GripperPositionCtrl(self, gripper_angle: float = 0.0, **kwargs) -> bool:
        self._gripper_angle = gripper_angle
        logger.info("[%s 模拟] 夹爪位置模式: %.2f rad", self._label, gripper_angle)
        return True

    def ZeroTorqueMode(self, enable: bool, **kwargs) -> bool:
        self._zero_torque = enable
        logger.info("[%s 模拟] 零力矩: %s", self._label, "开启" if enable else "关闭")
        return True

    def EmergencyStop(self) -> bool:
        self._estop = True
        logger.info("[%s 模拟] 急停!", self._label)
        return True

    def GetArmJointMsgs(self):
        return _JointStates(self._joints)

    def GetCanFps(self) -> float:
        return 0.0

    def GetCanTxStats(self):
        return (0, 0, 0.0)

    def GetCanBusState(self) -> str:
        return "SIM"

SPEED_LEVELS = [
    ("极慢", 0.10),
    ("慢",   0.25),
    ("中",   0.50),
    ("快",   0.75),
    ("最大", 1.00),
]

HOME_POSITIONS = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
ZERO_POSITIONS = [0.0] * 6

POSE_FILE = "/tmp/pico_latest_pose.json"

# WebXR xr-standard button indices
BTN_TRIGGER = 0
BTN_GRIP = 1
BTN_THUMBSTICK = 3
BTN_XA = 4   # X (left) / A (right)
BTN_YB = 5   # Y (left) / B (right)

GRIP_LONG_PRESS_SEC = 0.3


# ---- Pico data helpers ----

def _read_pico_pose(retries: int = 2, retry_delay: float = 0.001) -> Optional[Dict[str, Any]]:
    for attempt in range(max(1, retries)):
        try:
            with open(POSE_FILE, "r") as f:
                return json.load(f)
        except FileNotFoundError:
            return None
        except (json.JSONDecodeError, OSError):
            if attempt + 1 >= max(1, retries):
                return None
            time.sleep(retry_delay)
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


def _extract_buttons(source: Dict[str, Any]) -> List[Dict[str, Any]]:
    gp = source.get("gamepad")
    if not gp:
        return []
    return gp.get("buttons", [])


def _extract_axes(source: Dict[str, Any]) -> List[float]:
    gp = source.get("gamepad")
    if not gp:
        return []
    return gp.get("axes", [])


def _thumbstick_axes(axes: List[float]) -> Tuple[float, float]:
    """Return the active thumbstick x/y pair across Pico/WebXR axis layouts."""
    if len(axes) >= 4:
        primary = (float(axes[0]), float(axes[1]))
        xr_standard = (float(axes[2]), float(axes[3]))
        if max(abs(xr_standard[0]), abs(xr_standard[1])) > max(abs(primary[0]), abs(primary[1])):
            return xr_standard
        return primary
    if len(axes) >= 2:
        return float(axes[0]), float(axes[1])
    return 0.0, 0.0


def _reject_viewer_correlated_delta(
    hand_delta: List[float],
    viewer_delta: Optional[List[float]],
) -> List[float]:
    """Remove controller drift that is almost the same motion as the headset."""
    if viewer_delta is None:
        return hand_delta
    v_norm2 = sum(v * v for v in viewer_delta)
    if v_norm2 < 1e-6:
        return hand_delta
    h_norm2 = sum(v * v for v in hand_delta)
    if h_norm2 < 1e-9:
        return hand_delta
    dot = sum(hand_delta[i] * viewer_delta[i] for i in range(3))
    if dot <= 0.0:
        return hand_delta
    h_norm = math.sqrt(h_norm2)
    v_norm = math.sqrt(v_norm2)
    corr = dot / max(h_norm * v_norm, 1e-9)
    projection = dot / v_norm2
    if corr < 0.85 or projection < 0.5 or projection > 1.5:
        return hand_delta
    return [hand_delta[i] - projection * viewer_delta[i] for i in range(3)]


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


def _map_hand_delta_to_robot_delta(hand_delta: List[float]) -> List[float]:
    """Map WebXR hand translation delta to robot base translation delta."""
    # Current Pico/WebXR setup axis order relative to desired robot teleop:
    # -hand X = left/right, hand Z = up/down, hand Y = front/back.
    return [-hand_delta[0], hand_delta[2], hand_delta[1]]


# ================================================================
# ================================================================
# Gripper Hold Controller (reusable, stall-protected continuous hold)
# ================================================================

class GripperHold:
    """Continuously maintain gripper position with grasp-adaptive locking.

    State machine:
      CLOSING  — stick is driving the target angle; gripper is moving.
      GRASPED  — stall confirmed (object contacted); use low-duty-cycle
                 current pulses with thermal derating.
    """

    CLOSING = "closing"
    GRASPED = "grasped"

    def __init__(self, arm, hold_interval: float = 0.25):
        self._arm = arm
        self._hold_interval = hold_interval
        self._active = False
        self._state = self.CLOSING
        self._hold_preload = 0.002
        self._target_angle = 0.0
        self._effort = 1.1                  # Nm torque limit while closing
        self._hold_effort = 0.16            # low thermal load after contact lock
        # L7 angle increases while closing in position mode. On this hardware,
        # positive IQ opens the gripper; negative IQ keeps the closing direction.
        self._hold_iq_ref = -0.08           # current-mode hold command
        self._last_hold_time = 0.0
        self._stall_count = 0
        self._grasp_confirm_ticks = 2       # confirm fast, before drive over-current protection
        # Hard-stop / stuck detection
        self._last_positions = [0.0] * 8  # ring buffer of recent positions
        self._last_pos_idx = 0
        self._pos_window_valid = False
        self._hold_fault_code = 0
        self._last_feedback_pos = 0.0
        self._last_feedback_vel = 0.0
        self._slip_margin = 0.035
        self._manual_open_unlock = 0.08
        self._slip_regrip_effort = 0.22
        self._last_slip_regrip_time = 0.0
        self._protection_hold = False
        self._hold_mode = "position"
        self._last_protection_log = 0.0
        self._last_fault_recover_time = 0.0
        self._iq_ref_cap = 0.18
        self._iq_ref_floor = 0.04
        self._grasp_open_margin = 0.03
        self._manual_cmd_margin = 0.006
        self._pulse_phase_start = 0.0
        self._pulse_high = True
        self._slip_boost_until = 0.0
        self._temp_soft_limit = 40.0
        self._temp_mid_limit = 50.0
        self._temp_hard_limit = 60.0
        self._temp_shutdown_limit = 68.0

    # ---- public API ----

    def grip(self, angle: float, effort: float = 1.5):
        """Start / update hold target.

        In CLOSING state the target tracks the stick freely.
        Once GRASPED the target is locked near the current feedback
        position and only opening commands are honored directly.
        """
        angle = max(0.0, angle)
        if self._state == self.GRASPED:
            manual_open = angle < self._target_angle - self._manual_cmd_margin
            manual_close = angle > self._target_angle + self._manual_cmd_margin
            if not manual_open and not manual_close:
                return
            self._state = self.CLOSING
            self._stall_count = 0
            self._pos_window_valid = False
            if angle < self._grasp_open_margin:
                self.release()
                return

            # Any explicit stick command takes ownership back from the grasp
            # lock. Closing will run contact detection again; opening is sent
            # directly as a normal position command.
            self._target_angle = angle
            self._effort = effort
            self._set_position_hold(self._target_angle, self._effort)
            self._last_hold_time = time.monotonic()
            self._active = True
            return
        self._target_angle = angle
        self._effort = effort
        self._set_position_hold(self._target_angle, self._effort)
        self._last_hold_time = time.monotonic()
        self._active = True

    def release(self):
        """Stop hold loop."""
        self._active = False
        self._state = self.CLOSING
        self._stall_count = 0
        self._protection_hold = False
        self._hold_mode = "position"
        self._pos_window_valid = False

    def tick(self):
        """Call every control cycle (~100 Hz)."""
        self.maintain_continuous_grip()

    def maintain_continuous_grip(self, now: float | None = None) -> bool:
        """Reusable hold loop: keep the gripper closed and monitor health.

        Returns True when a hold command was sent in this call.
        """
        if not self._active:
            return False

        if now is None:
            now = time.monotonic()

        # ---- GRASPED state: low-heat position lock ----
        if self._state == self.GRASPED:
            fb7 = None
            if not hasattr(self, "_last_health_log"):
                self._last_health_log = 0.0
            if now - self._last_health_log > 5.0:
                self._last_health_log = now
                try:
                    ms = self._arm.GetMotorStates()
                    fb7 = ms.get(7)
                    if fb7 and fb7.is_valid:
                        logger.info(
                            "[GripperHold] 保持中: pos=%.3f temp=%.1f°C "
                            "fault=%d torque=%.2f",
                            fb7.position, fb7.temperature, fb7.fault_code,
                            fb7.torque)
                        if fb7.fault_code != 0:
                            logger.error(
                                "[GripperHold] 电机7 故障码=%d (0x%02X) 温度=%.1f°C",
                                fb7.fault_code, fb7.fault_code, fb7.temperature)
                except Exception:
                    pass

            fb7 = self._read_feedback()
            if fb7 is not None and int(getattr(fb7, "fault_code", 0) or 0) != 0:
                if now - self._last_protection_log >= 2.0:
                    self._last_protection_log = now
                    logger.error(
                        "[GripperHold] 电机7 已故障，停止保持: fault=%d temp=%.1f°C pos=%.3f",
                        fb7.fault_code, fb7.temperature, fb7.position)
                self._last_hold_time = now
                self.release()
                return True
            temp = float(getattr(fb7, "temperature", 0.0) or 0.0) if fb7 else 0.0
            if temp >= self._temp_shutdown_limit:
                logger.error(
                    "[GripperHold] 温度过高，停止保持: temp=%.1f°C pos=%.3f",
                    temp, getattr(fb7, "position", self._target_angle) if fb7 else self._target_angle)
                self.release()
                return True

            if fb7 is not None and self._slip_detected(fb7):
                if now - self._last_slip_regrip_time >= 0.8:
                    self._last_slip_regrip_time = now
                    if temp < self._temp_soft_limit:
                        self._target_angle = max(self._target_angle, fb7.position + self._hold_preload)
                        self._set_position_hold(self._target_angle, self._slip_regrip_effort)
                    self._last_hold_time = now
                    logger.info(
                        "[GripperHold] 检测到滑移，低力矩补偿: pos=%.3f target=%.3f effort=%.2f temp=%.1f°C",
                        fb7.position, self._target_angle, self._slip_regrip_effort, temp)
                    return True

            if now - self._last_hold_time < self._hold_interval:
                return False
            self._set_position_hold(self._target_angle, self._thermal_hold_effort(temp))
            self._last_hold_time = now
            return True

        # ---- CLOSING state: active tracking ----
        if now - self._last_hold_time < self._hold_interval:
            return False

        fb7 = self._read_feedback()
        if self._detect_grasp(fb7):
            return True

        self._set_position_hold(self._target_angle, self._effort)
        self._last_hold_time = now
        return True

    # ---- internals ----

    def _read_feedback(self):
        try:
            ms = self._arm.GetMotorStates()
            fb7 = ms.get(7)
            if fb7 is None or not fb7.is_valid:
                return None
            self._last_feedback_pos = fb7.position
            self._last_feedback_vel = fb7.velocity
            if not self._pos_window_valid:
                self._reset_position_window(fb7.position)
            return fb7
        except Exception:
            return None

    def _reset_position_window(self, position: float) -> None:
        self._last_positions = [position] * len(self._last_positions)
        self._last_pos_idx = 0
        self._pos_window_valid = True

    def _update_hold_profile(self, fb7) -> None:
        return

    def _slip_detected(self, fb7) -> bool:
        return (
            fb7.position < self._target_angle - self._slip_margin
            or abs(fb7.velocity) > 0.12
        )

    def _detect_grasp(self, fb7=None) -> bool:
        """When the motor pushes against an object and can't reach the
        commanded target, we confirm the stall and lock the target to
        the current *actual* position so the gripper stops squeezing."""
        if fb7 is None:
            self._stall_count = max(0, self._stall_count - 1)
            return False

        # Log fault code and temperature for diagnosis
        if fb7.fault_code != 0:
            logger.error(
                "[GripperHold] 电机7 故障码=%d (0x%02X) 温度=%.1f°C 力矩=%.2f Nm",
                fb7.fault_code, fb7.fault_code, fb7.temperature, fb7.torque)

        pos_err = self._target_angle - fb7.position  # positive = still closing
        torque_ratio = abs(fb7.torque) / max(self._effort, 0.1)

        # Track position for stuck/hard-stop detection
        self._last_positions[self._last_pos_idx] = fb7.position
        self._last_pos_idx = (self._last_pos_idx + 1) % len(self._last_positions)
        pos_spread = max(self._last_positions) - min(self._last_positions)

        closing_cmd = pos_err > 0.01
        slow_or_stopped = abs(fb7.velocity) < 0.10 or pos_spread < 0.012
        torque_contact = abs(fb7.torque) > 0.28 or torque_ratio > 0.28
        # Contact trend: target is still closing, but feedback is barely moving
        # and torque is rising. Lock before the motor reaches stall protection.
        contact_trend = closing_cmd and slow_or_stopped and torque_contact
        # Hard stop catches mechanical limits and very hard objects.
        hard_stop = (pos_spread < 0.008 and abs(fb7.torque) > 0.25
                     and self._target_angle > 0.05)

        if contact_trend or hard_stop:
            self._stall_count += 1
            if self._stall_count >= self._grasp_confirm_ticks:
                if hard_stop and not contact_trend:
                    reason = "硬限位趋势"
                else:
                    reason = "接触趋势"
                self._lock_grasp(fb7, reason, pos_spread)
                return True
        else:
            self._stall_count = max(0, self._stall_count - 1)
        return False

    def _lock_grasp(self, fb7, reason: str, pos_spread: float) -> None:
        locked = round(max(0.0, fb7.position + self._hold_preload), 4)
        self._target_angle = locked
        self._state = self.GRASPED
        self._stall_count = 0
        self._pulse_phase_start = 0.0
        self._pulse_high = False
        self._set_position_hold(self._target_angle, self._thermal_hold_effort(fb7.temperature))
        self._last_hold_time = time.monotonic()
        logger.info(
            "[GripperHold] %s: lock=%.3f rad target=%.3f torque=%.2f "
            "vel=%.3f spread=%.4f temp=%.1f°C effort=%.2f",
            reason, locked, self._target_angle, fb7.torque, fb7.velocity,
            pos_spread, fb7.temperature, self._thermal_hold_effort(fb7.temperature))

    def _set_position_hold(self, angle: float, effort: float) -> None:
        self._hold_mode = "position"
        self._arm.GripperCtrl(gripper_angle=angle, gripper_effort=effort)

    def _set_current_hold(self, iq_ref: float) -> None:
        self._hold_mode = "current"
        self._arm.GripperCurrentCtrl(
            iq_ref=self._clamp_iq(iq_ref),
            limit_cur=0.9,
            cur_kp=0.10,
            cur_ki=0.008,
            cur_filt_gain=0.08,
        )

    def _clamp_iq(self, iq_ref: float) -> float:
        if abs(iq_ref) < 1e-6:
            return 0.0
        sign = -1.0 if iq_ref < 0.0 else 1.0
        mag = min(self._iq_ref_cap, max(self._iq_ref_floor, abs(iq_ref)))
        return sign * mag

    def _scaled_hold_iq(self, scale: float) -> float:
        return self._clamp_iq(self._hold_iq_ref * scale)

    def _grasp_hold_profile(self, temp: float, now: float) -> tuple[float, float, float]:
        abs_iq = min(self._iq_ref_cap, max(self._iq_ref_floor, abs(self._hold_iq_ref)))
        pulse_on = 0.12
        pulse_off = 0.85

        if temp >= self._temp_soft_limit:
            abs_iq *= 0.85
            pulse_on = 0.10
            pulse_off = 1.10
        if temp >= self._temp_mid_limit:
            abs_iq *= 0.72
            pulse_on = 0.08
            pulse_off = 1.60
        if temp >= self._temp_hard_limit:
            abs_iq *= 0.55
            pulse_on = 0.05
            pulse_off = 2.20

        if now < self._slip_boost_until:
            abs_iq = min(self._iq_ref_cap, abs_iq * 1.25)
            pulse_on = max(pulse_on, 0.12)
            pulse_off = min(pulse_off, 0.60)

        return -abs_iq, pulse_on, pulse_off

    def _thermal_hold_effort(self, temp: float) -> float:
        effort = self._hold_effort
        if temp >= self._temp_soft_limit:
            effort *= 0.75
        if temp >= self._temp_mid_limit:
            effort *= 0.55
        if temp >= self._temp_hard_limit:
            effort *= 0.35
        return max(0.05, effort)

    def open_command(self, angle: float, effort: float = 1.0) -> None:
        angle = max(0.0, angle)
        self._target_angle = angle
        self._effort = effort
        self._set_position_hold(self._target_angle, self._effort)
        self._last_hold_time = time.monotonic()
        if angle < 0.03:
            self.release()

    # ---- properties ----

    @property
    def active(self) -> bool:
        return self._active

    @property
    def target_angle(self) -> float:
        return self._target_angle

    @property
    def state(self) -> str:
        return self._state

    @property
    def protection_hold(self) -> bool:
        return self._protection_hold


# ================================================================
# Single Arm Controller
# ================================================================

class _ArmController:
    """单个机械臂的控制器，管理一个 ELA3Interface + IK 状态"""

    def __init__(
        self,
        name: str,
        arm: ELA3Interface,
        update_rate: float,
        max_lin_vel: float,
        max_ang_vel: float,
        pos_scale: float,
        deadzone: float,
        input_alpha: float,
        filter_omega: float,
        max_ik_jump: float,
        yaw_scale: float = 1.0,
        joint_max_vel: float = 0.2,
    ):
        self.name = name
        self._arm = arm
        self._rate = update_rate
        self._dt = 1.0 / update_rate
        self._max_lin_vel = max_lin_vel
        self._max_ang_vel = max_ang_vel
        self._pos_scale = pos_scale
        self._dz_threshold = deadzone
        self._input_alpha = input_alpha
        self._filter_omega = filter_omega
        self._max_ik_jump = max_ik_jump
        self._yaw_scale = yaw_scale
        self._joint_max_vel = float(joint_max_vel)
        self._yaw_stick_scale = 1.5
        self._stick_x_ema = 0.0
        self._yaw_active = False

        self._kin = arm._get_kinematics()

        # Per-arm state
        self.is_moving = False
        self.is_estop = False
        self._calibrated = False
        self._prev_hand_pos: Optional[List[float]] = None  # for per-frame delta
        self._last_raw_hand_pos: Optional[List[float]] = None
        self._hand_filtered: Optional[List[float]] = None
        self._sv_vel: Optional[List[float]] = None  # EMA-smoothed velocity
        self._target_pose: Optional[ArmEndPose] = None
        self._smooth_pose: Optional[ArmEndPose] = None  # LPF平滑后的目标位姿
        self._last_ik_err: float = 0.0  # 上帧IK残差, 用于奇异区减速

        # IK state
        self._ik_seed: Optional[List[float]] = None
        self._ik_raw: Optional[List[float]] = None  # latest IK solution

        # Unitree-style: weighted moving average on joint output (last 4 frames)
        self._ik_filter_pos: Optional[List[float]] = None
        self._ik_filter_vel: Optional[List[float]] = None
        self._hold_q: Optional[List[float]] = None
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0
        self._seed_just_init = False
        self._resync_cooldown = 0
        self._joint_ma_window: List[List[float]] = []
        self._joint_ma_weights = [0.4, 0.3, 0.2, 0.1]
        self._last_joint_cmd: Optional[List[float]] = None

        # EMA
        self._sv = [0.0] * 6

        # Gripper
        self._gripper_angle = 0.0
        self._gripper_last_sent = -1.0
        self._grip_hold_start: Optional[float] = None
        self._grip_active = False
        self._gripper_hold = GripperHold(arm, hold_interval=0.04)
        self._gripper_available = self._has_motor_feedback(7)
        self._last_gripper_missing_log = 0.0
        self._tracking_engaged = False
        self._skip_frames = 0  # skip N frames after tracking engage
        self._sample_interval = 1  # process every new Pico packet (~30Hz)
        self._sample_counter = 0

        # Fine yaw mode: freeze position, only track wrist yaw
        self._fine_yaw_mode = False
        self._trigger_hold_start: Optional[float] = None

        # Button edge detection
        self._prev_buttons = [0] * 8
        self._a_hold_start: Optional[float] = None

        # Payload mode: auto-boost kp/kd/gravity when gripper closes
        self._payload_mode = False
        self._saved_kp: Optional[float] = None
        self._saved_kd: Optional[float] = None
        self._saved_grav: Optional[float] = None

    @property
    def calibrated(self) -> bool:
        return self._calibrated

    @property
    def grip_active(self) -> bool:
        return self._grip_active

    @property
    def target_pose(self) -> Optional[ArmEndPose]:
        return self._target_pose

    def _has_motor_feedback(self, motor_id: int) -> bool:
        if not hasattr(self._arm, "GetMotorStates"):
            return True
        try:
            return motor_id in self._arm.GetMotorStates()
        except Exception:
            return True

    def init_position(self):
        q = self._arm.GetArmJointMsgs().to_list()[:6]
        logger.info("[%s] 从当前关节位置初始化: %s", self.name,
                    [f"{v*180/math.pi:.1f}°" for v in q])
        self._last_joint_cmd = list(q)
        self._ik_seed = list(q)
        self._ik_raw = list(q)
        self._ik_filter_pos = list(q)
        self._ik_filter_vel = [0.0] * 6
        self._hold_q = list(q)
        self._seed_just_init = True
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0
        self._ref_quat = None
        self._ref_hand_rpy: Optional[List[float]] = None  # [roll, pitch, yaw] in VR space
        self._prev_hand_pos = None
        self._last_raw_hand_pos = None
        self._joint_ma_window = []
        self._ee_target_raw = None
        self._ee_target_filtered = None
        self._last_ik_target = None
        self._calibrated = False
        self._last_sent_pose_tuple = None
        self._smooth_pose = None
        if self._kin is not None:
            fk = self._kin.forward_kinematics(q)
            self._target_pose = fk
            self._ee_target_raw = [fk.x, fk.y, fk.z]
            self._ee_target_filtered = None
            self._prev_pose = None
            self._ref_robot_rpy = [fk.rx, fk.ry, fk.rz]

    def calibrate(self, pose: Dict[str, Any]):
        self._ref_quat = (pose["qx"], pose["qy"], pose["qz"], pose["qw"])
        hp = [pose["x"], pose["y"], pose["z"]]
        self._prev_hand_pos = list(hp)
        self._last_raw_hand_pos = list(hp)
        self._hand_filtered = list(hp)
        self._joint_ma_window = []
        self._calibrated = True
        logger.info("[%s] 标定完成: hand_pos=(%.3f, %.3f, %.3f)",
                    self.name, hp[0], hp[1], hp[2])

    def reset_calibration(self):
        self._ref_quat = None
        self._ref_hand_rpy = None
        self._prev_hand_pos = None
        self._last_raw_hand_pos = None
        self._hand_filtered = None
        self._joint_ma_window = []
        self._calibrated = False

    def move_to(self, positions: List[float], label: str):
        if self.is_estop:
            logger.info("[%s] 急停态忽略 %s 指令，长按 A/X 1 秒恢复使能", self.name, label)
            return
        self.is_moving = True
        threading.Thread(target=self._do_move, args=(positions, label), daemon=True).start()

    def _do_move(self, positions: List[float], label: str):
        try:
            self._arm.MoveJ(positions, duration=2.0, block=True)
            self._ik_seed = list(positions)
            self._ik_raw = list(positions)
            self._ik_filter_pos = list(positions)
            self._ik_filter_vel = [0.0] * 6
            self._hold_q = list(positions)
            self._joint_ma_window = []
            self._last_joint_cmd = list(positions)
            self._seed_just_init = True
            self._consecutive_rejects = 0
            self._consecutive_ik_fails = 0
            self._last_sent_pose_tuple = None
            if self._kin is not None:
                fk = self._kin.forward_kinematics(positions)
                self._target_pose = fk
                self._smooth_pose = ArmEndPose(x=fk.x, y=fk.y, z=fk.z,
                                               rx=fk.rx, ry=fk.ry, rz=fk.rz)
                self._last_sent_pose_tuple = (fk.x, fk.y, fk.z, fk.rx, fk.ry, fk.rz)
                self._ee_target_raw = None
                self._ee_target_filtered = None
                self._prev_pose = None
            self.reset_calibration()
            logger.info("[%s] 已到达 %s", self.name, label)
        except Exception as e:
            logger.error("[%s] 运动异常: %s", self.name, e)
        finally:
            self.is_moving = False

    def toggle_gripper_long_press(self, grip_val: int, prev_grip_val: int):
        """Handle grip button long-press toggle."""
        grip_pressed = grip_val == 1
        grip_was_pressed = prev_grip_val == 1

        if grip_pressed and not grip_was_pressed:
            self._grip_hold_start = time.monotonic()
        elif grip_pressed and grip_was_pressed and self._grip_hold_start is not None:
            if not self._grip_active and (time.monotonic() - self._grip_hold_start) >= GRIP_LONG_PRESS_SEC:
                self._grip_active = True
                ms = self._arm.GetMotorStates()
                fb7 = ms.get(7)
                if fb7 and fb7.is_valid:
                    self._gripper_angle = fb7.position
                self._arm.GripperPositionCtrl(gripper_angle=self._gripper_angle)
                self._arm.GripperCtrl(gripper_angle=self._gripper_angle, gripper_effort=1.0)
                logger.info("[%s] 侧边键长按: 夹爪位置保持", self.name)
        elif not grip_pressed and grip_was_pressed:
            if self._grip_active:
                self._grip_active = False
                self._arm.GripperCtrl(stop=True)
                logger.info("[%s] 侧边键松开: 停止夹爪 + 恢复控制", self.name)
            self._grip_hold_start = None

    def emergency_stop(self):
        logger.info("[%s] emergency_stop() 开始", self.name)
        self._gripper_hold.release()
        self._arm.GripperPositionCtrl(gripper_angle=0.0)
        self._arm.GripperCtrl(gripper_angle=0.0, gripper_effort=1.5)
        time.sleep(0.05)
        self._gripper_angle = 0.0
        self._gripper_last_sent = -1.0
        self._arm.EmergencyStop()
        self.is_estop = True
        logger.info("[%s] emergency_stop() 完成, is_estop=True", self.name)

    def recover_from_estop(self):
        self._arm.EnableArm()
        time.sleep(0.3)
        self._arm.start_control_loop(rate_hz=200.0)
        self.is_estop = False

    def set_payload_mode(self, enabled: bool):
        """Auto-switch control parameters when gripper closes/opens on a payload."""
        arm = self._arm
        lock = getattr(arm, '_state_lock', None)
        if lock is None:
            return
        if enabled and not self._payload_mode:
            with lock:
                if self._saved_kp is None:
                    self._saved_kp = arm._position_kp
                    self._saved_kd = arm._position_kd
                    self._saved_grav = arm._gravity_feedforward_ratio
                arm._position_kp = self._saved_kp * 2.0
                arm._position_kd = self._saved_kd * 1.5
                arm._gravity_feedforward_ratio = self._saved_grav * 1.5
            self._payload_mode = True
            logger.info("[%s] 负载模式 ON (kp=%.0f, grav=%.1f)",
                        self.name, arm._position_kp, arm._gravity_feedforward_ratio)
        elif not enabled and self._payload_mode:
            with lock:
                arm._position_kp = self._saved_kp
                arm._position_kd = self._saved_kd
                arm._gravity_feedforward_ratio = self._saved_grav
            self._payload_mode = False
            logger.info("[%s] 负载模式 OFF → 恢复正常参数", self.name)

    def process_pose(
        self,
        pose: Dict[str, Any],
        speed_factor: float,
        viewer_delta: Optional[List[float]] = None,
    ) -> bool:
        """Incremental delta → Unitree-style joint filtering + velocity clip."""
        if self.is_moving or self.is_estop or self._kin is None or self._target_pose is None:
            return False
        # Fine yaw mode: allow yaw-only tracking even without calibration or grip tracking
        if self._fine_yaw_mode:
            pass  # fall through to yaw tracking below
        elif pose is None or not self._calibrated:
            return False
        elif not self._tracking_engaged:
            return False

        # Fine yaw mode: skip position tracking, only update wrist yaw
        if not self._fine_yaw_mode:
            raw_hand = [pose["x"], pose["y"], pose["z"]]
            if self._hand_filtered is None:
                self._hand_filtered = list(raw_hand)
            else:
                alpha = max(0.05, min(0.8, self._input_alpha))
                for i in range(3):
                    self._hand_filtered[i] += alpha * (raw_hand[i] - self._hand_filtered[i])
            cur_hand = list(self._hand_filtered)

            # Skip first N frames after tracking engage
            if self._skip_frames > 0:
                self._skip_frames -= 1
                self._prev_hand_pos = list(cur_hand)
                self._last_raw_hand_pos = list(raw_hand)
                return True

            # Downsample: only process every Nth new Pico packet (~10Hz effective)
            self._sample_counter += 1
            if self._sample_counter < self._sample_interval:
                self._send_filtered()
                return True
            self._sample_counter = 0

            # Per-frame delta
            if self._last_raw_hand_pos is None:
                self._last_raw_hand_pos = list(raw_hand)
                self._prev_hand_pos = list(cur_hand)
                self._send_filtered()
                return True
            if self._prev_hand_pos is None:
                self._prev_hand_pos = list(cur_hand)
                self._send_filtered()
                return True

            hand_delta = [raw_hand[i] - self._last_raw_hand_pos[i] for i in range(3)]
            hand_delta = _reject_viewer_correlated_delta(hand_delta, viewer_delta)
            self._last_raw_hand_pos = list(raw_hand)
            self._prev_hand_pos = list(cur_hand)

            delta_norm = math.sqrt(sum(v * v for v in hand_delta))
            delta_deadband = min(max(self._dz_threshold, 0.0), 0.003)
            if delta_norm < delta_deadband:
                hand_delta = [0.0, 0.0, 0.0]
            elif delta_deadband > 0.0:
                scale_out = (delta_norm - delta_deadband) / max(delta_norm, 1e-9)
                hand_delta = [v * scale_out for v in hand_delta]

            robot_delta = _map_hand_delta_to_robot_delta(hand_delta)

            if self._sv_vel is None:
                self._sv_vel = [0.0, 0.0, 0.0]
            danger = min(1.0, max(0.0, (self._last_ik_err - 0.003) / 0.012))
            brake = 1.0 - danger * 0.75
            scale = self._pos_scale * brake
            raw_step = [d * scale for d in robot_delta]
            # Safety cap for a single Pico packet. This avoids large tracking
            # jumps without shrinking normal hand deltas by the 100Hz loop dt.
            max_step = max(0.08, self._max_lin_vel * 1.5)
            step_norm = math.sqrt(sum(v * v for v in raw_step))
            if step_norm <= 1e-9:
                self._sv_vel = [0.0, 0.0, 0.0]
            else:
                if step_norm > max_step:
                    step_scale = max_step / max(step_norm, 1e-9)
                    raw_step = [v * step_scale for v in raw_step]

                STEP_EMA = 0.8
                for i in range(3):
                    self._sv_vel[i] = STEP_EMA * raw_step[i] + (1 - STEP_EMA) * self._sv_vel[i]

                self._target_pose.x += self._sv_vel[0]
                self._target_pose.y += self._sv_vel[1]
                self._target_pose.z += self._sv_vel[2]

        # Full RPY rotation tracking: only active in fine-yaw mode (trigger held)
        if self._fine_yaw_mode:
            pose_q = (pose["qx"], pose["qy"], pose["qz"], pose["qw"])
            cur_roll, cur_pitch, cur_yaw = quat_to_rpy(pose_q)
            droll  = cur_roll  - self._ref_hand_rpy[0]
            dpitch = cur_pitch - self._ref_hand_rpy[1]
            dyaw   = cur_yaw   - self._ref_hand_rpy[2]
            DEADBAND_DEG = 0.5
            if abs(droll)  < math.radians(DEADBAND_DEG): droll  = 0.0
            if abs(dpitch) < math.radians(DEADBAND_DEG): dpitch = 0.0
            if abs(dyaw)   < math.radians(DEADBAND_DEG): dyaw   = 0.0
            # VR → robot rotation mapping (1:1):
            #   VR roll  (X) → robot rx
            #   VR pitch (Y) → robot rz
            #   VR yaw   (Z) → robot -ry
            self._target_pose.rx = self._ref_robot_rpy[0] + droll  * self._yaw_scale
            self._target_pose.ry = self._ref_robot_rpy[1] - dyaw   * self._yaw_scale
            self._target_pose.rz = self._ref_robot_rpy[2] + dpitch * self._yaw_scale

        # IK
        try:
            q_sol, ik_err = self._kin.ik_step(
                self._target_pose, self._ik_seed,
                damping=5e-3, max_step=self._max_ik_jump)
            self._last_ik_err = ik_err
            if q_sol is not None and self._accept_ik(q_sol):
                self._ik_raw = q_sol
                self._ik_seed = list(q_sol)
                self._consecutive_ik_fails = 0
            else:
                self._consecutive_ik_fails += 1
                if self._consecutive_ik_fails >= 50:
                    self._resync_ik()
        except Exception as e:
            logger.error("[%s] IK exception: %s", self.name, e)
            self._consecutive_ik_fails += 1

        self._send_filtered()
        return True

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
        if self._consecutive_rejects >= 50:
            self._resync_ik()
        return False

    def _resync_ik(self):
        samples = []
        for _ in range(5):
            q = self._arm.GetArmJointMsgs().to_list()[:6]
            samples.append(q)
            time.sleep(0.004)
        q_avg = [sum(s[i] for s in samples) / len(samples) for i in range(6)]
        self._ik_seed = list(q_avg)
        self._ik_raw = list(q_avg)
        self._ik_filter_pos = list(q_avg)
        self._hold_q = list(q_avg)
        self._joint_ma_window = []
        self._last_joint_cmd = list(q_avg)
        for i in range(6):
            self._ik_filter_vel[i] *= 0.2
        self._seed_just_init = True
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0
        self._resync_cooldown = 5
        self._last_sent_pose_tuple = None
        self._smooth_pose = None
        if self._kin is not None:
            fk = self._kin.forward_kinematics(q_avg)
            self._target_pose = fk
            self._ee_target_raw = [fk.x, fk.y, fk.z]
            self._ee_target_filtered = None
            self._prev_pose = None

    def _weighted_joint_target(self, q_raw: List[float]) -> List[float]:
        self._joint_ma_window.append(list(q_raw[:6]))
        if len(self._joint_ma_window) > len(self._joint_ma_weights):
            self._joint_ma_window.pop(0)

        usable = min(len(self._joint_ma_window), len(self._joint_ma_weights))
        weights = self._joint_ma_weights[:usable]
        norm = sum(weights)
        q_out = [0.0] * 6
        for k, weight in enumerate(weights):
            q_sample = self._joint_ma_window[-1 - k]
            for i in range(6):
                q_out[i] += (weight / norm) * q_sample[i]
        return q_out

    def _clip_joint_target_from_feedback(self, q_target: List[float]) -> List[float]:
        q_now = self._last_joint_cmd or self._hold_q or self._ik_filter_pos or q_target

        max_step = max(0.0005, self._joint_max_vel * max(self._dt, 1e-6))
        delta = [q_target[i] - q_now[i] for i in range(6)]
        max_abs_delta = max(abs(v) for v in delta)
        if max_abs_delta <= max_step:
            return list(q_target[:6])
        scale = max_abs_delta / max_step
        return [q_now[i] + delta[i] / scale for i in range(6)]

    def _send_filtered(self):
        """Second-order joint smoothing + velocity feedforward JointCtrl."""
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
            self._hold_q = list(self._ik_filter_pos)

        state = getattr(self._arm, "arm_state", None)
        if state is not None and state not in (ArmState.ENABLED, ArmState.RUNNING):
            return
        q_cmd = self._clip_joint_target_from_feedback(self._ik_filter_pos)
        if self._last_joint_cmd is None:
            vel_cmd = list(self._ik_filter_vel or [0.0] * 6)
        else:
            dt = max(self._dt, 1e-6)
            vel_cmd = [(q_cmd[i] - self._last_joint_cmd[i]) / dt for i in range(6)]
        max_ff = max(self._joint_max_vel, 0.05)
        vel_cmd = [max(-max_ff, min(max_ff, v)) for v in vel_cmd]
        self._arm.JointCtrl(*q_cmd, velocities=vel_cmd)
        self._last_joint_cmd = list(q_cmd)

    def brake_tracking_input(self, send_hold: bool = True):
        """Stop teleop input immediately and hold the latest commanded joints."""
        self._sv_vel = [0.0, 0.0, 0.0]
        self._stick_x_ema = 0.0
        self._yaw_active = False
        self._prev_hand_pos = None
        self._last_raw_hand_pos = None
        self._hand_filtered = None

        q_hold = self._last_joint_cmd or self._ik_filter_pos or self._hold_q
        if q_hold is None:
            try:
                q_hold = self._arm.GetArmJointMsgs().to_list()[:6]
            except Exception:
                q_hold = None
        if q_hold is None:
            return

        q_hold = list(q_hold[:6])
        self._ik_raw = list(q_hold)
        self._ik_seed = list(q_hold)
        self._ik_filter_pos = list(q_hold)
        self._ik_filter_vel = [0.0] * 6
        self._hold_q = list(q_hold)
        self._joint_ma_window = []
        self._last_sent_pose_tuple = None
        if self._kin is not None:
            try:
                self._target_pose = self._kin.forward_kinematics(q_hold)
            except Exception:
                pass
        if send_hold:
            self._send_filtered()

    def apply_base_yaw(self, angular_delta: float) -> bool:
        """Apply joystick yaw as a direct base-joint target increment."""
        if self.is_moving or self.is_estop:
            return False
        if self._ik_raw is not None:
            q_target = list(self._ik_raw[:6])
        elif self._ik_filter_pos is not None:
            q_target = list(self._ik_filter_pos[:6])
        else:
            try:
                q_target = self._arm.GetArmJointMsgs().to_list()[:6]
            except Exception:
                return False
        if len(q_target) < 6:
            return False
        q_target[0] += angular_delta
        self._ik_raw = q_target
        self._ik_seed = list(q_target)
        self._hold_q = list(q_target)
        if self._kin is not None:
            try:
                self._target_pose = self._kin.forward_kinematics(q_target)
                self._ref_robot_rpy = [
                    self._target_pose.rx,
                    self._target_pose.ry,
                    self._target_pose.rz,
                ]
            except Exception:
                pass
        self._send_filtered()
        return True

    def _send_joint_only(self):
        """Joint-space control: filter _ik_raw → send via JointCtrl, no MoveL/IK."""
        if self._ik_raw is None:
            return
        if self._ik_filter_pos is None:
            self._ik_filter_pos = list(self._ik_raw)
            self._ik_filter_vel = [0.0] * 6

        omega = self._filter_omega
        dt = self._dt
        a = omega * dt
        ea = math.exp(-a)
        for i in range(6):
            err = self._ik_raw[i] - self._ik_filter_pos[i]
            vel = self._ik_filter_vel[i]
            self._ik_filter_pos[i] = self._ik_raw[i] - ea * ((1.0 + a) * err - dt * vel)
            self._ik_filter_vel[i] = ea * (omega * omega * dt * err + (1.0 - a) * vel)

        self._hold_q = list(self._ik_filter_pos)
        self._arm.JointCtrl(*self._ik_filter_pos,
                            velocities=list(self._ik_filter_vel))

    def get_joint_degrees(self) -> List[str]:
        q = self._arm.GetArmJointMsgs().to_list()[:6]
        return [f"{v * 180 / math.pi:.1f}" for v in q]

    def get_can_fps(self) -> float:
        return self._arm.GetCanFps()

    def get_can_stats(self):
        return self._arm.GetCanTxStats()


# ================================================================
# Dual-Arm Pico Controller
# ================================================================


class _NoopGripperHold:
    active = False
    state = "idle"
    target_angle = 0.0

    def tick(self):
        return None

    def release(self):
        return None


class _NoopArm:
    def ZeroTorqueMode(self, _enabled: bool) -> bool:
        return True


class _NoopArmController:
    """Left-side placeholder used by single-arm right-hand mode."""

    def __init__(self, name: str = "左臂"):
        self.name = name
        self._arm = _NoopArm()
        self.is_moving = False
        self.is_estop = False
        self._gripper_hold = _NoopGripperHold()
        self._gripper_angle = 0.0
        self._payload_mode = False
        self._tracking_engaged = False
        self._grip_hold_start = None
        self._kin = None
        self._ik_raw = None
        self._target_pose = None
        self._stick_x_ema = 0.0
        self._yaw_active = False
        self._stick_conflict_warned = False

    @property
    def calibrated(self) -> bool:
        return True

    @property
    def grip_active(self) -> bool:
        return False

    @property
    def target_pose(self) -> Optional[ArmEndPose]:
        return None

    def init_position(self):
        return None

    def _send_filtered(self):
        return None

    def process_pose(self, _pose: Dict[str, Any], _speed_factor: float) -> bool:
        return False

    def move_to(self, _positions: List[float], _label: str):
        return None

    def emergency_stop(self):
        self.is_estop = True

    def recover_from_estop(self):
        self.is_estop = False

    def set_payload_mode(self, _enabled: bool):
        return None

    def _resync_ik(self):
        return None

    def get_joint_degrees(self) -> List[str]:
        return []

    def get_can_fps(self) -> float:
        return 0.0

    def get_can_stats(self):
        return (0, 0, 0.0)


def _export_robot_state(left: "_ArmController", right: "_ArmController", path: str) -> None:
    """Write current robot state to a JSON file for external data recording.

    Only writes the *right* arm state (the one controlling the primary arm).
    Uses atomic write: temp file + rename.
    """
    import json as _json, os as _os, tempfile as _tempfile
    from dataclasses import asdict as _asdict

    arm = right  # primary (right) arm for recording
    try:
        qpos = list(arm._arm.GetArmJointMsgs().to_list(include_gripper=True))
        qvel = list(arm._arm.GetArmJointVelocities().to_list(include_gripper=True))
        tau = list(arm._arm.GetArmJointEfforts().to_list(include_gripper=True))
        ee = _asdict(arm._arm.GetArmEndPoseMsgs())
        can = {
            "bus_state": arm._arm.GetCanBusState(),
            "fps": arm._arm.GetCanFps(),
            "can_name": arm._arm.GetCanName(),
        }
        # motor feedback
        motor_states = {}
        for mid, fb in arm._arm.GetMotorStates().items():
            motor_states[str(mid)] = _asdict(fb)
        robot_status = _asdict(arm._arm.GetArmStatus())
    except Exception:
        return  # silently skip if state read fails (e.g. during init)

    state = {
        "timestamp_ns": int(time.time_ns()),
        "qpos": qpos,
        "qvel": qvel,
        "tau": tau,
        "ee_pose": ee,
        "can": can,
        "motor_states": motor_states,
        "robot_status": robot_status,
        "is_estop": getattr(right, "is_estop", False),
        "episode_done": getattr(right, "is_estop", False),  # A long-press → estop → episode done
    }
    fd, tmp = _tempfile.mkstemp(dir=_os.path.dirname(path) or ".", suffix=".tmp")
    try:
        _os.write(fd, _json.dumps(state, ensure_ascii=False, separators=(",", ":")).encode())
        _os.fsync(fd)
    finally:
        _os.close(fd)
    _os.replace(tmp, path)


class PicoArmController:
    """双机械臂 Pico VR 遥操作控制器

    左手柄 → 左臂, 右手柄 → 右臂
    共享速度档位、零力矩模式、急停状态
    """

    def __init__(
        self,
        left_arm: ELA3Interface,
        right_arm: ELA3Interface,
        update_rate: float = 100.0,
        max_linear_velocity: float = 0.15,
        max_angular_velocity: float = 1.5,
        position_scale: float = 1.0,
        deadzone: float = 0.03,
        input_smoothing: float = 0.65,
        filter_omega: float = 24.0,
        max_ik_jump: float = 0.1,
        yaw_scale: float = 1.0,
        joint_max_vel: float = 0.2,
        grip_speed: float = 2.0,
        state_export_path: str | None = None,
        single_arm: bool = False,
    ):
        self._rate = update_rate
        self._single_arm = bool(single_arm)
        self._grip_speed = float(grip_speed)

        if self._single_arm:
            self._left = _NoopArmController("左臂")
        else:
            self._left = _ArmController(
                "左臂", left_arm, update_rate, max_linear_velocity, max_angular_velocity,
                position_scale, deadzone, input_smoothing, filter_omega, max_ik_jump,
                yaw_scale=yaw_scale,
                joint_max_vel=joint_max_vel,
            )
        self._right = _ArmController(
            "右臂", right_arm, update_rate, max_linear_velocity, max_angular_velocity,
            position_scale, deadzone, input_smoothing, filter_omega, max_ik_jump,
            yaw_scale=yaw_scale,
            joint_max_vel=joint_max_vel,
        )

        # Shared state
        self._running = False
        self._exit_requested = False
        self._zero_torque = False
        self._speed_idx = 2
        self._speed_factor = SPEED_LEVELS[self._speed_idx][1]
        self._a_hold_start: Optional[float] = None  # A-button long-press timer
        self._thumb_hold_start: Optional[float] = None  # 摇杆按下hold计时器(急停)
        self._zt_hold_start: Optional[float] = None     # 摇杆按下hold计时器(零力矩)

        # Button state for shared buttons
        self._prev_btn_left = [0] * 8
        self._prev_btn_right = [0] * 8
        self._buttons_synced = False

        # Pose tracking
        self._last_pose_seq = -1
        self._pose_stale_count = 0
        self._max_stale_ticks = 30
        self._duplicate_pose_count = 0
        self._max_duplicate_ticks = max(6, int(update_rate * 0.12))
        self._last_viewer_pos: Optional[List[float]] = None

        # State export for data recording
        self._state_export_path = state_export_path

        # Diagnostics
        self._diag_tick = 0

    @property
    def exit_requested(self) -> bool:
        return self._exit_requested

    # ---- Lifecycle ----

    def start(self):
        self._running = True
        self._initialize()
        self._print_banner()

        period = 1.0 / self._rate
        next_tick = time.monotonic()

        while self._running and not self._exit_requested:
            next_tick += period
            try:
                self._tick()
            except Exception as e:
                logger.error("控制循环异常: %s", e)
            if self._state_export_path:
                try:
                    _export_robot_state(self._left, self._right, self._state_export_path)
                except Exception:
                    pass  # never let export crash the control loop
            now = time.monotonic()
            sleep_s = next_tick - now
            if sleep_s < -period:
                next_tick = now + period
            elif sleep_s > 0:
                time.sleep(sleep_s)

    def stop(self):
        self._running = False

    def _initialize(self):
        if not self._single_arm:
            logger.info("初始化左臂...")
            self._left.init_position()
        logger.info("初始化右臂...")
        self._right.init_position()

    def _print_banner(self):
        print("\n" + "=" * 58)
        title = "EL-A3 Pico VR 单臂遥操作控制" if self._single_arm else "EL-A3 Pico VR 双机械臂遥操作控制"
        print(f"   {title}")
        print("=" * 58)
        if self._single_arm:
            print("  右手柄 → 单机械臂")
        else:
            print("  左手柄 → 左臂 (arm1)      右手柄 → 右臂 (arm2)")
        print()
        print("  数据源:    pico3_webxr_pose_receiver.py")
        print()
        print("  按钮映射 (每臂独立):")
        print("    右手 A / 左手 X   → 对应臂回零位")
        print("    右手 B / 左手 Y   → 对应臂回 Home")
        print("    侧边键(长按1秒)   → 开启跟踪")
        print("    侧边键(松开)      → 停止跟踪")
        print("    摇杆上推/下推     → 夹爪收紧/打开")
        print("    摇杆左推/右推     → 基座 yaw 旋转")
        print()
        print("  食指扳机(扣住0.3秒)  → 精细Yaw模式(位置冻结, 只跟手腕扭转)")
        print()
        print("  共享控制:")
        print("    左手摇杆按下       → 零力矩模式")
        print("    右手摇杆按下       → 急停")
        print("=" * 58)
        self._log_speed()
        print("  等待 Pico 手柄数据... 请确保 pico3_webxr_pose_receiver.py 已在运行")

    def _log_speed(self):
        name, factor = SPEED_LEVELS[self._speed_idx]
        ref_arm = self._right if self._single_arm else self._left
        lin_mm = ref_arm._max_lin_vel * factor * 1000
        ang = ref_arm._max_ang_vel * factor
        print(f"  速度档位:  {self._speed_idx + 1}/5 [{name}] "
              f"({lin_mm:.0f}mm/s, {ang:.2f}rad/s)")

    @staticmethod
    def _arm_needs_stream(arm) -> bool:
        return bool(
            getattr(arm, "_tracking_engaged", False)
            or getattr(arm, "_fine_yaw_mode", False)
            or getattr(arm, "_yaw_active", False)
            or getattr(arm, "_resync_cooldown", 0) > 0
        )

    def _send_active_streams(self):
        for arm in (self._left, self._right):
            if self._arm_needs_stream(arm):
                arm._send_filtered()

    def _brake_active_streams(self):
        for arm in (self._left, self._right):
            if self._arm_needs_stream(arm):
                arm.brake_tracking_input(send_hold=True)

    # ---- Button helpers ----

    @staticmethod
    def _btn_val(buttons, idx):
        if idx < len(buttons) and buttons[idx] is not None:
            return 1 if buttons[idx].get("pressed") else 0
        return 0

    @staticmethod
    def _btn_edge(current: int, prev: int) -> bool:
        return current == 1 and prev == 0

    # ---- Main tick ----

    def _tick(self):
        packet = _read_pico_pose()
        # Always tick gripper holds — they must run even without Pico data
        try:
            self._left._gripper_hold.tick()
        except Exception:
            pass
        try:
            self._right._gripper_hold.tick()
        except Exception:
            pass

        if packet is None:
            self._pose_stale_count += 1
            self._duplicate_pose_count = 0
            if self._pose_stale_count == 1:
                logger.warning("未收到 Pico 数据，等待中...")
            self._brake_active_streams()
            self._periodic_status()
            return

        self._pose_stale_count = 0
        seq = packet.get("seq", -1)
        moving = self._left.is_moving or self._right.is_moving
        if seq == self._last_pose_seq:
            self._duplicate_pose_count += 1
            if not moving:
                if self._duplicate_pose_count >= self._max_duplicate_ticks:
                    self._brake_active_streams()
                else:
                    self._send_active_streams()
            self._periodic_status()
            return
        self._duplicate_pose_count = 0
        self._last_pose_seq = seq

        sources = packet.get("inputSources") or []
        left_src, right_src = None, None
        for s in sources:
            hand = s.get("handedness", "")
            if hand == "left":
                left_src = s
            elif hand == "right":
                right_src = s

        left_pose = _extract_grip_pose(left_src) if left_src else None
        right_pose = _extract_grip_pose(right_src) if right_src else None
        track_pose = right_pose  # right hand controls the arm
        left_btns = _extract_buttons(left_src) if left_src else []
        right_btns = _extract_buttons(right_src) if right_src else []
        left_axes = _extract_axes(left_src) if left_src else []
        right_axes = _extract_axes(right_src) if right_src else []
        viewer_delta = None
        viewer = packet.get("viewer") or {}
        vpos = viewer.get("position") if isinstance(viewer, dict) else None
        if isinstance(vpos, dict):
            viewer_pos = [
                float(vpos.get("x", 0.0)),
                float(vpos.get("y", 0.0)),
                float(vpos.get("z", 0.0)),
            ]
            if self._last_viewer_pos is not None:
                viewer_delta = [viewer_pos[i] - self._last_viewer_pos[i] for i in range(3)]
            else:
                viewer_delta = [0.0, 0.0, 0.0]
            self._last_viewer_pos = viewer_pos

        if self._single_arm:
            left_pose = None
            left_btns = []
            left_axes = []

        # ---- Calibration ----
        l_need_cal = not self._left.calibrated and left_pose is not None and not self._left.is_moving
        r_need_cal = not self._right.calibrated and right_pose is not None and not self._right.is_moving
        if l_need_cal:
            self._left.calibrate(left_pose)
        if r_need_cal:
            self._right.calibrate(right_pose)
        if l_need_cal or r_need_cal:
            return  # skip this frame after calibration

        def _save_button_states():
            def _save(btns, prev):
                for i in range(min(len(btns), len(prev))):
                    b = btns[i]
                    if b is None:
                        prev[i] = 0
                    elif i <= 1:
                        prev[i] = 1 if float(b.get("value", 0)) > 0.5 else 0
                    else:
                        prev[i] = 1 if b.get("pressed") else 0
            if left_btns:
                _save(left_btns, self._prev_btn_left)
            if right_btns:
                _save(right_btns, self._prev_btn_right)

        if not self._buttons_synced:
            _save_button_states()
            self._buttons_synced = True
            self._periodic_status()
            return

        # ---- Button handling ----
        estop_active = self._left.is_estop or self._right.is_estop

        # A/X: short press=zero, long press 1s=disable motors
        l_a = self._btn_val(left_btns, BTN_XA)
        r_a = self._btn_val(right_btns, BTN_XA)
        a_pressed = (l_a == 1) or (r_a == 1)
        a_was = (self._prev_btn_left[BTN_XA] == 1) or (self._prev_btn_right[BTN_XA] == 1)
        now_a = time.monotonic()
        if not hasattr(self, '_a_hold_start') or self._a_hold_start is None:
            if a_pressed and not a_was:
                self._a_hold_start = now_a
        if a_pressed and self._a_hold_start is not None:
            hold_dur = now_a - self._a_hold_start
            if hold_dur >= 1.0:
                self._a_hold_start = None
                logger.info("A 长按检测: hold_dur=%.2fs is_estop L=%s R=%s",
                            hold_dur, self._left.is_estop, self._right.is_estop)
                if self._left.is_estop or self._right.is_estop:
                    logger.info("A long press: re-enabling motors...")
                    for arm in (self._left, self._right):
                        if arm.is_estop:
                            arm.recover_from_estop()
                    _save_button_states()
                    self._periodic_status()
                    return
                else:
                    logger.info("A long press: disabling all motors...")
                    for arm in (self._left, self._right):
                        arm.emergency_stop()
                    _save_button_states()
                    self._periodic_status()
                    return
        elif not a_pressed and self._a_hold_start is not None:
            hold_dur = now_a - self._a_hold_start
            logger.info("A 短按释放: hold_dur=%.2fs", hold_dur)
            if 0.05 < hold_dur < 1.0 and not estop_active:
                if self._prev_btn_left[BTN_XA] == 1:
                    self._left.move_to(ZERO_POSITIONS, "零位")
                if self._prev_btn_right[BTN_XA] == 1:
                    self._right.move_to(ZERO_POSITIONS, "零位")
            elif 0.05 < hold_dur < 1.0 and estop_active:
                logger.info("A 短按在急停态被忽略")
            self._a_hold_start = None

        # In disabled/estop state, reject every command except the A/X
        # long-press recovery handled above.
        if self._left.is_estop or self._right.is_estop:
            self._thumb_hold_start = None
            self._zt_hold_start = None
            self._left._grip_hold_start = None
            self._right._grip_hold_start = None
            self._left._tracking_engaged = False
            self._right._tracking_engaged = False
            _save_button_states()
            self._periodic_status()
            return

        # Left Y = left arm home
        l_b = self._btn_val(left_btns, BTN_YB)
        if self._btn_edge(l_b, self._prev_btn_left[BTN_YB]):
            self._left.move_to(HOME_POSITIONS, "Home")

        # Right B = right arm home
        r_b = self._btn_val(right_btns, BTN_YB)
        if self._btn_edge(r_b, self._prev_btn_right[BTN_YB]):
            self._right.move_to(HOME_POSITIONS, "Home")

        # Emergency stop: right thumbstick hold 0.5s (防止推摇杆时误触)
        r_thumb = self._btn_val(right_btns, BTN_THUMBSTICK)
        if r_thumb == 1:
            if self._thumb_hold_start is None:
                self._thumb_hold_start = time.monotonic()
            elif time.monotonic() - self._thumb_hold_start >= 0.5:
                self._emergency_stop()
                self._thumb_hold_start = None
        else:
            self._thumb_hold_start = None

        # Zero torque toggle: left thumbstick hold 0.3s
        l_thumb = self._btn_val(left_btns, BTN_THUMBSTICK)
        if l_thumb == 1:
            if self._zt_hold_start is None:
                self._zt_hold_start = time.monotonic()
            elif time.monotonic() - self._zt_hold_start >= 0.3:
                self._toggle_zero_torque()
                self._zt_hold_start = None
        else:
            self._zt_hold_start = None

        # ---- 摇杆控制夹爪: 上推收紧, 下推打开 ----
        # Active stick movement updates the gripper target; when the stick is
        # released, GripperHold re-sends the position command periodically so
        # the motor never times out, and monitors feedback for stall protection.
        STICK_DEADZONE = 0.2
        GRIP_EFFORT = 0.65      # Nm torque limit while gripping
        GRIP_RELEASE_THRESH = 0.02  # rad — below this, gripper is "fully open"
        for arm, axes in [
            (self._left, left_axes),
            (self._right, right_axes),
        ]:
            if self._zero_torque or arm.is_estop:
                continue
            stick_x_raw, stick_y = _thumbstick_axes(axes)

            stick_active = abs(stick_y) > STICK_DEADZONE and abs(stick_y) > abs(stick_x_raw)

            if stick_active and not arm._gripper_available:
                now = time.monotonic()
                if now - arm._last_gripper_missing_log > 2.0:
                    arm._last_gripper_missing_log = now
                    logger.warning("[%s] 电机7无反馈，仍发送夹爪命令；若实物不动请检查 L7 是否在线。", arm.name)

            if stick_active:
                # --- active stick control: update target angle ---
                if getattr(arm, '_axes_detected', False) is False:
                    arm._axes_detected = True
                    ms = arm._arm.GetMotorStates()
                    fb7 = ms.get(7)
                    if fb7 and fb7.is_valid:
                        arm._gripper_angle = fb7.position
                        arm._gripper_last_sent = fb7.position
                        logger.info("[%s] 夹爪初始位置: %.3f rad", arm.name, fb7.position)
                    logger.info("[%s] 摇杆轴检测: axes=%s, stick_y=%.3f", arm.name, axes, stick_y)
                speed = (-stick_y) * self._grip_speed
                force_send = False
                if arm._gripper_hold.active and arm._gripper_hold.state == GripperHold.GRASPED:
                    arm._gripper_angle = arm._gripper_hold.target_angle
                    arm._gripper_last_sent = arm._gripper_angle
                    force_send = True
                arm._gripper_angle += speed * arm._dt
                arm._gripper_angle = max(0.0, arm._gripper_angle)
                if force_send or abs(arm._gripper_angle - arm._gripper_last_sent) > 0.01:
                    logger.info("[%s] 夹爪: stick_y=%.3f speed=%.2f → angle=%.3f",
                                arm.name, stick_y, speed, arm._gripper_angle)
                    arm._gripper_last_sent = arm._gripper_angle
                    # Start / update continuous hold with stall protection
                    try:
                        arm._gripper_hold.grip(arm._gripper_angle, effort=GRIP_EFFORT)
                    except Exception:
                        pass

            # --- hold maintenance ---
            # If the gripper is partially closed but the stick is idle,
            # keep the hold alive.  Release only when fully open.
            if not stick_active and arm._gripper_hold.active:
                if arm._gripper_angle < GRIP_RELEASE_THRESH:
                    arm._gripper_hold.release()
                    arm._gripper_angle = 0.0
                    arm._gripper_last_sent = -1.0
                    arm._arm.GripperPositionCtrl(gripper_angle=0.0)
                    arm._arm.GripperCtrl(gripper_angle=0.0, gripper_effort=1.5)
                    logger.info("[%s] 夹爪已完全打开，停止持续保持", arm.name)

            # Tick the hold controller every cycle (no-op when inactive)
            try:
                arm._gripper_hold.tick()
            except Exception:
                pass

        # ---- Payload mode: auto-boost kp/gravity when gripping, restore on release ----
        try:
            GRIP_THRESHOLD = 0.2  # rad — above this the gripper is considered "holding something"
            for arm in (self._left, self._right):
                gripping = arm._gripper_angle > GRIP_THRESHOLD
                if gripping != arm._payload_mode:
                    arm.set_payload_mode(gripping)
        except Exception:
            pass  # never let payload switching break the main loop

        # ---- Trigger (btn[0]): hold 0.3s → fine yaw mode, release → exit ----
        TRIGGER_HOLD_SEC = 0.3
        for arm, btns in [(self._left, left_btns), (self._right, right_btns)]:
            if not btns:
                continue
            trigger_val = float(btns[0].get("value", 0)) if btns[0] is not None else 0.0
            tp = trigger_val > 0.5
            _prev = getattr(arm, '_last_tp_val', -1.0)
            if abs(trigger_val - _prev) > 0.05:
                arm._last_tp_val = trigger_val
                logger.info("[%s] TRIGGER val=%.3f tp=%s fine_yaw=%s",
                            arm.name, trigger_val, tp, arm._fine_yaw_mode)
            if tp and arm._trigger_hold_start is None:
                arm._trigger_hold_start = time.monotonic()
            elif tp and arm._trigger_hold_start is not None:
                if not arm._fine_yaw_mode and (time.monotonic() - arm._trigger_hold_start) >= TRIGGER_HOLD_SEC:
                    arm._fine_yaw_mode = True
                    # Init hand RPY reference from current hand orientation
                    arm_pose = left_pose if arm is self._left else right_pose
                    if arm_pose is not None:
                        arm._ref_hand_rpy = list(quat_to_rpy(
                            (arm_pose["qx"], arm_pose["qy"],
                             arm_pose["qz"], arm_pose["qw"])))
                    # Snapshot FK for orientation only (rx/ry/rz = actual arm state),
                    # keep x/y/z from _target_pose (no position jump).
                    if arm._kin is not None and arm._target_pose is not None:
                        cur_q = arm._arm.GetArmJointMsgs().to_list()[:6]
                        fk = arm._kin.forward_kinematics(cur_q)
                        if fk is not None:
                            arm._ref_robot_rpy = [fk.rx, fk.ry, fk.rz]
                            # Update target_pose orientation to match actual arm,
                            # but preserve tracked position
                            arm._target_pose.rx = fk.rx
                            arm._target_pose.ry = fk.ry
                            arm._target_pose.rz = fk.rz
                    logger.info("[%s] 精细Yaw ON (位置冻结, RPY跟踪)", arm.name)
            elif not tp:
                arm._trigger_hold_start = None
                if arm._fine_yaw_mode:
                    arm._fine_yaw_mode = False
                    logger.info("[%s] 精细Yaw OFF", arm.name)

        # ---- Grip (btn[0]): hold 1s → tracking, release → stop ----
        # Pico grip also analog, use value > 0.5
        GRIP_HOLD_SEC = 1.0
        for arm, btns, prev_btns in [
            (self._left, left_btns, self._prev_btn_left),
            (self._right, right_btns, self._prev_btn_right),
        ]:
            grip_val = 0.0
            if len(btns) > BTN_GRIP and btns[BTN_GRIP] is not None:
                grip_val = float(btns[BTN_GRIP].get("value", 0.0))
            gp = grip_val > 0.5
            gw = prev_btns[BTN_GRIP] == 1
            if gp and not gw:
                arm._grip_hold_start = time.monotonic()
                logger.info("[%s] Grip pressed (hold 1s for tracking)", arm.name)
            elif gp and gw and arm._grip_hold_start is not None:
                if not arm._tracking_engaged and (time.monotonic() - arm._grip_hold_start) >= GRIP_HOLD_SEC:
                    arm._tracking_engaged = True
                    arm._grip_hold_start = None
                    # Init from CURRENT actual positions (Unitree-style)
                    cur_q = arm._arm.GetArmJointMsgs().to_list()[:6]
                    fk = arm._kin.forward_kinematics(cur_q) if arm._kin else None
                    if fk is not None:
                        arm._target_pose = fk
                        arm._ref_robot_rpy = [fk.rx, fk.ry, fk.rz]
                    arm._last_sent_pose_tuple = None
                    arm._smooth_pose = None
                    arm._ik_raw = list(cur_q)
                    arm._ik_seed = list(cur_q)
                    arm._ik_filter_pos = list(cur_q)
                    arm._ik_filter_vel = [0.0] * 6
                    arm._hold_q = list(cur_q)
                    arm._last_joint_cmd = list(cur_q)
                    arm._sv_vel = [0.0, 0.0, 0.0]
                    arm._joint_ma_window = []
                    arm._consecutive_rejects = 0
                    arm._consecutive_ik_fails = 0
                    arm._resync_cooldown = 0
                    if hasattr(arm._arm, "cancel_motion"):
                        try:
                            arm._arm.cancel_motion()
                        except Exception:
                            pass
                    arm_pose = left_pose if arm is self._left else right_pose
                    if arm_pose is not None:
                        hp = [arm_pose["x"], arm_pose["y"], arm_pose["z"]]
                        arm._prev_hand_pos = list(hp)
                        arm._last_raw_hand_pos = list(hp)
                        arm._hand_filtered = list(hp)
                        arm._ref_hand_rpy = list(quat_to_rpy(
                            (arm_pose["qx"], arm_pose["qy"],
                             arm_pose["qz"], arm_pose["qw"])))
                    # Skip first frames to absorb Pico/WebXR pose settling on engagement.
                    arm._skip_frames = 8
                    logger.info("[%s] Tracking ON", arm.name)
            elif not gp and gw:
                arm._grip_hold_start = None
                if arm._tracking_engaged:
                    arm._tracking_engaged = False
                    if arm._ik_filter_pos is not None:
                        arm._ik_raw = list(arm._ik_filter_pos)
                        arm._ik_filter_vel = [0.0] * 6
                    logger.info("[%s] Tracking OFF", arm.name)

        _save_button_states()

        # ---- Pose processing (skip if zero-torque or estop) ----
        if self._zero_torque:
            self._send_active_streams()
            self._periodic_status()
            return
        if self._left.is_moving or self._right.is_moving:
            self._periodic_status()
            return

        if self._left.is_estop or self._right.is_estop:
            self._periodic_status()
            return

        # Per-arm control: stick yaw (velocity-form) takes priority over hand tracking
        STICK_YAW_EMA = 0.3
        sf = self._speed_factor
        for arm, pose, axes in [(self._left, left_pose, left_axes),
                                 (self._right, right_pose, right_axes)]:
            if arm._ik_raw is None or arm._kin is None:
                continue
            if pose is None and (arm._tracking_engaged or arm._fine_yaw_mode):
                arm.brake_tracking_input(send_hold=True)
                continue
            stick_x_raw, _stick_y = _thumbstick_axes(axes)
            # XBOX-style EMA: accelerated decay on release to prevent drag
            if abs(stick_x_raw) < 0.1:
                decay = min(STICK_YAW_EMA * 3.0, 1.0)
                arm._stick_x_ema = (1 - decay) * arm._stick_x_ema
            else:
                arm._stick_x_ema = STICK_YAW_EMA * stick_x_raw + (1 - STICK_YAW_EMA) * arm._stick_x_ema
            yaw_on = abs(arm._stick_x_ema) > 0.22
            yaw_off = abs(arm._stick_x_ema) < 0.10
            if yaw_on and arm._target_pose is not None:
                # Protection: stick takes priority, warn if hand tracking also active
                if arm._tracking_engaged:
                    if not getattr(arm, '_stick_conflict_warned', False):
                        arm._stick_conflict_warned = True
                        logger.warning("[%s] 摇杆Yaw优先，手部追踪暂停（松开摇杆恢复）", arm.name)
                arm._yaw_active = True
                ang_vel = -arm._stick_x_ema * arm._yaw_stick_scale * sf  # rad/s
                angle = ang_vel * arm._dt
                arm.apply_base_yaw(angle)
            else:
                # Reset conflict warning when stick released
                arm._stick_conflict_warned = False
                if yaw_off:
                    arm._yaw_active = False
                # No stick → normal hand tracking
                moved = arm.process_pose(pose, sf, viewer_delta=viewer_delta)
                if not moved and self._arm_needs_stream(arm):
                    arm._send_filtered()

        self._periodic_status()

    # ---- Shared actions ----

    def _toggle_zero_torque(self):
        new_state = not self._zero_torque
        logger.info("%s 零力矩模式...", "开启" if new_state else "关闭")
        ok_left = self._left._arm.ZeroTorqueMode(new_state)
        ok_right = self._right._arm.ZeroTorqueMode(new_state)
        if ok_left and ok_right:
            self._zero_torque = new_state
            if new_state:
                target = "机械臂" if self._single_arm else "双机械臂"
                print(f">>> 零力矩模式已开启: 可手动拖动{target} <<<")
            else:
                self._left._resync_ik()
                self._right._resync_ik()
                print(">>> 零力矩模式已关闭: 恢复 Pico 控制 <<<")
        else:
            logger.error("零力矩模式切换失败")

    def _emergency_stop(self):
        self._left.emergency_stop()
        self._right.emergency_stop()
        scope = "单臂" if self._single_arm else "双机械臂"
        print(f"\n!!! 急停已执行 ({scope}) — 按 Home 或零位恢复 !!!")

    # ---- Diagnostics ----

    def _periodic_status(self):
        self._diag_tick += 1
        if self._diag_tick < int(self._rate * 5):
            return
        self._diag_tick = 0

        r_deg = self._right.get_joint_degrees()

        if self._zero_torque:
            mode = "零力矩"
        elif self._left.is_estop or self._right.is_estop:
            mode = "急停"
        elif self._left.grip_active or self._right.grip_active:
            mode = "夹爪"
        elif not self._left.calibrated or not self._right.calibrated:
            mode = "未标定"
        else:
            mode = "正常"

        if self._single_arm:
            print(f"  [{mode}] 右臂(deg): [{', '.join(r_deg)}]")
            r_fps = self._right.get_can_fps()
            if r_fps > 0:
                print(f"         CAN: {r_fps:.0f}fps")
            else:
                print("         [SIM mode]")
            rp = self._right.target_pose
            if rp:
                print(f"  右臂末端: ({rp.x:.3f}, {rp.y:.3f}, {rp.z:.3f}) m")
            return

        l_deg = self._left.get_joint_degrees()
        print(f"  [{mode}] 左臂(deg): [{', '.join(l_deg)}]")
        print(f"         右臂(deg): [{', '.join(r_deg)}]")
        l_fps = self._left.get_can_fps()
        r_fps = self._right.get_can_fps()
        if l_fps > 0 or r_fps > 0:
            print(f"         CAN L: {l_fps:.0f}fps  CAN R: {r_fps:.0f}fps")
        else:
            print(f"         [SIM mode]")

        lp = self._left.target_pose
        rp = self._right.target_pose
        if lp:
            print(f"  左臂末端: ({lp.x:.3f}, {lp.y:.3f}, {lp.z:.3f}) m")
        if rp:
            print(f"  右臂末端: ({rp.x:.3f}, {rp.y:.3f}, {rp.z:.3f}) m")


# ================================================================
# Main
# ================================================================


def _check_can(args, can_name: str):
    try:
        with open(f"/sys/class/net/{can_name}/tx_queue_len") as f:
            qlen = int(f.read().strip())
        if qlen < 64:
            print(f"[WARNING] CAN TX 队列过小 ({can_name} qlen={qlen})，建议: "
                  f"sudo ip link set {can_name} txqueuelen 128")
    except Exception:
        pass


def _hold_current_position_after_enable(arm, label: str) -> bool:
    """Immediately command the current joint pose after EnableArm."""
    try:
        joint_msg = arm.GetArmJointMsgs()
        if getattr(joint_msg, "timestamp", 1.0) <= 0.0:
            print(f"[WARNING] {label} 使能后没有有效关节反馈，暂时无法立即保持")
            return False
        q_now = joint_msg.to_list()[:6]
    except Exception as exc:
        print(f"[WARNING] {label} 使能后读取当前关节失败，暂时无法立即保持: {exc}")
        return False
    if len(q_now) < 6:
        print(f"[WARNING] {label} 使能后关节反馈不足，暂时无法立即保持: {q_now}")
        return False
    torque_ff = None
    if hasattr(arm, "ComputeGravityTorques"):
        try:
            torque_ff = arm.ComputeGravityTorques(q_now)
        except Exception as exc:
            print(f"[WARNING] {label} 使能后重力前馈计算失败，仅使用 PD 保持: {exc}")
    ok = arm.JointCtrl(*q_now, velocities=[0.0] * 6, torque_ff=torque_ff)
    if not ok:
        print(f"[WARNING] {label} 使能后当前位置保持 JointCtrl 失败")
        return False
    return True


def _warn_missing_wrist_or_gripper(arm, can_name: str, label: str) -> None:
    states = arm.GetMotorStates()
    missing = [mid for mid in (6, 7) if mid not in states]
    if not missing:
        return
    print(
        f"[WARNING] {label} ({can_name}) 未收到电机 {missing} 反馈；"
        "如果 7 号电机不在线，夹爪摇杆会只改变目标角，实物不会动作。"
    )


def main():
    parser = argparse.ArgumentParser(
        description="EL-A3 Pico VR 双机械臂遥操作控制",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 仿真模式（无需硬件）
  %(prog)s --sim

  # 双机械臂
  %(prog)s --can-left can0 --can-right can1

  # 单机械臂
  %(prog)s --can can0
""",
    )
    parser.add_argument("--sim", action="store_true",
                        help="仿真模式（无需 CAN 硬件，纯 FK/IK 模拟）")
    parser.add_argument("--can", default=None,
                        help="单臂模式 CAN 接口名 (如: can0)")
    parser.add_argument("--can-left", default="can0",
                        help="左臂 CAN 接口名 (默认: can0)")
    parser.add_argument("--can-right", default="can1",
                        help="右臂 CAN 接口名 (默认: can1)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="输入处理频率 Hz (默认: 100)")
    parser.add_argument("--max-lin-vel", type=float, default=0.15,
                        help="最大线速度 m/s (默认: 0.15)")
    parser.add_argument("--max-ang-vel", type=float, default=1.5,
                        help="最大角速度 rad/s (默认: 1.5)")
    parser.add_argument("--pos-scale", type=float, default=1.2,
                        help="Pico 手柄位移到机械臂末端位移的比例 (默认: 1.2)")
    parser.add_argument("--input-smoothing", type=float, default=0.65,
                        help="Pico 输入低通系数，越大越跟手但噪声更多 (默认: 0.65)")
    parser.add_argument("--filter-omega", type=float, default=24.0,
                        help="关节二阶滤波响应频率，越大越跟手 (默认: 24)")
    parser.add_argument("--max-ik-jump", type=float, default=0.2,
                        help="IK 单步最大关节跳变保护 rad (默认: 0.2)")
    parser.add_argument("--kp", type=float, default=80.0,
                        help="位置增益 Kp (默认: 80，负载大时提到 120~150)")
    parser.add_argument("--kd", type=float, default=4.0,
                        help="速度增益 Kd (默认: 4)")
    parser.add_argument("--gravity-feedforward-ratio", type=float, default=1.0,
                        help="重力前馈比例 (默认: 1.0，负载重时设 1.2~1.5 补偿额外重量)")
    parser.add_argument("--deadzone", type=float, default=0.02,
                        help="位移死区 m (默认: 0.02)")
    parser.add_argument("--debug", action="store_true",
                        help="调试模式")
    parser.add_argument("--state-export", default=None,
                        help="Export robot state to JSON file for data recording (e.g. /tmp/robot_latest_state.json)")
    parser.add_argument("--joint-max-vel", type=float, default=1.2,
                        help="JointCtrl 每关节最大追踪速度 rad/s (默认: 1.2，保守可试 0.6)")
    parser.add_argument("--grip-speed", type=float, default=2.0,
                        help="夹爪摇杆开合速度 rad/s (默认: 2.0)")
    parser.add_argument("--yaw-scale", type=float, default=1.0,
                        help="精细Yaw手柄角度到末端姿态角度比例 (默认: 1.0，想更细可设 0.5)")
    args = parser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=log_level, format="[%(name)s][%(levelname)s] %(message)s")
    logging.getLogger("el_a3_sdk").propagate = False

    sim_mode = args.sim

    if sim_mode:
        left_arm = SimArm("左臂")
        right_arm = SimArm("右臂")
        left_arm.ConnectPort()
        right_arm.ConnectPort()
        left_arm.EnableArm()
        right_arm.EnableArm()
        left_arm.start_control_loop()
        right_arm.start_control_loop()
        print("\n" + "=" * 58)
        print("   EL-A3 Pico VR 遥操作 -- 仿真模式")
        print("=" * 58)
        print("  无 CAN 硬件，纯 FK/IK 数值模拟")
        print("=" * 58)
    else:
        single_arm = args.can is not None
        can_left = args.can if single_arm else args.can_left
        can_right = args.can if single_arm else args.can_right

        def _connect(can_name, label):
            arm = ELA3Interface(can_name=can_name, default_kp=args.kp, default_kd=args.kd,
                                logger_level=LogLevel.INFO,
                                gravity_feedforward_ratio=args.gravity_feedforward_ratio)
            if not arm.ConnectPort():
                print(f"\nCAN {can_name} ({label}) connect failed")
                return None
            _check_can(args, can_name)
            bus = arm.GetCanBusState()
            if bus not in ("ERROR-ACTIVE", "UNKNOWN"):
                print(f"[WARNING] CAN bus state ({can_name}): {bus}")
            if not arm.EnableArm():
                print(f"\nEnableArm failed on {can_name} ({label}); check CAN power/state before teleop")
                arm.DisconnectPort()
                return None
            time.sleep(0.5)
            _warn_missing_wrist_or_gripper(arm, can_name, label)
            arm.start_control_loop(rate_hz=200.0)
            return arm

        left_arm = _connect(can_left, "left arm")
        if left_arm is None:
            return 1

        if single_arm:
            print(f"\nsingle-arm: right hand -> {can_left}")
            right_arm = left_arm
        else:
            right_arm = _connect(can_right, "right arm")
            if right_arm is None:
                left_arm.DisconnectPort()
                return 1
            print(f"\ndual-arm: left hand -> {can_left}  right hand -> {can_right}")
    # ---- Check Pico data ----
    pose = _read_pico_pose()
    if pose is None:
        print("\n[WARNING] no Pico data at /tmp/pico_latest_pose.json")
        print("Start pico3_webxr_pose_receiver.py in another terminal\n")
    else:
        sources = pose.get("inputSources") or []
        print(f"Pico data: seq={pose.get('seq', '?')}, sources={len(sources)}")

    # ---- Run ----
    shutdown = threading.Event()

    def on_signal(_sig, _frame):
        shutdown.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    controller = PicoArmController(
        left_arm=left_arm,
        right_arm=right_arm,
        update_rate=args.rate,
        max_linear_velocity=args.max_lin_vel,
        max_angular_velocity=args.max_ang_vel,
        position_scale=args.pos_scale,
        deadzone=args.deadzone,
        input_smoothing=args.input_smoothing,
        filter_omega=args.filter_omega,
        max_ik_jump=args.max_ik_jump,
        yaw_scale=args.yaw_scale,
        joint_max_vel=args.joint_max_vel,
        grip_speed=args.grip_speed,
        state_export_path=args.state_export,
        single_arm=single_arm,
    )

    ctrl_thread = threading.Thread(target=controller.start, daemon=True)
    ctrl_thread.start()

    try:
        while not shutdown.is_set() and not controller.exit_requested:
            shutdown.wait(timeout=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        print("\ncleaning up...")
        controller.stop()
        if not sim_mode and not single_arm:
            for arm in [left_arm, right_arm]:
                if arm is not None:
                    try:
                        arm.ZeroTorqueMode(False)
                    except Exception:
                        pass
                    arm.stop_control_loop()
                    arm.DisableArm()
                    arm.DisconnectPort()
        elif not sim_mode:
            left_arm.stop_control_loop()
            left_arm.DisableArm()
            left_arm.DisconnectPort()
        print("exited")

    return 0


if __name__ == "__main__":
    exit(main())
