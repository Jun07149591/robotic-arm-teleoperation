#!/usr/bin/env python3
"""
EL-A3 手柄控制程序（纯 SDK，无 ROS 依赖）

通过 Linux joystick API 直接读取手柄输入，结合 SDK 的 200Hz 后台控制循环
和 Pinocchio 完整 IK 实现末端坐标空间实时遥操作。

手柄输入映射为末端位姿增量，累加到目标末端位姿后通过 IK 求解关节角。

控制映射:
  左摇杆 Y/X     → 末端 X/Y 平移
  LT / RT         → 末端 Z 下/上
  右摇杆 X/Y     → Yaw / Roll 旋转
  LB / RB         → Pitch 旋转
  A               → 切换速度档位（5 档）
  B               → 回 Home 位置
  X               → 回零位
  Y               → 切换零力矩模式（可手动拖动）
  D-pad 上/下     → 夹爪开/合
  Back            → 急停
  Start           → 退出程序

使用前确保:
  1. CAN 接口已激活: sudo ip link set can0 up type can bitrate 1000000
  2. 机械臂已上电
  3. 手柄已连接 (USB 或蓝牙)
  4. 安装 pinocchio: pip install pin (笛卡尔控制需要)

用法:
  python3 xbox_control.py                              # 自动识别 profile
  python3 xbox_control.py --can can1                  # 指定 CAN 接口
  python3 xbox_control.py --js /dev/input/js1         # 指定手柄设备
  python3 xbox_control.py --profile zikway_3537_1041 # 强制指定映射
  python3 xbox_control.py --list-profiles             # 查看内置映射
  python3 xbox_control.py --dump-input                # 仅打印输入调试
  python3 xbox_control.py --debug                     # 调试模式
"""

import os
import time
import threading
import math
import signal
import argparse
import logging
from typing import List, Optional

from el_a3_sdk import ELA3Interface, ArmEndPose, LogLevel
from el_a3_sdk.joystick import LinuxJoystick
from el_a3_sdk.controller_profiles import PROFILES, ControllerProfile, detect_controller

logger = logging.getLogger("xbox_control")

# Speed levels: (display_name, scale_factor)
SPEED_LEVELS = [
    ("极慢", 0.10),
    ("慢",   0.25),
    ("中",   0.50),
    ("快",   0.75),
    ("最大", 1.00),
]

HOME_POSITIONS = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
ZERO_POSITIONS = [0.0] * 6


# ================================================================
# Xbox Arm Controller
# ================================================================

class XboxArmController:
    """基于 SDK 的 Xbox 手柄机械臂控制器（末端坐标模式）

    核心控制流程:
      100Hz 输入循环 → 末端位姿增量累加 → 完整 IK → 2 阶临界阻尼滤波 → JointCtrl
      SDK 200Hz 控制循环 → 运控模式 Type 1 帧（PD + 速度前馈 + 重力补偿）
    """

    def __init__(
        self,
        arm: ELA3Interface,
        joystick: LinuxJoystick,
        profile: ControllerProfile,
        update_rate: float = 100.0,
        max_linear_velocity: float = 0.15,
        max_angular_velocity: float = 1.5,
        deadzone: float = 0.15,
        input_smoothing: float = 0.35,
        filter_omega: float = 14.0,
        max_joint_velocity: float = 1.5,
        max_ik_jump: float = 0.5,
    ):
        self._arm = arm
        self._joy = joystick
        self._profile = profile
        self._rate = update_rate
        self._dt = 1.0 / update_rate

        self._max_lin_vel = max_linear_velocity
        self._max_ang_vel = max_angular_velocity
        self._dz_threshold = deadzone
        self._input_alpha = input_smoothing
        self._filter_omega = filter_omega
        self._max_joint_vel = max_joint_velocity
        self._max_ik_jump = max_ik_jump

        # Pinocchio kinematics
        self._kin = arm._get_kinematics()
        if self._kin is None:
            logger.warning("Pinocchio 不可用，仅支持按钮功能（无笛卡尔控制）")

        # Speed
        self._speed_idx = 2
        self._speed_factor = SPEED_LEVELS[self._speed_idx][1]

        # Mode state
        self._running = False
        self._zero_torque = False
        self._is_moving = False
        self._exit_requested = False
        self._estop = False

        # End-effector pose tracking
        self._target_pose: Optional[ArmEndPose] = None
        self._prev_pose: Optional[ArmEndPose] = None

        # IK state: seed, 2nd-order filter, jump protection
        self._ik_seed: Optional[List[float]] = None
        self._ik_filter_pos: Optional[List[float]] = None
        self._ik_filter_vel: Optional[List[float]] = None
        self._ik_raw: Optional[List[float]] = None
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0
        self._seed_just_init = False
        self._resync_cooldown = 0

        # Input EMA state (velocity → position delta)
        self._sv = [0.0] * 6  # [vx, vy, vz, wroll, wpitch, wyaw]

        # Gripper
        self._gripper_angle = 0.0
        self._gripper_step = 0.2

        # Button edge detection
        self._prev_btn = [0] * LinuxJoystick.MAX_BUTTONS
        self._prev_dpad_up = 0
        self._prev_dpad_down = 0

        # Diagnostics
        self._diag_tick = 0
        self._diag_t0 = 0.0
        self._post_move_probe_ticks = 0

    @property
    def exit_requested(self) -> bool:
        return self._exit_requested

    # ---- Public lifecycle ----

    def start(self):
        """启动控制循环（阻塞，直到 stop() 或 exit_requested）"""
        self._running = True
        self._initialize()
        self._print_banner()

        period = 1.0 / self._rate
        next_tick = time.monotonic()
        self._diag_t0 = time.monotonic()

        while self._running and not self._exit_requested:
            next_tick += period
            try:
                self._tick()
            except Exception as e:
                logger.error("控制循环异常: %s", e)

            now = time.monotonic()
            sleep_s = next_tick - now
            if sleep_s < -period:
                next_tick = now + period
            elif sleep_s > 0:
                time.sleep(sleep_s)

    def stop(self):
        self._running = False

    # ---- Initialization ----

    def _initialize(self):
        logger.info("移动到零位...")
        self._arm.MoveJ(ZERO_POSITIONS, duration=3.0, block=True)
        time.sleep(0.3)

        self._ik_seed = list(ZERO_POSITIONS)
        self._ik_filter_pos = list(ZERO_POSITIONS)
        self._ik_filter_vel = [0.0] * 6
        self._ik_raw = None
        self._seed_just_init = True
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0

        if self._kin is not None:
            self._target_pose = self._kin.forward_kinematics(ZERO_POSITIONS)
            self._prev_pose = None
            p = self._target_pose
            logger.info("初始化完成, 末端位姿: (%.3f, %.3f, %.3f) m  (%.2f, %.2f, %.2f) rad",
                        p.x, p.y, p.z, p.rx, p.ry, p.rz)
        else:
            logger.info("初始化完成 (无运动学)")

    def _print_banner(self):
        print("\n" + "=" * 52)
        print("     EL-A3 手柄控制（纯 SDK 模式）")
        print("=" * 52)
        print(f"  控制器映射:  {self._profile.display_name} [{self._profile.profile_id}]")
        print("  左摇杆       →  XY 平移")
        print("  LT / RT      →  Z 下/上")
        print("  右摇杆       →  Yaw / Roll")
        print("  LB / RB      →  Pitch")
        print("  A            →  切换速度档")
        print("  B            →  回 Home")
        print("  X            →  回零位")
        print("  Y            →  零力矩模式（可拖动）")
        print("  D-pad ↑↓     →  夹爪")
        print("  Back         →  急停")
        print("  Start        →  退出")
        print("=" * 52)
        ik_status = "末端坐标 IK" if self._kin else "不可用（缺少 pinocchio）"
        print(f"  笛卡尔控制:  {ik_status}")
        if self._target_pose is not None:
            p = self._target_pose
            print(f"  初始末端:    ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m")
            print(f"               ({p.rx:.2f}, {p.ry:.2f}, {p.rz:.2f}) rad")
        self._log_speed()

    def _log_speed(self):
        name, factor = SPEED_LEVELS[self._speed_idx]
        lin_mm = self._max_lin_vel * factor * 1000
        ang = self._max_ang_vel * factor
        print(f"  速度档位:    {self._speed_idx + 1}/5 [{name}] "
              f"({lin_mm:.0f}mm/s, {ang:.2f}rad/s)")

    # ---- Input helpers ----

    def _apply_dz(self, val: float) -> float:
        if abs(val) < self._dz_threshold:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - self._dz_threshold) / (1.0 - self._dz_threshold)

    def _apply_trigger(self, raw: float) -> float:
        """Apply trigger deadzone after profile-specific normalization."""
        norm = max(0.0, min(raw, 1.0))
        dz = self._dz_threshold * 1.5
        if norm < dz:
            return 0.0
        return (norm - dz) / (1.0 - dz)

    def _axis_value(self, binding) -> float:
        return binding.read(self._joy.axes)

    def _trigger_value(self, binding) -> float:
        return binding.read(self._joy.axes, self._joy.buttons)

    def _button_state(self, idx: Optional[int]) -> int:
        if idx is None or idx >= len(self._joy.buttons):
            return 0
        return self._joy.buttons[idx]

    def _btn_edge(self, idx: Optional[int]) -> bool:
        if idx is None or idx >= len(self._joy.buttons):
            return False
        return self._joy.buttons[idx] == 1 and self._prev_btn[idx] == 0

    # ---- Main tick ----

    def _tick(self):
        if not self._joy.connected:
            return

        buttons = self._profile.buttons
        sticks = self._profile.sticks

        # ---- Button handling (edge-triggered) ----
        if self._btn_edge(buttons.south):
            self._speed_idx = (self._speed_idx + 1) % len(SPEED_LEVELS)
            self._speed_factor = SPEED_LEVELS[self._speed_idx][1]
            self._log_speed()

        if self._btn_edge(buttons.east):
            self._async_move(HOME_POSITIONS, "Home")

        if self._btn_edge(buttons.west):
            self._async_move(ZERO_POSITIONS, "零位")

        if self._btn_edge(buttons.north):
            self._toggle_zero_torque()

        if self._btn_edge(buttons.back):
            self._emergency_stop()

        if self._btn_edge(buttons.start):
            logger.info("收到退出请求")
            self._exit_requested = True

        # D-pad gripper
        dpad_y = self._axis_value(sticks.dpad_y)
        dpad_up = 1 if dpad_y < -0.5 else 0
        dpad_down = 1 if dpad_y > 0.5 else 0
        if dpad_up and not self._prev_dpad_up:
            self._gripper_angle = min(self._gripper_angle + self._gripper_step, 1.5708)
            self._arm.GripperCtrl(gripper_angle=self._gripper_angle)
            logger.info("夹爪: %.2f rad", self._gripper_angle)
        if dpad_down and not self._prev_dpad_down:
            self._gripper_angle = max(self._gripper_angle - self._gripper_step, -1.5708)
            self._arm.GripperCtrl(gripper_angle=self._gripper_angle)
            logger.info("夹爪: %.2f rad", self._gripper_angle)
        self._prev_dpad_up = dpad_up
        self._prev_dpad_down = dpad_down

        # Save previous button state
        self._prev_btn = list(self._joy.buttons)

        # Skip motion when in non-controllable state
        if self._zero_torque or self._is_moving or self._estop:
            self._periodic_status()
            return

        if self._kin is None or self._target_pose is None:
            self._periodic_status()
            return

        # ---- End-effector velocity mapping ----
        max_lin = self._max_lin_vel * self._speed_factor
        max_ang = self._max_ang_vel * self._speed_factor

        raw = [
            -self._apply_dz(self._axis_value(sticks.ly)) * max_lin,                 # vx
            -self._apply_dz(self._axis_value(sticks.lx)) * max_lin,                 # vy
            (self._apply_trigger(self._trigger_value(sticks.rt))
             - self._apply_trigger(self._trigger_value(sticks.lt))) * max_lin,      # vz
            self._apply_dz(self._axis_value(sticks.ry)) * max_ang,                  # wroll
            (self._button_state(buttons.rb)
             - self._button_state(buttons.lb)) * max_ang,                           # wpitch
            self._apply_dz(self._axis_value(sticks.rx)) * max_ang,                  # wyaw
        ]

        # EMA smoothing with symmetric release decay
        total = sum(abs(r) for r in raw)
        if total < 1e-6:
            decay = min(self._input_alpha * 3.0, 1.0)
            self._sv = [(1 - decay) * s for s in self._sv]
        else:
            a = self._input_alpha
            self._sv = [a * r + (1 - a) * s for r, s in zip(raw, self._sv)]

        # ---- Resync cooldown: let system stabilize after resync ----
        if self._resync_cooldown > 0:
            self._resync_cooldown -= 1
            self._send_filtered()
            self._periodic_status()
            return

        # ---- Accumulate to target pose and solve full IK ----
        sv = self._sv
        sv_mag = sum(abs(v) for v in sv)
        has_input = sv_mag > 1e-7

        if self._post_move_probe_ticks > 0:
            self._post_move_probe_ticks -= 1

        if has_input:
            p = self._target_pose
            self._prev_pose = ArmEndPose(
                x=p.x, y=p.y, z=p.z, rx=p.rx, ry=p.ry, rz=p.rz)

            dt = self._dt
            self._target_pose.x += sv[0] * dt
            self._target_pose.y += sv[1] * dt
            self._target_pose.z += sv[2] * dt
            self._target_pose.rx += sv[3] * dt
            self._target_pose.ry += sv[4] * dt
            self._target_pose.rz += sv[5] * dt

            try:
                q_sol, ik_err = self._kin.ik_step(
                    self._target_pose, self._ik_seed,
                    damping=5e-3, max_step=self._max_ik_jump,
                )
                if q_sol is not None and self._accept_ik(q_sol):
                    self._ik_raw = q_sol
                    self._ik_seed = list(q_sol)
                    self._consecutive_ik_fails = 0

                    if ik_err > 0.01:
                        fk_achieved = self._kin.forward_kinematics(q_sol)
                        blend = min((ik_err - 0.01) * 20.0, 0.7)
                        self._target_pose.x += blend * (fk_achieved.x - self._target_pose.x)
                        self._target_pose.y += blend * (fk_achieved.y - self._target_pose.y)
                        self._target_pose.z += blend * (fk_achieved.z - self._target_pose.z)
                        self._target_pose.rx += blend * (fk_achieved.rx - self._target_pose.rx)
                        self._target_pose.ry += blend * (fk_achieved.ry - self._target_pose.ry)
                        self._target_pose.rz += blend * (fk_achieved.rz - self._target_pose.rz)
                else:
                    self._target_pose = self._prev_pose
                    self._consecutive_ik_fails += 1
                    if self._consecutive_ik_fails >= 10:
                        logger.warning(
                            "IK 连续失败 %d 次 (err=%.4f)，目标可能超出工作空间",
                            self._consecutive_ik_fails, ik_err)
                    if self._consecutive_ik_fails >= 50:
                        logger.warning("IK 连续失败 50+ 次，自动重新同步...")
                        self._resync_ik()
            except Exception as e:
                logger.error("IK 异常: %s", e)
                self._target_pose = self._prev_pose
        else:
            self._consecutive_ik_fails = 0

        # ---- 2nd-order filter → JointCtrl ----
        self._send_filtered()
        self._periodic_status()

    # ---- IK jump protection ----

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
            logger.warning("疑似奇异区: IK 跳变=%.3frad, 已保护 %d 帧",
                           max_diff, self._consecutive_rejects)
        if self._consecutive_rejects >= 50:
            logger.warning("IK 连续拒绝 50+ 帧，自动重新同步...")
            self._resync_ik()
        return False

    def _read_averaged_feedback(self, n_samples: int = 5, interval: float = 0.004) -> List[float]:
        samples = []
        for _ in range(n_samples):
            q = self._arm.GetArmJointMsgs().to_list()[:6]
            samples.append(q)
            time.sleep(interval)
        return [sum(s[i] for s in samples) / len(samples) for i in range(6)]

    def _resync_ik(self):
        q_avg = self._read_averaged_feedback()
        self._ik_seed = list(q_avg)
        self._ik_filter_pos = list(q_avg)
        self._ik_raw = list(q_avg)
        for i in range(6):
            self._ik_filter_vel[i] *= 0.2
        self._seed_just_init = True
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0
        self._resync_cooldown = 5

        if self._kin is not None:
            self._target_pose = self._kin.forward_kinematics(q_avg)
            self._prev_pose = None

    # ---- 2nd-order filter and motor command ----

    def _send_filtered(self):
        """2 阶临界阻尼滤波 (exact matrix-exponential) → SDK JointCtrl"""
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

        self._arm.JointCtrl(*self._ik_filter_pos,
                            velocities=list(self._ik_filter_vel))

    # ---- Button actions ----

    def _async_move(self, positions: List[float], name: str):
        if self._is_moving:
            logger.warning("正在执行其他动作，请稍后再试")
            return
        logger.info("正在移动到 %s...", name)
        self._is_moving = True
        threading.Thread(
            target=self._do_move, args=(positions, name), daemon=True).start()

    def _do_move(self, positions: List[float], name: str):
        try:
            if self._zero_torque:
                self._arm.ZeroTorqueMode(False)
                self._zero_torque = False
                time.sleep(0.2)
            if self._estop:
                self._arm.EnableArm()
                time.sleep(0.3)
                self._arm.start_control_loop(rate_hz=200.0)
                self._estop = False
            self._arm.MoveJ(positions, duration=2.0, block=True)
            feedback_q = self._arm.GetArmJointMsgs().to_list()[:6]
            self._ik_seed = list(positions)
            self._ik_filter_pos = list(positions)
            self._ik_filter_vel = [0.0] * 6
            self._ik_raw = list(positions)
            self._seed_just_init = True
            self._consecutive_rejects = 0
            self._consecutive_ik_fails = 0

            if self._kin is not None:
                self._target_pose = self._kin.forward_kinematics(positions)
                self._prev_pose = None

            self._post_move_probe_ticks = 5
            logger.info("已到达 %s", name)
        except Exception as e:
            logger.error("运动异常: %s", e)
        finally:
            self._is_moving = False

    def _toggle_zero_torque(self):
        if self._is_moving:
            return

        new_state = not self._zero_torque
        logger.info("%s 零力矩模式...", "开启" if new_state else "关闭")

        if not new_state:
            q = self._read_averaged_feedback()
            self._arm.JointCtrl(*q)
            time.sleep(0.05)

        ok = self._arm.ZeroTorqueMode(new_state)
        if ok:
            self._zero_torque = new_state
            if new_state:
                print(">>> 零力矩模式已开启: 可手动拖动机械臂 <<<")
            else:
                self._resync_ik()
                print(">>> 零力矩模式已关闭: 恢复手柄控制 <<<")
        else:
            logger.error("零力矩模式切换失败")

    def _emergency_stop(self):
        self._arm.EmergencyStop()
        self._estop = True
        print("\n!!! 急停已执行 — 按 B(Home) 或 X(零位) 恢复 !!!")

    # ---- Diagnostics ----

    def _periodic_status(self):
        self._diag_tick += 1
        if self._diag_tick < int(self._rate * 5):
            return
        self._diag_tick = 0

        q = self._arm.GetArmJointMsgs().to_list()[:6]
        degs = [f"{v * 180 / math.pi:.1f}" for v in q]
        mode = "零力矩" if self._zero_torque else ("急停" if self._estop else "正常")
        fps = self._arm.GetCanFps()
        _ok, _fail, fail_rate = self._arm.GetCanTxStats()
        tx_tag = "OK" if fail_rate < 0.01 else f"WARN({fail_rate:.1%})"
        print(f"  [{mode}] 关节(°): [{', '.join(degs)}]  CAN: {fps:.0f}fps  TX: {tx_tag}")

        if self._target_pose is not None:
            p = self._target_pose
            print(f"  末端目标: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m  "
                  f"({p.rx:.2f}, {p.ry:.2f}, {p.rz:.2f}) rad")


# ================================================================
# Main
# ================================================================


def dump_input(joy: LinuxJoystick, profile: ControllerProfile):
    print("\n" + "=" * 60)
    print("  手柄输入调试模式")
    print("=" * 60)
    print(f"设备: {joy.device}")
    print(f"Profile: {profile.display_name} [{profile.profile_id}]")
    print("按 Ctrl+C 退出，移动摇杆或按键查看原始索引变化。")

    last_axes = [None] * len(joy.axes)
    last_buttons = [None] * len(joy.buttons)
    try:
        while joy.connected:
            changed = False
            for idx, value in enumerate(joy.axes):
                prev = last_axes[idx]
                if prev is None or abs(value - prev) >= 0.05:
                    print(f"axis[{idx}] = {value:+.3f}")
                    last_axes[idx] = value
                    changed = True
            for idx, value in enumerate(joy.buttons):
                prev = last_buttons[idx]
                if prev is None or value != prev:
                    print(f"button[{idx}] = {value}")
                    last_buttons[idx] = value
                    changed = True
            if not changed:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(
        description="EL-A3 手柄控制（纯 SDK 模式）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                          # 默认参数
  %(prog)s --can can1 --js /dev/input/js1
  %(prog)s --profile zikway_3537_1041
  %(prog)s --dump-input
  %(prog)s --max-lin-vel 0.10 --kp 60 --kd 3.5
  %(prog)s --list-profiles
""",
    )
    parser.add_argument("--can", default="can0",
                        help="CAN 接口名 (默认: can0)")
    parser.add_argument("--js", default="/dev/input/js0",
                        help="手柄设备路径 (默认: /dev/input/js0)")
    parser.add_argument("--profile", default="auto",
                        choices=["auto", *PROFILES.keys()],
                        help="控制器映射 profile (默认: auto)")
    parser.add_argument("--list-profiles", action="store_true",
                        help="列出内置 profile 并退出")
    parser.add_argument("--dump-input", action="store_true",
                        help="仅打印手柄原始输入并退出")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="输入处理频率 Hz (默认: 100)")
    parser.add_argument("--max-lin-vel", type=float, default=0.15,
                        help="最大线速度 m/s (默认: 0.15)")
    parser.add_argument("--max-ang-vel", type=float, default=1.5,
                        help="最大角速度 rad/s (默认: 1.5)")
    parser.add_argument("--kp", type=float, default=80.0,
                        help="位置增益 Kp (默认: 80)")
    parser.add_argument("--kd", type=float, default=4.0,
                        help="速度增益 Kd (默认: 4)")
    parser.add_argument("--deadzone", type=float, default=None,
                        help="摇杆死区 (默认: 使用 profile 推荐值)")
    parser.add_argument("--debug", action="store_true",
                        help="调试模式")
    args = parser.parse_args()

    if args.list_profiles:
        for profile_id, profile in PROFILES.items():
            print(f"{profile_id}: {profile.display_name} - {profile.description}")
        return 0

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="[%(name)s][%(levelname)s] %(message)s",
    )
    logging.getLogger("el_a3_sdk").propagate = False

    detection = detect_controller(args.js, requested_profile=args.profile)
    deadzone = args.deadzone if args.deadzone is not None else detection.profile.default_deadzone
    logger.info(
        "手柄检测: device=%s name=%s vid=%s pid=%s profile=%s source=%s",
        detection.resolved_device,
        detection.name or "unknown",
        detection.vendor or "unknown",
        detection.product or "unknown",
        detection.profile.profile_id,
        detection.source,
    )

    # ---- Connect joystick ----
    joy = LinuxJoystick(device=args.js)
    if not joy.connect():
        print(f"\n无法打开手柄 {args.js}")
        print("请确认:")
        print("  1. 手柄已连接 (蓝牙或 USB)")
        print("  2. 设备存在: ls /dev/input/js*")
        print("  3. 权限足够: sudo chmod 666 /dev/input/js0")
        print("  4. 驱动已加载: sudo modprobe joydev")
        return 1
    logger.info("手柄已连接: %s", args.js)

    if args.dump_input:
        try:
            dump_input(joy, detection.profile)
        finally:
            joy.disconnect()
        return 0

    # ---- Connect arm ----
    arm = ELA3Interface(
        can_name=args.can,
        default_kp=args.kp,
        default_kd=args.kd,
        logger_level=LogLevel.INFO,
        gravity_feedforward_ratio=1.0,
    )

    if not arm.ConnectPort():
        joy.disconnect()
        print(f"\nCAN 接口 {args.can} 连接失败")
        print("请确认:")
        print(f"  sudo ip link set {args.can} up type can bitrate 1000000")
        return 1

    # ---- CAN 总线预检 ----
    try:
        with open(f"/sys/class/net/{args.can}/tx_queue_len") as f:
            qlen = int(f.read().strip())
        if qlen < 64:
            print(f"\n[WARNING] CAN TX 队列过小 (qlen={qlen})，可能导致丢帧，建议执行:")
            print(f"  sudo ip link set {args.can} txqueuelen 128")
    except Exception:
        pass
    bus_state = arm.GetCanBusState()
    if bus_state not in ("ERROR-ACTIVE", "UNKNOWN"):
        print(f"\n[WARNING] CAN 总线状态异常: {bus_state}，建议重置总线:")
        print(f"  sudo ip link set {args.can} down && "
              f"sudo ip link set {args.can} up type can bitrate 1000000")

    arm.EnableArm()
    time.sleep(0.5)
    arm.start_control_loop(rate_hz=200.0)

    # ---- Signal handling ----
    shutdown = threading.Event()

    def on_signal(_sig, _frame):
        shutdown.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    controller = XboxArmController(
        arm=arm,
        joystick=joy,
        profile=detection.profile,
        update_rate=args.rate,
        max_linear_velocity=args.max_lin_vel,
        max_angular_velocity=args.max_ang_vel,
        deadzone=deadzone,
    )

    ctrl_thread = threading.Thread(target=controller.start, daemon=True)
    ctrl_thread.start()

    try:
        while not shutdown.is_set() and not controller.exit_requested:
            shutdown.wait(timeout=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n正在清理...")
        controller.stop()
        try:
            arm.ZeroTorqueMode(False)
        except Exception:
            pass
        arm.stop_control_loop()
        arm.DisableArm()
        arm.DisconnectPort()
        joy.disconnect()
        print("已退出")

    return 0


if __name__ == "__main__":
    exit(main())
