#!/usr/bin/env python3
"""
EL-A3 Xbox/gamepad teleoperation node for ROS2.

Subscribes to /joy, performs Cartesian IK control via pinocchio,
and publishes JointTrajectory commands to ros2_control controllers.

Reuses el_a3_sdk.controller_profiles for automatic device detection
and profile-aware axis/button mapping.
"""

import math
import os
import sys
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64

# Ensure el_a3_sdk is importable when running from an uninstalled source tree.
_sdk_dir = os.path.realpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "el_a3_sdk")
)
if os.path.isdir(os.path.join(_sdk_dir, "el_a3_sdk")) and _sdk_dir not in sys.path:
    sys.path.insert(0, _sdk_dir)

from el_a3_sdk.controller_profiles import (
    ControllerProfile,
    detect_controller,
    get_profile,
    PROFILES,
)
from el_a3_sdk.kinematics import ELA3Kinematics
from el_a3_sdk.data_types import ArmEndPose

from .gripper_control import XboxGripperVelocityControl


ARM_JOINTS = [
    "L1_joint", "L2_joint", "L3_joint",
    "L4_joint", "L5_joint", "L6_joint",
]
GRIPPER_JOINT = "L7_joint"

SPEED_LEVELS = [
    ("极慢", 0.10),
    ("慢",   0.25),
    ("中",   0.50),
    ("快",   0.75),
    ("最大", 1.00),
]

HOME_POSITIONS = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
ZERO_POSITIONS = [0.0] * 6


class XboxTeleopNode(Node):

    def __init__(self) -> None:
        super().__init__("xbox_teleop_node")

        # --- Parameters ---
        self.declare_parameter("controller_profile", "xbox_default")
        self.declare_parameter("controller_name", "unknown")
        self.declare_parameter("joy_device", "/dev/input/js0")
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("max_linear_velocity", 0.15)
        self.declare_parameter("max_angular_velocity", 1.5)
        self.declare_parameter("deadzone", 0.15)
        self.declare_parameter("input_smoothing", 0.35)
        self.declare_parameter("filter_omega", 14.0)
        self.declare_parameter("max_ik_jump", 0.5)
        self.declare_parameter("trajectory_time_from_start", 0.08)
        self.declare_parameter("gripper_speed", 3.0)
        self.declare_parameter("gripper_max_angle", 2.0)
        self.declare_parameter("gripper_deadzone", 0.2)
        self.declare_parameter("gripper_hold_interval", 0.25)
        self.declare_parameter("gripper_close_effort", 0.65)
        self.declare_parameter("gripper_hold_effort", 0.65)

        profile_id = str(self.get_parameter("controller_profile").value).strip()
        self._controller_name = str(self.get_parameter("controller_name").value).strip()
        self._rate = float(self.get_parameter("update_rate").value)
        self._max_lin_vel = float(self.get_parameter("max_linear_velocity").value)
        self._max_ang_vel = float(self.get_parameter("max_angular_velocity").value)
        self._dz_threshold = float(self.get_parameter("deadzone").value)
        self._input_alpha = float(self.get_parameter("input_smoothing").value)
        self._filter_omega = float(self.get_parameter("filter_omega").value)
        self._max_ik_jump = float(self.get_parameter("max_ik_jump").value)
        self._traj_dt = float(self.get_parameter("trajectory_time_from_start").value)
        self._gripper_speed = float(self.get_parameter("gripper_speed").value)
        self._gripper_max_angle = float(self.get_parameter("gripper_max_angle").value)
        self._gripper_deadzone = float(self.get_parameter("gripper_deadzone").value)
        self._gripper_hold_interval = float(self.get_parameter("gripper_hold_interval").value)
        self._gripper_close_effort = float(self.get_parameter("gripper_close_effort").value)
        self._gripper_hold_effort = float(self.get_parameter("gripper_hold_effort").value)

        # --- Resolve profile ---
        if profile_id in PROFILES:
            self._profile: ControllerProfile = PROFILES[profile_id]
        else:
            self.get_logger().warn(f"未知 profile '{profile_id}'，回退到 generic_hid")
            self._profile = PROFILES["generic_hid"]

        self.get_logger().info(
            f"控制器: {self._controller_name} | profile: {self._profile.profile_id} "
            f"({self._profile.display_name})"
        )

        # Override deadzone from profile if not explicitly set
        if self._dz_threshold == 0.15 and self._profile.default_deadzone != 0.15:
            self._dz_threshold = self._profile.default_deadzone
            self.get_logger().info(f"使用 profile 默认死区: {self._dz_threshold:.2f}")

        # --- Kinematics ---
        self._kin = ELA3Kinematics()
        self.get_logger().info("Pinocchio 运动学已初始化")

        # --- ROS interfaces ---
        cb_group = ReentrantCallbackGroup()

        self._arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self._gripper_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self._gripper_torque_pub = self.create_publisher(
            Float64, "/gripper_controller/torque_limit", 10)

        self._joy_sub = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10, callback_group=cb_group)
        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10,
            callback_group=cb_group)

        self._switch_ctrl_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller",
            callback_group=cb_group)

        # --- Control state ---
        self._dt = 1.0 / self._rate
        self._speed_idx = 2
        self._speed_factor = SPEED_LEVELS[self._speed_idx][1]

        self._zero_torque = False
        self._is_moving = False
        self._estop = False

        self._target_pose: Optional[ArmEndPose] = None
        self._prev_pose: Optional[ArmEndPose] = None
        self._ik_seed: Optional[List[float]] = None
        self._ik_filter_pos: Optional[List[float]] = None
        self._ik_filter_vel: Optional[List[float]] = None
        self._ik_raw: Optional[List[float]] = None
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0
        self._seed_just_init = False
        self._sv = [0.0] * 6

        self._gripper = XboxGripperVelocityControl(
            max_angle=self._gripper_max_angle,
            speed=self._gripper_speed,
            deadzone=self._gripper_deadzone,
            hold_interval=self._gripper_hold_interval,
            close_effort=self._gripper_close_effort,
            hold_effort=self._gripper_hold_effort,
        )

        self._current_q: Optional[List[float]] = None
        self._joint_state_received = False
        self._q_buffer: List[List[float]] = []
        self._q_buffer_size = 5
        self._resync_cooldown = 0

        # Joy state
        self._joy_axes: List[float] = [0.0] * 8
        self._joy_buttons: List[int] = [0] * 16
        self._prev_buttons: List[int] = [0] * 16
        self._joy_updated = False
        self._initialized = False

        self._diag_tick = 0

        # --- Timer ---
        timer_period = 1.0 / self._rate
        self._timer = self.create_timer(timer_period, self._control_tick, callback_group=cb_group)
        self.get_logger().info(f"控制频率: {self._rate:.0f} Hz, 轨迹dt: {self._traj_dt:.3f}s")

    # ================================================================
    # Callbacks
    # ================================================================

    def _joy_callback(self, msg: Joy) -> None:
        self._joy_axes = list(msg.axes) + [0.0] * max(0, 8 - len(msg.axes))
        self._joy_buttons = list(msg.buttons) + [0] * max(0, 16 - len(msg.buttons))
        self._joy_updated = True

    def _joint_state_callback(self, msg: JointState) -> None:
        q = [0.0] * 6
        found = 0
        for i, name in enumerate(ARM_JOINTS):
            if name in msg.name:
                idx = list(msg.name).index(name)
                q[i] = msg.position[idx]
                found += 1
        if GRIPPER_JOINT in msg.name:
            idx = list(msg.name).index(GRIPPER_JOINT)
            if idx < len(msg.position) and self._gripper.sync_feedback_angle(msg.position[idx]):
                self.get_logger().info(
                    f"夹爪初始位置同步: {self._gripper.angle:.3f} rad"
                )
        if found == 6:
            self._current_q = q
            self._q_buffer.append(list(q))
            if len(self._q_buffer) > self._q_buffer_size:
                self._q_buffer.pop(0)
            if not self._joint_state_received:
                self._joint_state_received = True
                self.get_logger().info(
                    f"收到关节状态: [{', '.join(f'{v:.3f}' for v in q)}]"
                )

    # ================================================================
    # Input helpers (use SDK profile bindings)
    # ================================================================

    def _axis_value(self, binding) -> float:
        return binding.read(self._joy_axes)

    def _trigger_value(self, binding) -> float:
        return binding.read(self._joy_axes, self._joy_buttons)

    def _button_state(self, idx: Optional[int]) -> int:
        if idx is None or idx >= len(self._joy_buttons):
            return 0
        return self._joy_buttons[idx]

    def _btn_edge(self, idx: Optional[int]) -> bool:
        if idx is None or idx >= len(self._joy_buttons):
            return False
        return self._joy_buttons[idx] == 1 and self._prev_buttons[idx] == 0

    def _apply_dz(self, val: float) -> float:
        if abs(val) < self._dz_threshold:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - self._dz_threshold) / (1.0 - self._dz_threshold)

    def _apply_trigger(self, raw: float) -> float:
        norm = max(0.0, min(raw, 1.0))
        dz = self._dz_threshold * 1.5
        if norm < dz:
            return 0.0
        return (norm - dz) / (1.0 - dz)

    # ================================================================
    # Main control tick
    # ================================================================

    def _control_tick(self) -> None:
        if not self._joint_state_received or self._current_q is None:
            return

        if not self._initialized:
            self._initialize()

        sticks = self._profile.sticks
        buttons = self._profile.buttons

        # --- Button handling (edge-triggered) ---
        if self._btn_edge(buttons.south):
            self._speed_idx = (self._speed_idx + 1) % len(SPEED_LEVELS)
            self._speed_factor = SPEED_LEVELS[self._speed_idx][1]
            name, factor = SPEED_LEVELS[self._speed_idx]
            self.get_logger().info(
                f"速度档位: {self._speed_idx+1}/5 [{name}] "
                f"({self._max_lin_vel*factor*1000:.0f}mm/s, "
                f"{self._max_ang_vel*factor:.2f}rad/s)"
            )

        if self._btn_edge(buttons.east):
            self._async_move(HOME_POSITIONS, "Home")

        if self._btn_edge(buttons.west):
            self._async_move(ZERO_POSITIONS, "零位")

        if self._btn_edge(buttons.north):
            self._toggle_zero_torque()

        if self._btn_edge(buttons.back):
            self._emergency_stop()

        if self._btn_edge(buttons.start):
            self.get_logger().info("收到退出请求 (Start)")

        # D-pad gripper: velocity control + periodic hold, matching Pico teleop.
        dpad_y = self._axis_value(sticks.dpad_y)
        gripper_cmd = self._gripper.update(
            dpad_y=dpad_y,
            dt=self._dt,
            now=time.monotonic(),
        )
        if gripper_cmd is not None:
            self._send_gripper(gripper_cmd.angle, gripper_cmd.effort)
            self.get_logger().debug(
                f"夹爪: dpad_y={dpad_y:.3f} angle={gripper_cmd.angle:.3f} rad "
                f"effort={gripper_cmd.effort:.2f} Nm"
            )

        self._prev_buttons = list(self._joy_buttons)

        if self._zero_torque or self._is_moving or self._estop:
            self._periodic_status()
            return

        if self._target_pose is None:
            self._periodic_status()
            return

        # --- Resync cooldown: let system stabilize after resync ---
        if self._resync_cooldown > 0:
            self._resync_cooldown -= 1
            self._send_filtered()
            self._periodic_status()
            return

        # --- End-effector velocity mapping ---
        max_lin = self._max_lin_vel * self._speed_factor
        max_ang = self._max_ang_vel * self._speed_factor

        raw = [
            -self._apply_dz(self._axis_value(sticks.ly)) * max_lin,
            -self._apply_dz(self._axis_value(sticks.lx)) * max_lin,
            (self._apply_trigger(self._trigger_value(sticks.rt))
             - self._apply_trigger(self._trigger_value(sticks.lt))) * max_lin,
            self._apply_dz(self._axis_value(sticks.ry)) * max_ang,
            (self._button_state(buttons.rb)
             - self._button_state(buttons.lb)) * max_ang,
            self._apply_dz(self._axis_value(sticks.rx)) * max_ang,
        ]

        total = sum(abs(r) for r in raw)
        if total < 1e-6:
            decay = min(self._input_alpha * 3.0, 1.0)
            self._sv = [(1 - decay) * s for s in self._sv]
        else:
            a = self._input_alpha
            self._sv = [a * r + (1 - a) * s for r, s in zip(raw, self._sv)]

        sv = self._sv
        has_input = sum(abs(v) for v in sv) > 1e-7

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
                    damping=5e-3, max_step=self._max_ik_jump)
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
                        self.get_logger().warn(
                            f"IK 连续失败 {self._consecutive_ik_fails} 次 "
                            f"(err={ik_err:.4f})，目标可能超出工作空间"
                        )
                    if self._consecutive_ik_fails >= 50:
                        self.get_logger().warn("IK 连续失败 50+ 次，自动重新同步...")
                        self._resync_ik()
            except Exception as e:
                self.get_logger().error(f"IK 异常: {e}")
                self._target_pose = self._prev_pose
        else:
            self._consecutive_ik_fails = 0

        self._send_filtered()
        self._periodic_status()

    # ================================================================
    # Initialization
    # ================================================================

    def _initialize(self) -> None:
        q = list(self._current_q)
        self._ik_seed = list(q)
        self._ik_filter_pos = list(q)
        self._ik_filter_vel = [0.0] * 6
        self._ik_raw = None
        self._seed_just_init = True
        self._consecutive_rejects = 0
        self._consecutive_ik_fails = 0

        self._target_pose = self._kin.forward_kinematics(q)
        self._prev_pose = None
        p = self._target_pose
        self.get_logger().info(
            f"初始化: 末端({p.x:.3f}, {p.y:.3f}, {p.z:.3f})m "
            f"({p.rx:.2f}, {p.ry:.2f}, {p.rz:.2f})rad"
        )
        self.get_logger().info(f"关节: [{', '.join(f'{v:.3f}' for v in q)}]")
        self._initialized = True

    # ================================================================
    # IK helpers
    # ================================================================

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
                f"IK 跳变保护: {max_diff:.3f}rad, 已保护 {self._consecutive_rejects} 帧")
        if self._consecutive_rejects >= 50:
            self.get_logger().warn("连续拒绝 50+ 帧，自动重新同步...")
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
        self._target_pose = self._kin.forward_kinematics(q_avg)
        self._prev_pose = None

    # ================================================================
    # 2nd-order filter and trajectory publish
    # ================================================================

    def _send_filtered(self) -> None:
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

    def _publish_arm_trajectory(
        self, positions: List[float], velocities: Optional[List[float]] = None
    ) -> None:
        msg = JointTrajectory()
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        
        # Extrapolate positions based on velocity and trajectory time 
        # to prevent spline oscillation in the trajectory controller
        extrapolated_positions = list(positions)
        if velocities is not None:
            for i in range(6):
                extrapolated_positions[i] += velocities[i] * self._traj_dt
                
        pt.positions = extrapolated_positions
        if velocities is not None:
            pt.velocities = list(velocities)
        secs = int(self._traj_dt)
        nsecs = int((self._traj_dt - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
        msg.points = [pt]
        self._arm_pub.publish(msg)

    def _send_gripper(self, angle: float, effort: float = 0.0) -> None:
        torque_msg = Float64()
        torque_msg.data = float(effort)
        self._gripper_torque_pub.publish(torque_msg)

        msg = JointTrajectory()
        msg.joint_names = [GRIPPER_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions = [angle]
        pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
        msg.points = [pt]
        self._gripper_pub.publish(msg)

    # ================================================================
    # Button actions
    # ================================================================

    def _async_move(self, positions: List[float], name: str) -> None:
        if self._is_moving:
            self.get_logger().warn("正在执行其他动作，请稍后再试")
            return
        self.get_logger().info(f"正在移动到 {name}...")
        self._is_moving = True
        threading.Thread(
            target=self._do_move, args=(positions, name), daemon=True).start()

    def _do_move(self, positions: List[float], name: str) -> None:
        try:
            if self._zero_torque:
                self._switch_to_trajectory_controller()
                self._zero_torque = False
                time.sleep(0.3)

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
                self._publish_arm_trajectory(interp)
                time.sleep(0.05)

            time.sleep(0.3)
            self._ik_seed = list(positions)
            self._ik_filter_pos = list(positions)
            self._ik_filter_vel = [0.0] * 6
            self._ik_raw = list(positions)
            self._seed_just_init = True
            self._consecutive_rejects = 0
            self._consecutive_ik_fails = 0
            self._target_pose = self._kin.forward_kinematics(positions)
            self._prev_pose = None
            self.get_logger().info(f"已到达 {name}")
        except Exception as e:
            self.get_logger().error(f"运动异常: {e}")
        finally:
            self._is_moving = False

    def _toggle_zero_torque(self) -> None:
        if self._is_moving:
            return
        new_state = not self._zero_torque
        self.get_logger().info(f"{'开启' if new_state else '关闭'} 零力矩模式...")

        if new_state:
            self._switch_to_zero_torque_controller()
        else:
            self._switch_to_trajectory_controller()
            time.sleep(0.2)
            self._resync_ik()

        self._zero_torque = new_state

    def _switch_to_zero_torque_controller(self) -> None:
        self._call_switch_controller(
            activate=["zero_torque_controller"],
            deactivate=["arm_controller", "gripper_controller"])

    def _switch_to_trajectory_controller(self) -> None:
        self._call_switch_controller(
            activate=["arm_controller", "gripper_controller"],
            deactivate=["zero_torque_controller"])

    def _call_switch_controller(
        self, activate: List[str], deactivate: List[str]
    ) -> None:
        if not self._switch_ctrl_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("controller_manager switch 服务不可用")
            return
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self._switch_ctrl_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None and future.result().ok:
            self.get_logger().info(
                f"控制器切换: activate={activate} deactivate={deactivate}")
        else:
            self.get_logger().error("控制器切换失败")

    def _emergency_stop(self) -> None:
        self._estop = True
        if self._ik_filter_pos is not None:
            self._publish_arm_trajectory(self._ik_filter_pos, [0.0] * 6)
        self.get_logger().error("!!! 急停已执行 — 按 B(Home) 或 X(零位) 恢复 !!!")

    # ================================================================
    # Diagnostics
    # ================================================================

    def _periodic_status(self) -> None:
        self._diag_tick += 1
        if self._diag_tick < int(self._rate * 5):
            return
        self._diag_tick = 0

        if self._current_q is None:
            return
        degs = [f"{v * 180 / math.pi:.1f}" for v in self._current_q]
        mode = "零力矩" if self._zero_torque else ("急停" if self._estop else "正常")
        self.get_logger().info(f"[{mode}] 关节(°): [{', '.join(degs)}]")
        if self._target_pose is not None:
            p = self._target_pose
            self.get_logger().info(
                f"末端目标: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})m "
                f"({p.rx:.2f}, {p.ry:.2f}, {p.rz:.2f})rad"
            )


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
