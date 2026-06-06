#!/usr/bin/env python3
"""Pico JointCtrl experimental controller tests.

These tests use a fake arm and do not require CAN hardware.
"""
import os
import sys
import math
import unittest
from contextlib import redirect_stdout
from io import StringIO
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from demo.pico_control_jointctrl import (
    PicoArmController,
    _ArmController,
    _hold_current_position_after_enable,
    _map_hand_delta_to_robot_delta,
    _read_pico_pose,
    _thumbstick_axes,
)
from el_a3_sdk import ArmState
from el_a3_sdk.data_types import ArmEndPose


class _FakeKin:
    def forward_kinematics(self, _q):
        return ArmEndPose(x=0.1, y=0.0, z=0.2, rx=0.0, ry=0.0, rz=0.0)

    def ik_step(self, target_pose, _seed, **_kwargs):
        return [target_pose.y, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0


class _JointStates:
    def __init__(self, positions):
        self._positions = list(positions)

    def to_list(self, *args, **kwargs):
        return list(self._positions)


class _FakeArm:
    def __init__(self):
        self.move_l_calls = []
        self.joint_ctrl_calls = []
        self.gripper_ctrl_calls = []
        self.q = [0.0] * 6
        self.arm_state = ArmState.ENABLED
        self.cancel_motion_calls = 0
        self.raise_on_feedback = False
        self.last_torque_ff = None
        self.kin = None

    def _get_kinematics(self):
        return self.kin

    def GetArmJointMsgs(self):
        if self.raise_on_feedback:
            raise RuntimeError("feedback unavailable")
        return _JointStates(self.q)

    def GetMotorStates(self):
        return {}

    def MoveL(self, *args, **kwargs):
        self.move_l_calls.append((args, kwargs))
        return True

    def JointCtrl(self, *positions, velocities=None, torque_ff=None):
        q = list(positions[:6])
        self.joint_ctrl_calls.append((q, list(velocities or [])))
        self.last_torque_ff = list(torque_ff) if torque_ff is not None else None
        self.q = q
        return True

    def ComputeGravityTorques(self, positions=None):
        return [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    def GripperPositionCtrl(self, gripper_angle=0.0, **kwargs):
        self.gripper_ctrl_calls.append(("position", gripper_angle, kwargs))
        return True

    def GripperCtrl(self, gripper_angle=None, stop=False, **kwargs):
        self.gripper_ctrl_calls.append(("ctrl", gripper_angle, {"stop": stop, **kwargs}))
        return True

    def cancel_motion(self):
        self.cancel_motion_calls += 1


class _FakeGripperHold:
    CLOSING = "closing"
    GRASPED = "grasped"

    def __init__(self):
        self.active = False
        self.state = self.CLOSING
        self.target_angle = 0.0
        self.grip_calls = []
        self.release_calls = 0

    def tick(self):
        return None

    def grip(self, angle, effort=1.5):
        self.active = True
        self.target_angle = angle
        self.grip_calls.append((angle, effort))

    def release(self):
        self.active = False
        self.release_calls += 1


def _make_controller() -> tuple[_ArmController, _FakeArm]:
    arm = _FakeArm()
    ctrl = _ArmController(
        name="test",
        arm=arm,
        update_rate=100.0,
        max_lin_vel=0.15,
        max_ang_vel=1.5,
        pos_scale=1.0,
        deadzone=0.03,
        input_alpha=0.08,
        filter_omega=14.0,
        max_ik_jump=0.1,
    )
    ctrl._target_pose = ArmEndPose(x=0.1, y=0.0, z=0.2, rx=0.0, ry=0.0, rz=0.0)
    ctrl._tracking_engaged = True
    ctrl._last_sent_pose_tuple = None
    ctrl._ik_raw = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ctrl._ik_filter_pos = [0.0] * 6
    ctrl._ik_filter_vel = [0.0] * 6
    return ctrl, arm


class PicoJointCtrlTests(unittest.TestCase):
    def test_hand_delta_mapping_is_intuitive_xyz(self):
        # Observed robot frame for Pico teleop:
        # hand X = left/right, hand Y = up/down, hand Z = front/back.
        self.assertEqual(_map_hand_delta_to_robot_delta([1.0, 0.0, 0.0]), [-1.0, 0.0, 0.0])
        self.assertEqual(_map_hand_delta_to_robot_delta([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0])
        self.assertEqual(_map_hand_delta_to_robot_delta([0.0, 0.0, 1.0]), [0.0, 1.0, 0.0])

    def test_thumbstick_axes_accepts_pico_primary_axis_pair(self):
        self.assertEqual(_thumbstick_axes([0.0, -1.0, 0.0, 0.0]), (0.0, -1.0))
        self.assertEqual(_thumbstick_axes([0.0, 0.0, 0.5, -0.75]), (0.5, -0.75))

    def test_hold_current_position_after_enable_commands_feedback_pose(self):
        _ctrl, arm = _make_controller()
        arm.q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]

        ok = _hold_current_position_after_enable(arm, "test")

        self.assertTrue(ok)
        self.assertEqual(len(arm.joint_ctrl_calls), 1)
        q_cmd, vel_cmd = arm.joint_ctrl_calls[-1]
        self.assertEqual(q_cmd, [0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
        self.assertEqual(vel_cmd, [0.0] * 6)
        self.assertEqual(arm.last_torque_ff, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    def test_tracking_output_uses_joint_ctrl_not_movel(self):
        ctrl, arm = _make_controller()

        ctrl._send_filtered()

        self.assertEqual(arm.move_l_calls, [])
        self.assertEqual(len(arm.joint_ctrl_calls), 1)

    def test_tracking_output_advances_from_internal_command_state(self):
        ctrl, arm = _make_controller()
        ctrl._joint_max_vel = 0.5

        ctrl._send_filtered()

        self.assertEqual(len(arm.joint_ctrl_calls), 1)
        q_cmd, vel_cmd = arm.joint_ctrl_calls[-1]
        self.assertGreater(q_cmd[0], 0.0)

    def test_tracking_output_sends_velocity_feedforward(self):
        ctrl, arm = _make_controller()
        ctrl._joint_max_vel = 100.0

        ctrl._send_filtered()

        self.assertEqual(len(arm.joint_ctrl_calls), 1)
        _q_cmd, vel_cmd = arm.joint_ctrl_calls[-1]
        self.assertEqual(len(vel_cmd), 6)
        self.assertGreater(vel_cmd[0], 0.0)

    def test_tracking_output_skips_when_arm_is_not_enabled(self):
        ctrl, arm = _make_controller()
        arm.arm_state = ArmState.IDLE

        ctrl._send_filtered()

        self.assertEqual(arm.move_l_calls, [])
        self.assertEqual(arm.joint_ctrl_calls, [])

    def test_tracking_output_clips_from_last_command_when_feedback_unavailable(self):
        ctrl, arm = _make_controller()
        ctrl._joint_max_vel = 0.5
        ctrl._last_joint_cmd = [0.0] * 6
        arm.raise_on_feedback = True

        ctrl._send_filtered()

        self.assertEqual(len(arm.joint_ctrl_calls), 1)
        q_cmd, _vel_cmd = arm.joint_ctrl_calls[-1]
        self.assertGreater(q_cmd[0], 0.0)
        self.assertLessEqual(q_cmd[0], 0.5 * ctrl._dt + 1e-9)

    def test_tracking_output_does_not_follow_falling_feedback(self):
        ctrl, arm = _make_controller()
        ctrl._joint_max_vel = 0.5
        ctrl._last_joint_cmd = [0.0] * 6
        arm.q = [-0.2, 0.0, 0.0, 0.0, 0.0, 0.0]

        ctrl._send_filtered()

        self.assertEqual(len(arm.joint_ctrl_calls), 1)
        q_cmd, _vel_cmd = arm.joint_ctrl_calls[-1]
        self.assertGreaterEqual(q_cmd[0], 0.0)

    def test_missing_pico_packet_brakes_active_tracking_stream(self):
        arm = _FakeArm()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
        )
        controller._initialize()
        right = controller._right
        right._tracking_engaged = True
        right._ik_raw = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        right._ik_filter_pos = [0.0] * 6
        right._ik_filter_vel = [0.0] * 6
        right._last_joint_cmd = [0.0] * 6

        with patch("demo.pico_control_jointctrl._read_pico_pose", return_value=None):
            controller._tick()

        self.assertGreater(len(arm.joint_ctrl_calls), 0)
        q_cmd, vel_cmd = arm.joint_ctrl_calls[-1]
        self.assertEqual(q_cmd, [0.0] * 6)
        self.assertEqual(vel_cmd, [0.0] * 6)

    def test_repeated_pico_seq_brakes_after_duplicate_threshold(self):
        arm = _FakeArm()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
        )
        controller._initialize()
        controller._last_pose_seq = 10
        controller._duplicate_pose_count = 1
        controller._max_duplicate_ticks = 2
        right = controller._right
        right._tracking_engaged = True
        right._ik_raw = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        right._ik_filter_pos = [0.0] * 6
        right._ik_filter_vel = [0.0] * 6
        right._last_joint_cmd = [0.0] * 6
        packet = {"seq": 10, "inputSources": []}

        with patch("demo.pico_control_jointctrl._read_pico_pose", return_value=packet):
            controller._tick()

        self.assertGreater(len(arm.joint_ctrl_calls), 0)
        q_cmd, vel_cmd = arm.joint_ctrl_calls[-1]
        self.assertEqual(q_cmd, [0.0] * 6)
        self.assertEqual(vel_cmd, [0.0] * 6)

    def test_right_gripper_stick_works_while_tracking(self):
        arm = _FakeArm()
        arm.kin = _FakeKin()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
            grip_speed=2.0,
        )
        controller._initialize()
        controller._buttons_synced = True
        right = controller._right
        right._calibrated = True
        right._tracking_engaged = True
        right._gripper_available = True
        right._gripper_hold = _FakeGripperHold()
        packet = {
            "seq": 10,
            "inputSources": [
                {
                    "handedness": "right",
                    "grip": {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "gamepad": {
                        "axes": [0.0, -1.0, 0.0, 0.0],
                        "buttons": [{}, {}, {}, {}, {}, {}],
                    },
                }
            ],
        }

        with patch("demo.pico_control_jointctrl._read_pico_pose", return_value=packet):
            controller._tick()

        self.assertGreater(len(right._gripper_hold.grip_calls), 0)
        angle, effort = right._gripper_hold.grip_calls[-1]
        self.assertGreater(angle, 0.0)
        self.assertEqual(effort, 0.65)

    def test_right_gripper_stick_sends_command_without_feedback(self):
        arm = _FakeArm()
        arm.kin = _FakeKin()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
            grip_speed=2.0,
        )
        controller._initialize()
        controller._buttons_synced = True
        right = controller._right
        right._calibrated = True
        right._gripper_available = False
        right._gripper_hold = _FakeGripperHold()
        packet = {
            "seq": 11,
            "inputSources": [
                {
                    "handedness": "right",
                    "grip": {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "gamepad": {
                        "axes": [0.0, -1.0, 0.0, 0.0],
                        "buttons": [{}, {}, {}, {}, {}, {}],
                    },
                }
            ],
        }

        with patch("demo.pico_control_jointctrl._read_pico_pose", return_value=packet):
            controller._tick()

        self.assertGreater(len(right._gripper_hold.grip_calls), 0)

    def test_fine_yaw_small_rotation_is_not_eaten_by_large_deadband(self):
        arm = _FakeArm()
        arm.kin = _FakeKin()
        ctrl = _ArmController(
            name="test",
            arm=arm,
            update_rate=100.0,
            max_lin_vel=0.15,
            max_ang_vel=1.5,
            pos_scale=1.0,
            deadzone=0.003,
            input_alpha=1.0,
            filter_omega=14.0,
            max_ik_jump=0.5,
            yaw_scale=1.0,
        )
        ctrl.init_position()
        ctrl._fine_yaw_mode = True
        ctrl._ref_hand_rpy = [0.0, 0.0, 0.0]
        ctrl._ref_robot_rpy = [0.0, 0.0, 0.0]
        angle = 0.02
        pose = {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "qx": 0.0, "qy": 0.0,
            "qz": math.sin(angle / 2.0), "qw": math.cos(angle / 2.0),
        }

        moved = ctrl.process_pose(pose, speed_factor=0.5)

        self.assertTrue(moved)
        self.assertAlmostEqual(ctrl._target_pose.ry, -angle, places=4)

    def test_base_yaw_updates_first_joint_target(self):
        ctrl, arm = _make_controller()
        ctrl._ik_raw = [0.0] * 6
        ctrl._ik_filter_pos = [0.0] * 6
        ctrl._ik_filter_vel = [0.0] * 6
        ctrl._last_joint_cmd = [0.0] * 6

        moved = ctrl.apply_base_yaw(angular_delta=0.1)

        self.assertTrue(moved)
        self.assertGreater(ctrl._ik_raw[0], 0.0)

    def test_single_arm_mode_initializes_only_right_controller(self):
        arm = _FakeArm()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
        )

        controller._initialize()

        self.assertEqual(len(arm.joint_ctrl_calls), 0)

    def test_single_arm_banner_uses_right_controller_speed(self):
        arm = _FakeArm()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
        )

        with redirect_stdout(StringIO()):
            controller._log_speed()

    def test_single_arm_first_packet_syncs_buttons_without_starting_a_action(self):
        arm = _FakeArm()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
        )
        controller._initialize()
        calls_after_init = len(arm.joint_ctrl_calls)
        packet = {
            "seq": 1,
            "inputSources": [
                {
                    "handedness": "right",
                    "grip": {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "gamepad": {
                        "axes": [0.0, 0.0, 0.0, 0.0],
                        "buttons": [{}, {}, {}, {}, {"pressed": True, "value": 1.0}, {}],
                    },
                }
            ],
        }

        packet2 = dict(packet)
        packet2["seq"] = 2
        with patch("demo.pico_control_jointctrl._read_pico_pose", side_effect=[packet, packet2]):
            controller._tick()
            controller._tick()

        self.assertIsNone(controller._a_hold_start)
        self.assertEqual(controller._prev_btn_right[4], 1)
        self.assertEqual(len(arm.joint_ctrl_calls), calls_after_init)

    def test_single_arm_idle_pose_packets_do_not_stream_jointctrl(self):
        arm = _FakeArm()
        arm.kin = _FakeKin()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
        )
        controller._initialize()
        packet = {
            "seq": 1,
            "inputSources": [
                {
                    "handedness": "right",
                    "grip": {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "gamepad": {
                        "axes": [0.0, 0.0, 0.0, 0.0],
                        "buttons": [{}, {}, {}, {}, {"pressed": False, "value": 0.0}, {}],
                    },
                }
            ],
        }
        packet2 = dict(packet)
        packet2["seq"] = 2
        packet3 = dict(packet)
        packet3["seq"] = 3

        with patch("demo.pico_control_jointctrl._read_pico_pose", side_effect=[packet, packet2, packet3]):
            controller._tick()  # calibration
            controller._tick()  # button sync
            controller._tick()  # idle packet after setup

        self.assertEqual(arm.joint_ctrl_calls, [])

    def test_viewer_position_does_not_change_world_frame_hand_delta(self):
        arm = _FakeArm()
        arm.kin = _FakeKin()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
        )
        controller._initialize()
        controller._buttons_synced = True
        right = controller._right
        right._calibrated = True
        right._tracking_engaged = True
        right._skip_frames = 0
        right._prev_hand_pos = [0.0, 0.0, 0.0]
        right._last_raw_hand_pos = [0.0, 0.0, 0.0]
        right._hand_filtered = [0.0, 0.0, 0.0]
        packet = {
            "seq": 20,
            "viewer": {"position": {"x": 1.0, "y": 0.0, "z": 0.0}},
            "inputSources": [
                {
                    "handedness": "right",
                    "grip": {
                        "position": {"x": 0.10, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "gamepad": {
                        "axes": [0.0, 0.0, 0.0, 0.0],
                        "buttons": [{}, {}, {}, {}, {}, {}],
                    },
                }
            ],
        }

        with patch("demo.pico_control_jointctrl._read_pico_pose", return_value=packet):
            controller._tick()

        self.assertLess(right._target_pose.x, 0.05)

    def test_viewer_correlated_controller_drift_is_rejected(self):
        arm = _FakeArm()
        arm.kin = _FakeKin()
        controller = PicoArmController(
            left_arm=arm,
            right_arm=arm,
            update_rate=100.0,
            single_arm=True,
        )
        controller._initialize()
        controller._buttons_synced = True
        controller._last_viewer_pos = [0.0, 0.0, 0.0]
        right = controller._right
        right._calibrated = True
        right._tracking_engaged = True
        right._skip_frames = 0
        right._prev_hand_pos = [0.0, 0.0, 0.0]
        right._last_raw_hand_pos = [0.0, 0.0, 0.0]
        right._hand_filtered = [0.0, 0.0, 0.0]
        before = right._target_pose.x
        packet = {
            "seq": 21,
            "viewer": {"position": {"x": 0.10, "y": 0.0, "z": 0.0}},
            "inputSources": [
                {
                    "handedness": "right",
                    "grip": {
                        "position": {"x": 0.10, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "gamepad": {
                        "axes": [0.0, 0.0, 0.0, 0.0],
                        "buttons": [{}, {}, {}, {}, {}, {}],
                    },
                }
            ],
        }

        with patch("demo.pico_control_jointctrl._read_pico_pose", return_value=packet):
            controller._tick()

        self.assertEqual(right._sv_vel, [0.0, 0.0, 0.0])
        self.assertEqual(right._target_pose.x, before)

    def test_pose_tracking_maps_hand_delta_to_visible_target_delta(self):
        arm = _FakeArm()
        arm.kin = _FakeKin()
        ctrl = _ArmController(
            name="test",
            arm=arm,
            update_rate=100.0,
            max_lin_vel=0.15,
            max_ang_vel=1.5,
            pos_scale=1.0,
            deadzone=0.003,
            input_alpha=1.0,
            filter_omega=14.0,
            max_ik_jump=0.5,
        )
        ctrl.init_position()
        ctrl._tracking_engaged = True
        ctrl._skip_frames = 0
        ctrl.calibrate({"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0})

        moved = ctrl.process_pose(
            {"x": 0.10, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            speed_factor=0.5,
        )

        self.assertTrue(moved)
        self.assertLess(ctrl._target_pose.x, 0.05)

    def test_pose_tracking_zero_delta_clears_residual_step(self):
        arm = _FakeArm()
        arm.kin = _FakeKin()
        ctrl = _ArmController(
            name="test",
            arm=arm,
            update_rate=100.0,
            max_lin_vel=0.15,
            max_ang_vel=1.5,
            pos_scale=1.0,
            deadzone=0.003,
            input_alpha=1.0,
            filter_omega=14.0,
            max_ik_jump=0.5,
        )
        ctrl.init_position()
        ctrl._tracking_engaged = True
        ctrl._skip_frames = 0
        ctrl.calibrate({"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0})
        ctrl._sv_vel = [0.05, 0.0, 0.0]
        before = ctrl._target_pose.x

        moved = ctrl.process_pose(
            {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            speed_factor=0.5,
        )

        self.assertTrue(moved)
        self.assertEqual(ctrl._sv_vel, [0.0, 0.0, 0.0])
        self.assertEqual(ctrl._target_pose.x, before)

    def test_pose_tracking_static_raw_hand_does_not_drift_from_input_lpf(self):
        arm = _FakeArm()
        arm.kin = _FakeKin()
        ctrl = _ArmController(
            name="test",
            arm=arm,
            update_rate=100.0,
            max_lin_vel=0.15,
            max_ang_vel=1.5,
            pos_scale=1.0,
            deadzone=0.003,
            input_alpha=0.2,
            filter_omega=14.0,
            max_ik_jump=0.5,
        )
        ctrl.init_position()
        ctrl._tracking_engaged = True
        ctrl._skip_frames = 0
        ctrl.calibrate({"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0})

        pose = {"x": 0.10, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        ctrl.process_pose(pose, speed_factor=0.5)
        after_first = ctrl._target_pose.x

        ctrl.process_pose(pose, speed_factor=0.5)

        self.assertEqual(ctrl._sv_vel, [0.0, 0.0, 0.0])
        self.assertEqual(ctrl._target_pose.x, after_first)

    def test_read_pico_pose_retries_transient_partial_json(self):
        reads = [StringIO("{"), StringIO('{"seq": 42, "inputSources": []}')]

        with patch("builtins.open", side_effect=reads), patch("time.sleep"):
            packet = _read_pico_pose()

        self.assertEqual(packet["seq"], 42)


if __name__ == "__main__":
    unittest.main()
