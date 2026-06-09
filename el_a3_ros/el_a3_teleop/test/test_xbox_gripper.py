from __future__ import annotations

import unittest

from el_a3_teleop.gripper_control import XboxGripperVelocityControl


class XboxGripperVelocityControlTest(unittest.TestCase):
    def test_vertical_dpad_moves_gripper_continuously_and_clamps(self) -> None:
        control = XboxGripperVelocityControl(
            angle=0.5,
            last_sent=0.5,
            max_angle=2.0,
            speed=3.0,
            deadzone=0.2,
            min_send_delta=0.01,
        )

        command = control.update(dpad_y=-1.0, dt=0.1, now=1.0)

        self.assertIsNotNone(command)
        self.assertAlmostEqual(command.angle, 0.8, places=6)
        self.assertAlmostEqual(command.effort, 0.65, places=6)
        self.assertAlmostEqual(control.angle, 0.8, places=6)
        self.assertTrue(control.hold_active)

        command = control.update(dpad_y=-1.0, dt=1.0, now=2.0)

        self.assertIsNotNone(command)
        self.assertAlmostEqual(command.angle, 2.0, places=6)
        self.assertAlmostEqual(command.effort, 0.65, places=6)
        self.assertAlmostEqual(control.angle, 2.0, places=6)

    def test_idle_closed_gripper_reissues_hold_at_interval(self) -> None:
        control = XboxGripperVelocityControl(
            angle=0.7,
            last_sent=0.7,
            hold_active=True,
            last_hold_time=1.0,
            hold_interval=0.25,
        )

        self.assertIsNone(control.update(dpad_y=0.0, dt=0.02, now=1.1))
        command = control.update(dpad_y=0.0, dt=0.02, now=1.3)
        self.assertIsNotNone(command)
        self.assertEqual(command.angle, 0.7)
        self.assertAlmostEqual(command.effort, 0.65, places=6)
        self.assertAlmostEqual(control.last_hold_time, 1.3, places=6)

    def test_open_below_release_threshold_disables_hold(self) -> None:
        control = XboxGripperVelocityControl(
            angle=0.04,
            last_sent=0.04,
            hold_active=True,
            last_hold_time=1.0,
            speed=3.0,
            deadzone=0.2,
        )

        command = control.update(dpad_y=1.0, dt=0.1, now=1.1)

        self.assertIsNotNone(command)
        self.assertEqual(command.angle, 0.0)
        self.assertEqual(command.effort, 0.0)
        self.assertFalse(control.hold_active)
        self.assertAlmostEqual(control.angle, 0.0, places=6)

    def test_custom_efforts_are_used_for_close_and_hold(self) -> None:
        control = XboxGripperVelocityControl(
            angle=0.1,
            last_sent=0.1,
            hold_active=True,
            last_hold_time=1.0,
            close_effort=0.8,
            hold_effort=0.12,
            hold_interval=0.25,
        )

        close_command = control.update(dpad_y=-1.0, dt=0.1, now=1.1)
        self.assertIsNotNone(close_command)
        self.assertAlmostEqual(close_command.effort, 0.8, places=6)

        hold_command = control.update(dpad_y=0.0, dt=0.02, now=1.4)
        self.assertIsNotNone(hold_command)
        self.assertAlmostEqual(hold_command.effort, 0.12, places=6)

    def test_feedback_sync_initializes_gripper_target_once(self) -> None:
        control = XboxGripperVelocityControl(max_angle=2.0)

        self.assertTrue(control.sync_feedback_angle(0.9))
        self.assertAlmostEqual(control.angle, 0.9, places=6)
        self.assertAlmostEqual(control.last_sent, 0.9, places=6)

        self.assertFalse(control.sync_feedback_angle(0.2))
        self.assertAlmostEqual(control.angle, 0.9, places=6)


if __name__ == "__main__":
    unittest.main()
