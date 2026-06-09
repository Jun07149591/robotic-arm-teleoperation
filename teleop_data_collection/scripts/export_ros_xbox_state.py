#!/usr/bin/env python3
"""Export ROS2 XBOX teleop state to JSON files for data collection.

Run this alongside `real_xbox_teleop.launch.py`.  The recorder remains a pure
observer: it reads these files plus cameras and does not command the robot.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_ROOT = REPO_ROOT / "el_a3_sdk"
for _path in (str(REPO_ROOT), str(SDK_ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

ARM_JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
]
GRIPPER_JOINT = "L7_joint"


def _write_atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    tmp.replace(path)


def _mode_from_buttons(
    buttons: list[int],
    *,
    back_button_index: int = 6,
    zero_torque_button_index: int = 3,
) -> str:
    # Back is emergency stop in xbox_teleop_node; Y toggles zero torque.
    if 0 <= back_button_index < len(buttons) and buttons[back_button_index]:
        return "estop"
    if 0 <= zero_torque_button_index < len(buttons) and buttons[zero_torque_button_index]:
        return "zero_torque_toggle"
    return "normal"


def _profile_button(profile, name: str, fallback: int) -> int:
    value = getattr(profile.buttons, name, None)
    return int(value) if value is not None else int(fallback)


def _apply_controller_profile(args: argparse.Namespace) -> argparse.Namespace:
    from el_a3_sdk.controller_profiles import detect_controller, get_profile

    profile_id = args.profile
    if profile_id == "auto":
        try:
            profile = detect_controller(args.device, "auto").profile
        except Exception:
            profile = get_profile("xbox_default")
    else:
        profile = get_profile(profile_id)

    args.profile = profile.profile_id
    if args.speed_button_index is None:
        args.speed_button_index = _profile_button(profile, "south", 0)
    if args.start_button_index is None:
        args.start_button_index = _profile_button(profile, "start", 7)
    if args.back_button_index is None:
        args.back_button_index = _profile_button(profile, "back", 6)
    if args.zero_torque_button_index is None:
        args.zero_torque_button_index = _profile_button(profile, "north", 3)
    return args


def _load_ros_dependencies():
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Joy, JointState
        from el_a3_sdk.kinematics import ELA3Kinematics
    except ImportError as exc:  # pragma: no cover - only hit outside ROS2 env
        raise SystemExit(
            "ROS2 Python packages are required or could not be imported.\n"
            f"Import error: {exc}\n"
            f"Python executable: {sys.executable}\n"
            "Source ROS and run the exporter with ROS Humble's system Python, for example:\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  /usr/bin/python3 teleop_data_collection/scripts/export_ros_xbox_state.py"
        ) from exc
    return rclpy, Node, Joy, JointState, ELA3Kinematics


def _make_exporter_node_class(Node, Joy, JointState, ELA3Kinematics):
    class XboxStateExporter(Node):
        def __init__(self, args: argparse.Namespace) -> None:
            super().__init__("xbox_state_exporter")
            self._robot_state_file = Path(args.state_file)
            self._gamepad_file = Path(args.gamepad_file)
            self._profile = args.profile
            self._device = args.device
            self._speed_button_index = int(args.speed_button_index)
            self._start_button_index = int(args.start_button_index)
            self._back_button_index = int(args.back_button_index)
            self._zero_torque_button_index = int(args.zero_torque_button_index)
            self._done_hold_s = float(args.done_hold_s)
            self._rate_hz = float(args.rate_hz)
            self._kin = ELA3Kinematics()

            self._axes: list[float] = []
            self._buttons: list[int] = []
            self._prev_buttons: list[int] = []
            self._last_joint_state = None
            self._start_pressed_since: float | None = None
            self._episode_done = False
            self._speed_level = int(args.initial_speed_level)

            self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
            self.create_subscription(Joy, "/joy", self._joy_cb, 10)
            self.create_timer(1.0 / max(self._rate_hz, 1e-6), self._export_tick)
            self.get_logger().info(
                f"Exporting XBOX state: state={self._robot_state_file} "
                f"gamepad={self._gamepad_file}"
            )

        def _joy_cb(self, msg) -> None:
            self._prev_buttons = list(self._buttons)
            self._axes = [float(v) for v in msg.axes]
            self._buttons = [int(v) for v in msg.buttons]
            speed_pressed = (
                self._speed_button_index < len(self._buttons)
                and self._buttons[self._speed_button_index] == 1
                and (
                    self._speed_button_index >= len(self._prev_buttons)
                    or self._prev_buttons[self._speed_button_index] == 0
                )
            )
            if speed_pressed:
                self._speed_level = (self._speed_level % 5) + 1

            start_pressed = (
                self._start_button_index < len(self._buttons)
                and self._buttons[self._start_button_index] == 1
            )
            now = time.monotonic()
            if start_pressed:
                if self._start_pressed_since is None:
                    self._start_pressed_since = now
                elif now - self._start_pressed_since >= self._done_hold_s:
                    self._episode_done = True
            else:
                self._start_pressed_since = None
                self._episode_done = False

        def _joint_state_cb(self, msg) -> None:
            self._last_joint_state = msg

        def _export_tick(self) -> None:
            now_ns = time.time_ns()
            gamepad_payload = {
                "timestamp_ns": now_ns,
                "axes": self._axes,
                "buttons": self._buttons,
                "speed_level": self._speed_level,
                "mode": _mode_from_buttons(
                    self._buttons,
                    back_button_index=self._back_button_index,
                    zero_torque_button_index=self._zero_torque_button_index,
                ),
                "profile": self._profile,
                "device": self._device,
                "episode_done": self._episode_done,
            }
            _write_atomic_json(self._gamepad_file, gamepad_payload)

            if self._last_joint_state is None:
                return
            qpos, qvel, tau = self._extract_joint_vectors(self._last_joint_state)
            ee = self._kin.forward_kinematics(qpos[:6])
            robot_payload = {
                "timestamp_ns": now_ns,
                "qpos": qpos,
                "qvel": qvel,
                "tau": tau,
                "ee_pose": {
                    "x": ee.x,
                    "y": ee.y,
                    "z": ee.z,
                    "rx": ee.rx,
                    "ry": ee.ry,
                    "rz": ee.rz,
                },
                "can": {},
                "motor_states": {},
                "robot_status": {
                    "source": "ros2_xbox",
                    "joint_names": ARM_JOINTS + [GRIPPER_JOINT],
                    "controller_mode": gamepad_payload["mode"],
                },
                "is_estop": gamepad_payload["mode"] == "estop",
                "episode_done": self._episode_done,
            }
            _write_atomic_json(self._robot_state_file, robot_payload)

        @staticmethod
        def _extract_joint_vectors(msg) -> tuple[list[float], list[float], list[float]]:
            names = list(msg.name)
            qpos: list[float] = []
            qvel: list[float] = []
            tau: list[float] = []
            for joint_name in ARM_JOINTS + [GRIPPER_JOINT]:
                if joint_name in names:
                    idx = names.index(joint_name)
                    qpos.append(float(msg.position[idx]) if idx < len(msg.position) else 0.0)
                    qvel.append(float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0)
                    tau.append(float(msg.effort[idx]) if idx < len(msg.effort) else 0.0)
                else:
                    qpos.append(0.0)
                    qvel.append(0.0)
                    tau.append(0.0)
            return qpos, qvel, tau

    return XboxStateExporter


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export ROS2 XBOX state for collection.")
    parser.add_argument("--state-file", default="/tmp/robot_latest_state.json")
    parser.add_argument("--gamepad-file", default="/tmp/xbox_latest_input.json")
    parser.add_argument("--profile", default="auto")
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--speed-button-index", type=int, default=None)
    parser.add_argument("--start-button-index", type=int, default=None)
    parser.add_argument("--back-button-index", type=int, default=None)
    parser.add_argument("--zero-torque-button-index", type=int, default=None)
    parser.add_argument("--done-hold-s", type=float, default=1.0)
    parser.add_argument("--initial-speed-level", type=int, default=3)
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    _apply_controller_profile(args)
    rclpy, Node, Joy, JointState, ELA3Kinematics = _load_ros_dependencies()
    XboxStateExporter = _make_exporter_node_class(Node, Joy, JointState, ELA3Kinematics)
    rclpy.init()
    node = XboxStateExporter(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
