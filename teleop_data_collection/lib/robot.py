from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

try:
    from el_a3_sdk import ELA3Interface
except ImportError:
    ELA3Interface = None  # type: ignore


@dataclass
class RobotState:
    """Snapshot of robot state, read from a shared JSON file."""
    timestamp_ns: int = 0
    qpos: list[float] = field(default_factory=list)
    qvel: list[float] = field(default_factory=list)
    tau: list[float] = field(default_factory=list)
    ee_pose: dict[str, float] = field(default_factory=dict)
    can: dict[str, Any] = field(default_factory=dict)
    motor_states: dict[str, Any] = field(default_factory=dict)
    robot_status: dict[str, Any] = field(default_factory=dict)
    valid: bool = False         # True when state was read from a file (not defaults)
    is_estop: bool = False
    episode_done: bool = False  # A long-press → motor disable → episode boundary


def read_robot_state(path: Path) -> RobotState:
    """Read robot state from ``pico_control_jointctrl.py --state-export``.

    Returns an empty ``RobotState`` if the file is missing or stale.
    """
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return RobotState()  # valid=False
    timestamp_ns = int(data.get("timestamp_ns", 0) or 0)
    if timestamp_ns <= 0 or time.time_ns() - timestamp_ns > int(1.0e9):
        return RobotState()
    return RobotState(
        valid=True,
        timestamp_ns=timestamp_ns,
        qpos=data.get("qpos", []),
        qvel=data.get("qvel", []),
        tau=data.get("tau", []),
        ee_pose=data.get("ee_pose", {}),
        can=data.get("can", {}),
        motor_states=data.get("motor_states", {}),
        robot_status=data.get("robot_status", {}),
        is_estop=data.get("is_estop", False),
        episode_done=data.get("episode_done", False),
    )


def arm_joint_list(arm: "ELA3Interface") -> list[float]:
    return list(arm.GetArmJointMsgs().to_list(include_gripper=True))


def arm_joint_velocity_list(arm: ELA3Interface) -> list[float]:
    return list(arm.GetArmJointVelocities().to_list(include_gripper=True))


def arm_joint_effort_list(arm: ELA3Interface) -> list[float]:
    return list(arm.GetArmJointEfforts().to_list(include_gripper=True))


def arm_end_pose_dict(arm: ELA3Interface) -> dict[str, float]:
    return asdict(arm.GetArmEndPoseMsgs())


def arm_status_dict(arm: ELA3Interface) -> dict[str, Any]:
    return asdict(arm.GetArmStatus())


def motor_states_dict(arm: ELA3Interface) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for motor_id, fb in arm.GetMotorStates().items():
        result[str(motor_id)] = asdict(fb)
    return result


def can_stats_dict(arm: ELA3Interface) -> dict[str, Any]:
    success, failed, recent_rate = arm.GetCanTxStats()
    return {
        "can_name": arm.GetCanName(),
        "fps": arm.GetCanFps(),
        "tx_success": success,
        "tx_failed": failed,
        "tx_failure_rate": recent_rate,
        "bus_state": arm.GetCanBusState(),
    }
