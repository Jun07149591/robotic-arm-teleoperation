from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


GAMEPAD_OBSERVATION_DIM = 32
GAMEPAD_AXIS_DIM = 16
GAMEPAD_BUTTON_DIM = 12
XBOX_OBSERVATION_NAMES = [
    "lx", "ly", "rx", "ry", "lt", "rt", "dpad_x", "dpad_y",
    "btn_a", "btn_b", "btn_x", "btn_y", "btn_lb", "btn_rb",
    "btn_back", "btn_start", "valid", "speed_level",
    "mode_normal", "episode_done",
]
XBOX_OBSERVATION_DIM = len(XBOX_OBSERVATION_NAMES)


@dataclass
class GamepadState:
    """Snapshot of an XBOX/gamepad controller exported by a ROS2 helper."""

    timestamp_ns: int = 0
    axes: list[float] = field(default_factory=list)
    buttons: list[int] = field(default_factory=list)
    speed_level: int = 0
    mode: str = "unknown"
    profile: str = ""
    device: str = ""
    episode_done: bool = False
    raw: dict[str, Any] = field(default_factory=dict)
    valid: bool = False


def read_gamepad_state(path: Path, *, max_age_s: float = 1.0) -> GamepadState:
    """Read a gamepad snapshot file, returning invalid state if missing or stale."""

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return GamepadState()
    if not isinstance(data, dict):
        return GamepadState()

    timestamp_ns = int(data.get("timestamp_ns", 0) or 0)
    if timestamp_ns <= 0 or time.time_ns() - timestamp_ns > int(max_age_s * 1e9):
        return GamepadState()

    return GamepadState(
        valid=True,
        timestamp_ns=timestamp_ns,
        axes=[float(v) for v in list(data.get("axes") or [])],
        buttons=[int(v) for v in list(data.get("buttons") or [])],
        speed_level=int(data.get("speed_level", 0) or 0),
        mode=str(data.get("mode", "unknown")),
        profile=str(data.get("profile", "")),
        device=str(data.get("device", "")),
        episode_done=bool(data.get("episode_done", False)),
        raw=data,
    )


def flatten_gamepad_observation(state: GamepadState | dict[str, Any] | None) -> np.ndarray:
    """Return the fixed-width `observation.gamepad` vector.

    Layout:
    - 0..15: raw axes, padded with zeros
    - 16..27: raw buttons, padded with zeros
    - 28: valid flag
    - 29: speed level
    - 30: normal-mode flag
    - 31: episode_done flag
    """

    if state is None:
        return np.zeros((GAMEPAD_OBSERVATION_DIM,), dtype=np.float32)
    if isinstance(state, dict):
        state = GamepadState(
            valid=bool(state.get("valid", True)),
            timestamp_ns=int(state.get("timestamp_ns", 0) or 0),
            axes=[float(v) for v in list(state.get("axes") or [])],
            buttons=[int(v) for v in list(state.get("buttons") or [])],
            speed_level=int(state.get("speed_level", 0) or 0),
            mode=str(state.get("mode", "unknown")),
            profile=str(state.get("profile", "")),
            device=str(state.get("device", "")),
            episode_done=bool(state.get("episode_done", False)),
            raw=dict(state),
        )

    values = [0.0] * GAMEPAD_OBSERVATION_DIM
    for i, value in enumerate(state.axes[:GAMEPAD_AXIS_DIM]):
        values[i] = float(value)
    for i, value in enumerate(state.buttons[:GAMEPAD_BUTTON_DIM]):
        values[GAMEPAD_AXIS_DIM + i] = float(value)
    values[28] = 1.0 if state.valid else 0.0
    values[29] = float(state.speed_level)
    values[30] = 1.0 if state.mode == "normal" else 0.0
    values[31] = 1.0 if state.episode_done else 0.0
    return np.array(values, dtype=np.float32)


def flatten_xbox_observation(state: GamepadState | dict[str, Any] | None) -> np.ndarray:
    """Return the compact `observation.xbox` vector.

    Layout is semantic, not raw padded Joy arrays:
    lx, ly, rx, ry, lt, rt, dpad_x, dpad_y,
    A/B/X/Y, LB/RB, Back/Start, valid, speed level, normal-mode, done.
    """

    if state is None:
        return np.zeros((XBOX_OBSERVATION_DIM,), dtype=np.float32)
    if isinstance(state, dict):
        state = GamepadState(
            valid=bool(state.get("valid", True)),
            timestamp_ns=int(state.get("timestamp_ns", 0) or 0),
            axes=[float(v) for v in list(state.get("axes") or [])],
            buttons=[int(v) for v in list(state.get("buttons") or [])],
            speed_level=int(state.get("speed_level", 0) or 0),
            mode=str(state.get("mode", "unknown")),
            profile=str(state.get("profile", "")),
            device=str(state.get("device", "")),
            episode_done=bool(state.get("episode_done", False)),
            raw=dict(state),
        )

    axis_map, trigger_map, button_map = _xbox_profile_maps(state.profile)
    values = [0.0] * XBOX_OBSERVATION_DIM
    for out_i, name in enumerate(("lx", "ly", "rx", "ry", "dpad_x", "dpad_y")):
        values[out_i if out_i < 4 else out_i + 2] = _read_axis(
            state.axes, axis_map[name]
        )
    values[4] = _read_trigger(state.axes, state.buttons, trigger_map["lt"])
    values[5] = _read_trigger(state.axes, state.buttons, trigger_map["rt"])
    for out_i, name in enumerate(
        ("a", "b", "x", "y", "lb", "rb", "back", "start"),
        start=8,
    ):
        values[out_i] = _read_button(state.buttons, button_map[name])
    values[16] = 1.0 if state.valid else 0.0
    values[17] = float(state.speed_level)
    values[18] = 1.0 if state.mode == "normal" else 0.0
    values[19] = 1.0 if state.episode_done else 0.0
    return np.array(values, dtype=np.float32)


def _xbox_profile_maps(profile: str) -> tuple[dict[str, int], dict[str, tuple[str, int]], dict[str, int]]:
    if profile in ("zikway_3537_1041", "generic_hid"):
        return (
            {"lx": 0, "ly": 1, "rx": 2, "ry": 3, "dpad_x": 6, "dpad_y": 7},
            {"lt": ("axis", 4), "rt": ("axis", 5)},
            {"a": 0, "b": 1, "x": 3, "y": 4, "lb": 6, "rb": 7, "back": 10, "start": 11},
        )
    return (
        {"lx": 0, "ly": 1, "rx": 3, "ry": 4, "dpad_x": 6, "dpad_y": 7},
        {"lt": ("axis", 2), "rt": ("axis", 5)},
        {"a": 0, "b": 1, "x": 2, "y": 3, "lb": 4, "rb": 5, "back": 6, "start": 7},
    )


def _read_axis(axes: list[float], index: int) -> float:
    if index < 0 or index >= len(axes):
        return 0.0
    return float(axes[index])


def _read_trigger(axes: list[float], buttons: list[int], binding: tuple[str, int]) -> float:
    kind, index = binding
    if kind == "button":
        return _read_button(buttons, index)
    return max(0.0, min(1.0, _read_axis(axes, index) * 0.5 + 0.5))


def _read_button(buttons: list[int], index: int) -> float:
    if index < 0 or index >= len(buttons):
        return 0.0
    return 1.0 if int(buttons[index]) else 0.0
