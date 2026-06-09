from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class AlignedSample:
    step_idx: int
    timestamp_ns: int
    step_time_s: float
    frame: Any
    robot: dict[str, Any]
    pico: dict[str, Any] | None
    source: str

