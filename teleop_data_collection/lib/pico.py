from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class PicoPacketView:
    raw: dict[str, Any]

    @property
    def seq(self) -> int | None:
        value = self.raw.get("seq")
        return int(value) if isinstance(value, int) else None

    @property
    def received_at(self) -> str | None:
        value = self.raw.get("receivedAt")
        return str(value) if value is not None else None

    def source(self, handedness: str) -> dict[str, Any] | None:
        for src in self.raw.get("inputSources") or []:
            if src.get("handedness") == handedness:
                return src
        return None

    def grip_pose(self, handedness: str) -> dict[str, Any] | None:
        src = self.source(handedness)
        if not src:
            return None
        return src.get("grip") or src.get("targetRay")

    def buttons(self, handedness: str) -> list[dict[str, Any]]:
        src = self.source(handedness)
        if not src:
            return []
        gp = src.get("gamepad") or {}
        return list(gp.get("buttons") or [])

    def axes(self, handedness: str) -> list[float]:
        src = self.source(handedness)
        if not src:
            return []
        gp = src.get("gamepad") or {}
        return list(gp.get("axes") or [])


def read_pico_pose(path: Path) -> PicoPacketView | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    except json.JSONDecodeError:
        return None
    if not isinstance(payload, dict):
        return None
    return PicoPacketView(raw=payload)

