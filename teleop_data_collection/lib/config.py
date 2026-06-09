from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class CollectionConfig:
    dataset: dict[str, Any]
    camera: dict[str, Any]
    teleop: dict[str, Any]
    robot: dict[str, Any]


def load_config(path: Path) -> CollectionConfig:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"配置文件必须是 mapping: {path}")
    return CollectionConfig(
        dataset=dict(payload.get("dataset", {})),
        camera=dict(payload.get("camera", {})),
        teleop=dict(payload.get("teleop", {})),
        robot=dict(payload.get("robot", {})),
    )


def resolve_camera_configs(camera: dict[str, Any]) -> list[dict[str, Any]]:
    cameras = camera.get("cameras")
    if isinstance(cameras, list):
        resolved = [dict(cam) for cam in cameras if isinstance(cam, dict)]
        if resolved:
            return resolved
    if camera:
        single = dict(camera)
        single.setdefault("name", "wrist")
        single.pop("cameras", None)
        return [single]
    return []


def resolve_camera_config(camera: dict[str, Any], camera_name: str | None = None) -> dict[str, Any]:
    configs = resolve_camera_configs(camera)
    if not configs:
        raise ValueError("No camera configuration found.")
    if camera_name is None:
        return configs[0]
    for config in configs:
        if str(config.get("name", "")).strip() == camera_name:
            return config
    raise ValueError(f"Unknown camera name: {camera_name}")
