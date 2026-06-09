from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any

from el_a3_sdk import RealSenseD435


def camera_frame_dict(frame) -> dict[str, Any]:
    return {
        "timestamp_ms": float(frame.timestamp_ms),
        "frame_number": int(frame.frame_number),
        "depth_scale": float(frame.depth_scale),
        "depth_aligned_to_color": bool(frame.depth_aligned_to_color),
        "intrinsics": asdict(frame.intrinsics),
    }


def save_frame_images(frame, directory: Path, *, prefix: str, depth_vis_max_m: float) -> dict[str, str]:
    paths = frame.save_images(directory, prefix=prefix, depth_vis_max_m=depth_vis_max_m)
    return {name: str(path) for name, path in paths.items()}


def make_camera(config: dict[str, Any]) -> RealSenseD435:
    return RealSenseD435(
        width=int(config.get("width", 640)),
        height=int(config.get("height", 480)),
        fps=int(config.get("fps", 30)),
        serial=config.get("serial"),
        align_depth_to_color=bool(config.get("align_depth_to_color", True)),
        depth_width=config.get("depth_width"),
        depth_height=config.get("depth_height"),
    )

