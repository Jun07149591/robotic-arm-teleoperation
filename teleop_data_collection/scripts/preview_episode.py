#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_ROOT = REPO_ROOT / "el_a3_sdk"
for _path in (str(REPO_ROOT), str(SDK_ROOT)):
    while _path in sys.path:
        sys.path.remove(_path)
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SDK_ROOT))

from teleop_data_collection.scripts.convert_to_lerobot import load_step_image_frames


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a side-by-side preview image for one episode.")
    parser.add_argument("episode_dir", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Default: <episode_dir>/preview.png",
    )
    return parser.parse_args()


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def _camera_names_from_meta(meta: dict[str, Any]) -> list[str]:
    cameras = meta.get("cameras")
    if isinstance(cameras, list):
        names = [
            str(camera.get("name", "")).strip()
            for camera in cameras
            if isinstance(camera, dict)
        ]
        names = [name for name in names if name]
        if names:
            return names
    return ["wrist", "side"]


def _first_step(episode_dir: Path) -> dict[str, Any] | None:
    steps = _load_jsonl(episode_dir / "steps.jsonl")
    return steps[0] if steps else None


def _raw_image_step(image_observation_keys: list[str]) -> dict[str, Any]:
    return {
        "camera_paths": {
            key: {"color": f"raw/{key}/step_000000_{key}_color.png"}
            for key in image_observation_keys
        }
    }


def build_preview_image(
    episode_dir: Path,
    step: dict[str, Any],
    image_observation_keys: list[str],
) -> np.ndarray | None:
    frames = load_step_image_frames(episode_dir, step, image_observation_keys)
    ordered_frames = [frames[key] for key in image_observation_keys if key in frames]
    if not ordered_frames:
        return None

    target_height = max(frame.shape[0] for frame in ordered_frames)
    resized = []
    for frame in ordered_frames:
        height, width = frame.shape[:2]
        if height != target_height:
            scale = target_height / float(height)
            frame = cv2.resize(frame, (int(round(width * scale)), target_height))
        resized.append(frame)

    spacer = np.full((target_height, 12, 3), 255, dtype=np.uint8)
    parts: list[np.ndarray] = []
    for index, frame in enumerate(resized):
        if index:
            parts.append(spacer)
        parts.append(frame)
    return np.concatenate(parts, axis=1)


def build_preview_image_from_episode_dir(episode_dir: Path) -> np.ndarray | None:
    meta = json.loads((episode_dir / "meta.json").read_text(encoding="utf-8"))
    image_observation_keys = _camera_names_from_meta(meta)
    step = _first_step(episode_dir)
    if step is not None:
        preview = build_preview_image(episode_dir, step, image_observation_keys)
        if preview is not None:
            return preview
    return build_preview_image(episode_dir, _raw_image_step(image_observation_keys), image_observation_keys)


def main() -> int:
    args = parse_args()
    preview = build_preview_image_from_episode_dir(args.episode_dir)
    if preview is None:
        print("No previewable camera frames found.", file=sys.stderr)
        return 1

    output_path = args.output or (args.episode_dir / "preview.png")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), cv2.cvtColor(preview, cv2.COLOR_RGB2BGR))
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
