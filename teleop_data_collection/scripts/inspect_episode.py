#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_ROOT = REPO_ROOT / "el_a3_sdk"
for _path in (str(REPO_ROOT), str(SDK_ROOT)):
    while _path in sys.path:
        sys.path.remove(_path)
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SDK_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect one teleop episode.")
    parser.add_argument("episode_dir", type=Path)
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
        names = [str(cam.get("name", "")).strip() for cam in cameras if isinstance(cam, dict)]
        return [name for name in names if name]
    camera = meta.get("camera")
    if isinstance(camera, dict):
        name = str(camera.get("name", "wrist")).strip() or "wrist"
        return [name]
    return []


def _summarize_lerobot_episode(episode_dir: Path, meta: dict[str, Any]) -> dict[str, Any]:
    camera_rows = _load_jsonl(episode_dir / "raw" / "camera_frames.jsonl")
    camera_names = _camera_names_from_meta(meta)
    if not camera_names and camera_rows:
        first_row = camera_rows[0]
        if isinstance(first_row, dict):
            camera_names = sorted(first_row.keys())

    frame_counts = {name: 0 for name in camera_names}
    latest_frames: dict[str, dict[str, Any]] = {}
    for row in camera_rows:
        if not isinstance(row, dict):
            continue
        for camera_name, frame_info in row.items():
            if not isinstance(frame_info, dict):
                continue
            frame_counts[camera_name] = frame_counts.get(camera_name, 0) + 1
            latest_frames[camera_name] = frame_info

    return {
        "episode_id": meta.get("episode_id", episode_dir.name),
        "num_steps": int(meta.get("num_steps", len(camera_rows))),
        "task_text": meta.get("task_text"),
        "success": meta.get("success"),
        "camera_names": camera_names,
        "camera_frame_counts": frame_counts,
        "latest_camera_frames": latest_frames,
    }


def _summarize_legacy_episode(episode_dir: Path, meta: dict[str, Any]) -> dict[str, Any]:
    steps = _load_jsonl(episode_dir / "steps.jsonl")
    return {
        "episode_id": meta.get("episode_id", episode_dir.name),
        "num_steps": len(steps),
        "task_text": meta.get("task_text"),
        "success": meta.get("success"),
        "camera_names": _camera_names_from_meta(meta) or ["front"],
    }


def main() -> int:
    args = parse_args()
    meta = json.loads((args.episode_dir / "meta.json").read_text(encoding="utf-8"))
    if (args.episode_dir / "raw" / "camera_frames.jsonl").exists():
        payload = _summarize_lerobot_episode(args.episode_dir, meta)
    else:
        payload = _summarize_legacy_episode(args.episode_dir, meta)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
