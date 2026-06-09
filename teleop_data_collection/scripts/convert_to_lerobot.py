#!/usr/bin/env python3
"""Convert existing JSONL+PNG episodes to LeRobot-compatible format.

Usage::

    python teleop_data_collection/scripts/convert_to_lerobot.py \\
        --input-root teleop_data_collection/datasets/teleop_vla \\
        --repo-id edulite_a3 \\
        --output-root teleop_data_collection/datasets/lerobot \\
        --fps 15
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_ROOT = REPO_ROOT / "el_a3_sdk"
for _path in (str(REPO_ROOT), str(SDK_ROOT)):
    while _path in sys.path:
        sys.path.remove(_path)
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SDK_ROOT))

from teleop_data_collection.lib.lerobot_writer import LeRobotEpisodeWriter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert legacy JSONL episodes to LeRobot format."
    )
    parser.add_argument(
        "--input-root",
        type=Path,
        required=True,
        help="Directory containing episode_XXXXXX subdirs.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("teleop_data_collection/datasets/lerobot"),
    )
    parser.add_argument("--repo-id", default="edulite_a3")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument(
        "--episodes",
        nargs="*",
        default=None,
        help="Specific episode dirs to convert (default: all).",
    )
    return parser.parse_args()


def load_step_image(episode_dir: Path, rel_path: str) -> np.ndarray | None:
    """Load an image from its relative path inside an episode directory."""
    p = episode_dir / rel_path
    if not p.exists():
        return None
    img = cv2.imread(str(p))
    if img is None:
        return None
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def load_step_image_frames(
    episode_dir: Path,
    step: dict[str, object],
    image_observation_keys: list[str],
) -> dict[str, np.ndarray]:
    frames: dict[str, np.ndarray] = {}
    camera_paths = step.get("camera_paths")
    if isinstance(camera_paths, dict):
        for image_key in image_observation_keys:
            image_meta = camera_paths.get(image_key)
            if isinstance(image_meta, dict):
                rel_path = image_meta.get("color")
                if isinstance(rel_path, str):
                    frame = load_step_image(episode_dir, rel_path)
                    if frame is not None:
                        frames[image_key] = frame

    if frames:
        return frames

    rgb_frame = load_step_image(episode_dir, str(step.get("rgb_path", "")))
    depth_frame = load_step_image(episode_dir, str(step.get("rgb_vis_path", "")))
    if rgb_frame is not None and image_observation_keys:
        frames[image_observation_keys[0]] = rgb_frame
    if depth_frame is not None and len(image_observation_keys) > 1:
        frames[image_observation_keys[1]] = depth_frame
    return frames


def convert_episode(
    episode_dir: Path,
    output_root: Path,
    episode_index: int,
    fps: int,
) -> int:
    """Convert one episode directory. Returns frame count."""
    steps_path = episode_dir / "steps.jsonl"
    meta_path = episode_dir / "meta.json"

    if not steps_path.exists():
        print(f"  [SKIP] {episode_dir.name} — no steps.jsonl")
        return 0

    # Load metadata
    task = "unknown"
    image_observation_keys = ["front", "depth"]
    if meta_path.exists():
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        task = meta.get("task_text", "unknown")
        cameras = meta.get("cameras")
        if isinstance(cameras, list):
            image_observation_keys = [
                str(camera.get("name", "")).strip()
                for camera in cameras
                if isinstance(camera, dict) and str(camera.get("name", "")).strip()
            ] or image_observation_keys

    # Load steps
    steps = []
    for line in steps_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            steps.append(json.loads(line))

    if not steps:
        print(f"  [SKIP] {episode_dir.name} — empty steps.jsonl")
        return 0

    writer = LeRobotEpisodeWriter(
        root=output_root,
        episode_index=episode_index,
        task=task,
        fps=fps,
        robot_type="edulite_a3",
        image_observation_keys=image_observation_keys,
    )

    frame_count = 0
    for s in steps:
        image_frames = load_step_image_frames(episode_dir, s, image_observation_keys)

        writer.add_step(
            {
                "qpos": s.get("qpos", []),
                "qvel": s.get("qvel", []),
                "tau": s.get("tau", []),
                "ee_pose": s.get("ee_pose", []),
                "gripper_pos": s.get("gripper_target", 0.0),
                "pico_pose": s.get("pico_pose"),
                "action_joint_delta": s.get("action_joint_delta"),
                "action_gripper_delta": s.get("action_gripper_delta"),
                "image_frames": image_frames,
                "done": 0,
                "success": 0,
            }
        )
        frame_count += 1

    writer.finalize()
    return frame_count


def main() -> int:
    args = parse_args()
    input_root = args.input_root
    output_root = args.output_root / args.repo_id

    if args.episodes:
        episode_dirs = [input_root / ep for ep in args.episodes]
    else:
        episode_dirs = sorted(
            [p for p in input_root.iterdir() if p.is_dir() and p.name.startswith("episode_")]
        )

    if not episode_dirs:
        print("No episodes found.")
        return 1

    total = 0
    for i, ep_dir in enumerate(episode_dirs):
        print(f"Converting {ep_dir.name} ...")
        n = convert_episode(ep_dir, output_root, episode_index=i, fps=int(args.fps))
        print(f"  -> {n} frames")
        total += n

    print(f"Done. {len(episode_dirs)} episodes, {total} frames total.")
    print(f"Output: {output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
