#!/usr/bin/env python3
"""Record XBOX teleop episodes in LeRobot-compatible format."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_ROOT = REPO_ROOT / "el_a3_sdk"
for _path in (str(REPO_ROOT), str(SDK_ROOT)):
    while _path in sys.path:
        sys.path.remove(_path)
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SDK_ROOT))

from teleop_data_collection.lib.camera import camera_frame_dict, make_camera
from teleop_data_collection.lib.config import load_config
from teleop_data_collection.lib.episode import EpisodeWriter
from teleop_data_collection.lib.gamepad import read_gamepad_state
from teleop_data_collection.lib.keyboard import KeyboardStopWatcher
from teleop_data_collection.lib.robot import read_robot_state
from teleop_data_collection.lib.utils import ensure_dir, utc_now_iso
from teleop_data_collection.scripts.record_sdk_episode import (
    DEFAULT_CAMERA_NAME,
    _camera_depth_vis_max_m,
    _resolve_camera_configs,
    _resolve_episode_identity,
    _show_live_preview,
)


TELEOP_CONTROLLER = "el_a3_ros/el_a3_teleop/el_a3_teleop/xbox_teleop_node.py"
STATE_EXPORTER = "teleop_data_collection/scripts/export_ros_xbox_state.py"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record XBOX teleop episodes.")
    parser.add_argument(
        "--config", type=Path, default=Path("teleop_data_collection/configs/dataset_v1.yaml")
    )
    parser.add_argument("--camera-serial", default=None)
    parser.add_argument("--task", required=True)
    parser.add_argument("--operator", default=None)
    parser.add_argument("--dataset-root", type=Path, default=None)
    parser.add_argument("--repo-id", default="edulite_a3_xbox")
    parser.add_argument("--hz", type=float, default=15.0)
    parser.add_argument(
        "--fps",
        type=int,
        default=15,
        help="LeRobot/video FPS. Must match --hz so rows and video frames share one time base.",
    )
    parser.add_argument("--max-duration", type=float, default=0)
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--success", action="store_true", default=False)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--no-align", action="store_true")
    parser.add_argument("--save-pngs", action="store_true")
    parser.add_argument("--save-raw-images", action="store_true")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--preview-scale", type=float, default=0.5)
    parser.add_argument("--preview-window", default="EL-A3 XBOX teleop cameras")
    parser.add_argument(
        "--keyboard-stop-key",
        default="q",
        help='Terminal key that saves the current episode as success and stops collection (default: "q"; empty disables).',
    )
    parser.add_argument(
        "--keyboard-fail-key",
        default="f",
        help='Terminal key that saves the current episode as failure and stops collection (default: "f"; empty disables).',
    )
    parser.add_argument("--state-file", default="/tmp/robot_latest_state.json")
    parser.add_argument("--gamepad-file", default="/tmp/xbox_latest_input.json")
    parser.add_argument("--episode-id", default=None)
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="Keep recording new episodes after Start long-press; Ctrl+C stops the session.",
    )
    parser.add_argument(
        "--max-episodes",
        type=int,
        default=0,
        help="Maximum episodes to record in --continuous mode (0 = unlimited).",
    )
    parser.add_argument("--task-category", default=None)
    parser.add_argument("--session-id", default=None)
    parser.add_argument("--workspace-id", default=None)
    parser.add_argument("--scene-id", default=None)
    parser.add_argument("--notes", default=None)
    return parser.parse_args(argv)


def _joint_delta(qpos: list[float], prev_q: list[float] | None) -> list[float] | None:
    if prev_q is None:
        return None
    return [qpos[i] - prev_q[i] for i in range(min(len(qpos), len(prev_q)))]


def _ee_delta(ee_pose: dict[str, float], prev_ee: dict[str, float] | None) -> list[float] | None:
    if prev_ee is None:
        return None
    return [
        ee_pose["x"] - prev_ee["x"],
        ee_pose["y"] - prev_ee["y"],
        ee_pose["z"] - prev_ee["z"],
        ee_pose["rx"] - prev_ee["rx"],
        ee_pose["ry"] - prev_ee["ry"],
        ee_pose["rz"] - prev_ee["rz"],
    ]


def _validate_output_fps(args: argparse.Namespace) -> None:
    if abs(float(args.fps) - float(args.hz)) > 1e-6:
        raise ValueError(
            f"--fps must match --hz for one-to-one LeRobot row/video alignment "
            f"(got --hz {args.hz:g}, --fps {args.fps:g})."
        )


def _should_continue_after_episode(
    *,
    continuous: bool,
    saved_episodes: int,
    max_episodes: int,
    shutdown_requested: bool,
    episode_success: bool,
    continue_after_keyboard_failure: bool = False,
) -> bool:
    if not continuous:
        return False
    if shutdown_requested:
        return False
    if not episode_success and not continue_after_keyboard_failure:
        return False
    if max_episodes > 0 and saved_episodes >= max_episodes:
        return False
    return True


def _make_episode_writer(
    *,
    args: argparse.Namespace,
    dataset_cfg: dict[str, Any],
    robot_cfg: dict[str, Any],
    camera_cfg: dict[str, Any],
    camera_configs: list[dict[str, Any]],
    image_observation_keys: list[str],
    dataset_root: Path,
    episode_id: str,
    ep_idx: int,
    state_file: Path,
    gamepad_file: Path,
    save_raw_images: bool,
) -> EpisodeWriter:
    return EpisodeWriter(
        root=dataset_root,
        episode_id=episode_id,
        meta={
            "schema_version": "lerobot_v3",
            "dataset_name": args.repo_id,
            "dataset_version": "v3.0",
            "episode_index": ep_idx,
            "task_text": args.task,
            "task_category": args.task_category,
            "operator": args.operator,
            "session_id": args.session_id,
            "workspace_id": args.workspace_id,
            "scene_id": args.scene_id,
            "success": None,
            "failure_reason": None,
            "termination_reason": None,
            "notes": args.notes,
            "robot": robot_cfg,
            "camera": camera_cfg,
            "cameras": camera_configs,
            "controller": {
                "type": "xbox",
                "state_file": str(state_file),
                "gamepad_file": str(gamepad_file),
                "done_trigger": "Start long-press from export_ros_xbox_state.py",
            },
            "timing": {
                "sample_rate_hz": args.hz,
                "raw_camera_fps": dataset_cfg.get("raw_camera_fps"),
                "step_sample_rate_hz": dataset_cfg.get("step_sample_rate_hz"),
            },
            "storage": {
                "save_raw_images": save_raw_images,
                "save_raw_streams": bool(dataset_cfg.get("save_raw_streams", True)),
                "save_depth_png": save_raw_images,
                "live_preview": bool(args.preview),
            },
            "code": {
                "teleop_controller": TELEOP_CONTROLLER,
                "state_exporter": STATE_EXPORTER,
                "collection_script": "teleop_data_collection/scripts/record_xbox_episode.py",
            },
            "environment": {
                "camera_serial": args.camera_serial or camera_cfg.get("serial"),
                "camera_serials": {
                    cam["name"]: cam.get("serial") for cam in camera_configs
                },
                "state_file": str(state_file),
                "gamepad_file": str(gamepad_file),
                "timestamp": utc_now_iso(),
            },
            "quality": {},
            "labels": {},
        },
        fps=int(args.fps),
        image_observation_keys=image_observation_keys,
        controller_observation="xbox",
    )


def _wait_for_episode_rearm(
    *,
    state_file: Path,
    shutdown: threading.Event,
    poll_s: float = 0.05,
) -> None:
    while not shutdown.is_set():
        rs = read_robot_state(state_file)
        if not rs.valid or not rs.episode_done:
            return
        time.sleep(poll_s)


def _record_one_episode(
    *,
    args: argparse.Namespace,
    writer: EpisodeWriter,
    cameras: dict[str, Any],
    camera_cfg: dict[str, Any],
    camera_cfg_by_name: dict[str, dict[str, Any]],
    image_observation_keys: list[str],
    state_file: Path,
    gamepad_file: Path,
    save_raw_images: bool,
    shutdown: threading.Event,
    preview_enabled: bool,
    keyboard_stop: KeyboardStopWatcher,
) -> tuple[bool, int, bool, bool]:
    step_idx = 0
    prev_q = None
    prev_ee = None
    prev_episode_done = False
    arm_was_healthy = False
    start_wall_ns = time.time_ns()
    episode_success = args.success
    local_preview_enabled = preview_enabled
    continue_after_keyboard_failure = False

    try:
        while not shutdown.is_set():
            keyboard_action = keyboard_stop.poll_action()
            if keyboard_action is not None:
                episode_success = keyboard_action == "success"
                result = "success" if episode_success else "failure"
                print(f"\nKeyboard {result} requested. Ending XBOX episode.")
                if episode_success:
                    shutdown.set()
                else:
                    continue_after_keyboard_failure = True
                break

            elapsed_s = (time.time_ns() - start_wall_ns) / 1e9
            if args.max_duration > 0 and elapsed_s >= args.max_duration:
                print(f"\nReached max duration {args.max_duration}s, stopping.")
                break
            if args.max_steps > 0 and step_idx >= args.max_steps:
                print(f"\nReached max steps {args.max_steps}, stopping.")
                break

            loop_start = time.monotonic()
            capture_wall_ns = time.time_ns()
            frames = {
                name: camera.get_frame(timeout_ms=int(args.timeout_ms))
                for name, camera in cameras.items()
            }
            rs = read_robot_state(state_file)
            gamepad = read_gamepad_state(gamepad_file)
            if not rs.valid or not gamepad.valid:
                continue

            if not rs.episode_done and not arm_was_healthy:
                arm_was_healthy = True
            if rs.episode_done and not prev_episode_done and arm_was_healthy:
                print("\nStart long-press detected. Ending XBOX episode.")
                episode_success = True
                break
            prev_episode_done = rs.episode_done
            if not arm_was_healthy:
                continue

            image_frames = {}
            camera_artifacts: dict[str, dict[str, Any]] = {}
            for camera_name, frame in frames.items():
                image_frames[camera_name] = frame.color_rgb
                dst_rgb = None
                dst_depth = None
                dst_vis = None
                if save_raw_images:
                    cam_raw_dir = ensure_dir(writer.raw_dir / camera_name)
                    image_bundle = frame.save_images(
                        cam_raw_dir,
                        prefix=f"step_{step_idx:06d}_{camera_name}",
                        depth_vis_max_m=_camera_depth_vis_max_m(
                            camera_cfg_by_name[camera_name],
                            fallback=_camera_depth_vis_max_m(camera_cfg),
                        ),
                    )
                    dst_rgb = Path(image_bundle["color"])
                    dst_depth = Path(image_bundle["depth_raw"])
                    dst_vis = Path(image_bundle["depth_vis"])
                camera_artifacts[camera_name] = {
                    "frame": frame,
                    "rgb_path": dst_rgb,
                    "depth_path": dst_depth,
                    "depth_vis_path": dst_vis,
                    "saved_paths": {
                        "color": str(dst_rgb) if dst_rgb is not None else None,
                        "depth_raw": str(dst_depth) if dst_depth is not None else None,
                        "depth_vis": str(dst_vis) if dst_vis is not None else None,
                    },
                }

            if local_preview_enabled:
                try:
                    if _show_live_preview(
                        image_frames,
                        image_observation_keys,
                        scale=args.preview_scale,
                        window_name=args.preview_window,
                    ):
                        print("\nPreview window requested stop. Ending XBOX collection.")
                        episode_success = True
                        shutdown.set()
                        break
                except cv2.error as exc:
                    local_preview_enabled = False
                    print(f"\n[WARNING] Live preview disabled: {exc}")

            qpos = rs.qpos
            qvel = rs.qvel
            tau = rs.tau
            ee_pose = rs.ee_pose
            action_joint_delta = _joint_delta(qpos, prev_q)
            action_gripper_delta = (
                None
                if prev_q is None
                else (
                    qpos[6] - prev_q[6]
                    if len(qpos) > 6 and len(prev_q) > 6
                    else None
                )
            )

            primary_camera_name = (
                DEFAULT_CAMERA_NAME
                if DEFAULT_CAMERA_NAME in camera_artifacts
                else image_observation_keys[0]
            )
            primary_artifact = camera_artifacts[primary_camera_name]
            primary_frame = primary_artifact["frame"]

            step = {
                "image_frames": image_frames,
                "qpos": qpos,
                "qvel": qvel,
                "tau": tau,
                "ee_pose": [
                    ee_pose["x"],
                    ee_pose["y"],
                    ee_pose["z"],
                    ee_pose["rx"],
                    ee_pose["ry"],
                    ee_pose["rz"],
                ],
                "gripper_pos": qpos[6] if len(qpos) > 6 else 0.0,
                "gamepad_state": gamepad,
                "controller_raw": gamepad.raw,
                "action_joint_delta": action_joint_delta,
                "action_gripper_delta": action_gripper_delta,
                "done": 0,
                "success": 0,
                "step_idx": step_idx,
                "timestamp_ns": capture_wall_ns,
                "step_time_s": (capture_wall_ns - start_wall_ns) / 1e9,
                "frame_number": int(primary_frame.frame_number),
                "camera_timestamp_ms": float(primary_frame.timestamp_ms),
                "rgb_path": (
                    str(primary_artifact["rgb_path"])
                    if primary_artifact["rgb_path"] is not None
                    else None
                ),
                "depth_path": (
                    str(primary_artifact["depth_path"])
                    if primary_artifact["depth_path"] is not None
                    else None
                ),
                "rgb_vis_path": (
                    str(primary_artifact["depth_vis_path"])
                    if primary_artifact["depth_vis_path"] is not None
                    else None
                ),
                "camera_paths": {
                    name: artifact["saved_paths"]
                    for name, artifact in camera_artifacts.items()
                },
                "ee_pose_dict": ee_pose,
                "gripper_actual": qpos[6] if len(qpos) > 6 else None,
                "gripper_vel": qvel[6] if len(qvel) > 6 else None,
                "joint_enabled": rs.robot_status.get("joint_enabled"),
                "joint_faults": rs.robot_status.get("joint_faults"),
                "joint_mode_states": rs.robot_status.get("joint_mode_states"),
                "motor_states": rs.motor_states or {},
                "can": rs.can or {},
                "robot_status": rs.robot_status or {},
                "action_ee_delta": _ee_delta(ee_pose, prev_ee),
                "raw_camera": camera_frame_dict(primary_frame),
                "raw_cameras": {
                    name: camera_frame_dict(artifact["frame"])
                    for name, artifact in camera_artifacts.items()
                },
            }
            writer.add_step(step)

            prev_q = qpos
            prev_ee = ee_pose
            step_idx += 1

            elapsed = time.monotonic() - loop_start
            target = 1.0 / max(float(args.hz), 1e-6)
            if elapsed < target:
                time.sleep(target - elapsed)
    finally:
        writer.meta["success"] = bool(episode_success)
        writer.meta["termination_reason"] = "success" if episode_success else "stopped"
        writer.finalize()
    return episode_success, step_idx, local_preview_enabled, continue_after_keyboard_failure


def main() -> int:
    args = parse_args()
    _validate_output_fps(args)
    cfg = load_config(args.config)
    dataset_cfg = cfg.dataset
    robot_cfg = cfg.robot
    camera_cfg = cfg.camera
    camera_configs = _resolve_camera_configs(
        camera_cfg,
        camera_serial=args.camera_serial,
        no_align=args.no_align,
    )
    image_observation_keys = [cam["name"] for cam in camera_configs]

    dataset_root = args.dataset_root or Path(
        dataset_cfg.get("output_root", "teleop_data_collection/datasets")
    )
    dataset_root = dataset_root / args.repo_id
    state_file = Path(args.state_file)
    gamepad_file = Path(args.gamepad_file)
    save_raw_images = bool(
        args.save_raw_images
        or args.save_pngs
        or dataset_cfg.get("save_raw_images", False)
    )

    shutdown = threading.Event()

    def on_signal(_sig, _frame):
        shutdown.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    cameras = {cam_cfg["name"]: make_camera(cam_cfg) for cam_cfg in camera_configs}
    camera_cfg_by_name = {cam_cfg["name"]: cam_cfg for cam_cfg in camera_configs}
    preview_enabled = bool(args.preview)
    saved_episodes = 0

    keyboard_stop = KeyboardStopWatcher(
        args.keyboard_stop_key,
        fail_key=args.keyboard_fail_key,
    )
    with keyboard_stop, ExitStack() as stack:
        for camera in cameras.values():
            stack.enter_context(camera)
        for camera in cameras.values():
            camera.warmup(frame_count=int(args.warmup), timeout_ms=int(args.timeout_ms))
        try:
            while not shutdown.is_set():
                if args.episode_id and saved_episodes > 0:
                    raise ValueError("--episode-id cannot be used with multiple continuous episodes.")
                episode_id, ep_idx = _resolve_episode_identity(dataset_root, args.episode_id)
                writer = _make_episode_writer(
                    args=args,
                    dataset_cfg=dataset_cfg,
                    robot_cfg=robot_cfg,
                    camera_cfg=camera_cfg,
                    camera_configs=camera_configs,
                    image_observation_keys=image_observation_keys,
                    dataset_root=dataset_root,
                    episode_id=episode_id,
                    ep_idx=ep_idx,
                    state_file=state_file,
                    gamepad_file=gamepad_file,
                    save_raw_images=save_raw_images,
                )
                print(f"\nRecording XBOX episode {episode_id}...")
                (
                    episode_success,
                    step_idx,
                    preview_enabled,
                    continue_after_keyboard_failure,
                ) = _record_one_episode(
                    args=args,
                    writer=writer,
                    cameras=cameras,
                    camera_cfg=camera_cfg,
                    camera_cfg_by_name=camera_cfg_by_name,
                    image_observation_keys=image_observation_keys,
                    state_file=state_file,
                    gamepad_file=gamepad_file,
                    save_raw_images=save_raw_images,
                    shutdown=shutdown,
                    preview_enabled=preview_enabled,
                    keyboard_stop=keyboard_stop,
                )
                saved_episodes += 1 if step_idx > 0 else 0
                if step_idx > 0:
                    status = "success" if episode_success else "done"
                    print(f"Saved XBOX episode {episode_id} [{status}] to {dataset_root}")
                    print(f"  frames: {step_idx}")
                    print("  format: LeRobot v3.0 compatible")
                else:
                    print(f"Skipped empty XBOX episode {episode_id}; no frames were saved.")
                if not _should_continue_after_episode(
                    continuous=args.continuous,
                    saved_episodes=saved_episodes,
                    max_episodes=args.max_episodes,
                    shutdown_requested=shutdown.is_set(),
                    episode_success=episode_success,
                    continue_after_keyboard_failure=continue_after_keyboard_failure,
                ):
                    break
                if continue_after_keyboard_failure:
                    print("Prepare for the next episode.")
                else:
                    print("Release Start to arm the next episode.")
                    _wait_for_episode_rearm(state_file=state_file, shutdown=shutdown)
        finally:
            if args.preview:
                try:
                    cv2.destroyWindow(args.preview_window)
                except cv2.error:
                    pass
    if args.continuous:
        print(f"Continuous collection stopped after {saved_episodes} saved episode(s).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
