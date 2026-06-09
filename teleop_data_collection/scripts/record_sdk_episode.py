#!/usr/bin/env python3
"""Record SDK teleop episodes in LeRobot-compatible format.

Usage::

    python teleop_data_collection/scripts/record_sdk_episode.py \\
        --state-file /tmp/robot_latest_state.json \\
        --task "pick up the object and place it in the target area"

Output::

    datasets/<repo_id>/
      meta/
        info.json
        tasks.jsonl
        episodes/
      data/chunk-000/
        episode_000000.parquet
      videos/
        observation.images.wrist/chunk-000/
          episode_000000.mp4
        observation.images.side/chunk-000/
          episode_000000.mp4
      raw/                          # backward-compat JSONL logs
        camera_frames.jsonl
        robot_states.jsonl
        pico_packets.jsonl
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import signal
import ssl
import sys
import threading
import time
from pathlib import Path
from typing import Any
import urllib.error
import urllib.request

import cv2
import numpy as np
import pyarrow.parquet as pq

REPO_ROOT = Path(__file__).resolve().parents[2]

SDK_ROOT = REPO_ROOT / "el_a3_sdk"
for _path in (str(REPO_ROOT), str(SDK_ROOT)):
    while _path in sys.path:
        sys.path.remove(_path)
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SDK_ROOT))
_cached_sdk = sys.modules.get("el_a3_sdk")
if _cached_sdk is not None and getattr(_cached_sdk, "__file__", None) is None:
    sys.modules.pop("el_a3_sdk", None)

from teleop_data_collection.lib.camera import camera_frame_dict, make_camera
from teleop_data_collection.lib.config import load_config
from teleop_data_collection.lib.episode import EpisodeWriter
from teleop_data_collection.lib.keyboard import KeyboardStopWatcher
from teleop_data_collection.lib.pico import read_pico_pose
from teleop_data_collection.lib.robot import read_robot_state
from teleop_data_collection.lib.utils import ensure_dir, sha256_file, utc_now_iso


DEFAULT_CAMERA_NAME = "wrist"
TELEOP_CONTROLLER = "el_a3_sdk/demo/pico_control_jointctrl.py"


def _resolve_camera_configs(
    camera_cfg: dict[str, Any],
    *,
    camera_serial: str | None,
    no_align: bool,
) -> list[dict[str, Any]]:
    if isinstance(camera_cfg.get("cameras"), list):
        camera_configs = [dict(cam) for cam in camera_cfg["cameras"]]
    else:
        camera_configs = [dict(camera_cfg)]
        camera_configs[0].setdefault("name", camera_cfg.get("name", DEFAULT_CAMERA_NAME))
        if camera_serial:
            camera_configs[0]["serial"] = camera_serial

    names = [str(cam.get("name", "")).strip() for cam in camera_configs]
    if any(not name for name in names):
        raise ValueError("Every camera config must define a non-empty name.")
    if len(set(names)) != len(names):
        raise ValueError(f"Camera names must be unique: {names}")

    for cam in camera_configs:
        cam["name"] = str(cam["name"]).strip()
        if no_align:
            cam["align_depth_to_color"] = False
        else:
            cam["align_depth_to_color"] = bool(cam.get("align_depth_to_color", True))
    return camera_configs


def _camera_depth_vis_max_m(config: dict[str, Any], fallback: float = 1.0) -> float:
    return float(config.get("depth_vis_max_m", fallback))


def _build_live_preview_image(
    image_frames: dict[str, np.ndarray],
    camera_order: list[str],
    *,
    scale: float = 0.5,
) -> np.ndarray:
    ordered_frames = [
        image_frames[name]
        for name in camera_order
        if name in image_frames and image_frames[name] is not None
    ]
    if not ordered_frames:
        raise ValueError("No camera frames available for preview.")

    target_height = max(frame.shape[0] for frame in ordered_frames)
    resized: list[np.ndarray] = []
    for frame in ordered_frames:
        height, width = frame.shape[:2]
        if height != target_height:
            resize_scale = target_height / float(height)
            frame = cv2.resize(
                frame,
                (int(round(width * resize_scale)), target_height),
                interpolation=cv2.INTER_AREA,
            )
        resized.append(frame)

    spacer = np.full((target_height, 12, 3), 32, dtype=np.uint8)
    preview = np.concatenate(
        [part for index, frame in enumerate(resized) for part in ((spacer, frame) if index else (frame,))],
        axis=1,
    )

    scale = max(float(scale), 0.05)
    if abs(scale - 1.0) > 1e-6:
        height, width = preview.shape[:2]
        preview = cv2.resize(
            preview,
            (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
            interpolation=cv2.INTER_AREA,
        )
    return preview


def _show_live_preview(
    image_frames: dict[str, np.ndarray],
    camera_order: list[str],
    *,
    scale: float,
    window_name: str,
) -> bool:
    preview_rgb = _build_live_preview_image(image_frames, camera_order, scale=scale)
    preview_bgr = cv2.cvtColor(preview_rgb, cv2.COLOR_RGB2BGR)
    cv2.imshow(window_name, preview_bgr)
    key = cv2.waitKey(1) & 0xFF
    return key in (ord("q"), 27)


def _encode_preview_jpeg(preview_rgb: np.ndarray, *, quality: int = 70) -> bytes:
    quality = int(max(1, min(95, quality)))
    preview_bgr = cv2.cvtColor(preview_rgb, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(
        ".jpg",
        preview_bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), quality],
    )
    if not ok:
        raise RuntimeError("Failed to encode preview JPEG.")
    return encoded.tobytes()


def _post_headset_preview(
    payload: bytes,
    *,
    url: str,
    timeout_s: float = 0.05,
) -> None:
    request = urllib.request.Request(
        url,
        data=payload,
        headers={"Content-Type": "image/jpeg"},
        method="POST",
    )
    context = None
    if url.startswith("https://"):
        context = ssl._create_unverified_context()
    with urllib.request.urlopen(request, timeout=timeout_s, context=context) as response:
        response.read(1)


def _write_atomic_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(payload)
    tmp.replace(path)


def _resolve_episode_identity(dataset_root: Path, episode_id: str | None) -> tuple[str, int]:
    """Return (episode_id, episode_index) using zero-based LeRobot numbering."""
    next_idx = _next_episode_index(dataset_root)
    next_episode_id = f"episode_{next_idx:06d}"
    if episode_id:
        try:
            ep_idx = int(episode_id.split("_")[-1])
        except (ValueError, IndexError):
            ep_idx = next_idx
        if ep_idx != next_idx:
            raise ValueError(
                f"LeRobot v3 requires contiguous episode indices. "
                f"Requested {episode_id}, but next episode is {next_episode_id}."
            )
        _validate_raw_episode_dir_available(dataset_root, episode_id)
        return episode_id, ep_idx

    _validate_raw_episode_dir_available(dataset_root, next_episode_id)
    return next_episode_id, next_idx


def _next_episode_index(dataset_root: Path) -> int:
    info_path = dataset_root / "meta" / "info.json"
    if info_path.exists():
        try:
            info = json.loads(info_path.read_text(encoding="utf-8"))
            total_episodes = int(info.get("total_episodes", -1))
        except (json.JSONDecodeError, TypeError, ValueError):
            total_episodes = -1
        if total_episodes >= 0:
            return total_episodes

    indices: set[int] = set()
    episodes_jsonl = dataset_root / "meta" / "episodes.jsonl"
    if episodes_jsonl.exists():
        for line in episodes_jsonl.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                idx = int(json.loads(line).get("episode_index", -1))
            except (json.JSONDecodeError, TypeError, ValueError):
                idx = -1
            if idx >= 0:
                indices.add(idx)
    v3_episode_dir = dataset_root / "meta" / "episodes"
    if v3_episode_dir.exists():
        for parquet_path in v3_episode_dir.glob("chunk-*/*.parquet"):
            try:
                table = pq.read_table(parquet_path, columns=["episode_index"])
            except Exception:
                continue
            for idx in table["episode_index"].to_pylist():
                try:
                    indices.add(int(idx))
                except (TypeError, ValueError):
                    continue
    for base in (
        dataset_root / "meta" / "episodes",
        dataset_root / "data" / "chunk-000",
    ):
        if not base.exists():
            continue
        for p in base.glob("episode_*"):
            idx = _parse_episode_index(p.stem if p.is_file() else p.name)
            if idx is not None:
                indices.add(idx)
    return max(indices) + 1 if indices else 0


def _validate_raw_episode_dir_available(dataset_root: Path, episode_id: str) -> None:
    raw_episode_dir = dataset_root / episode_id
    if raw_episode_dir.exists() and any(raw_episode_dir.iterdir()):
        raise ValueError(
            f"Next LeRobot episode is {episode_id}, but raw directory already exists: "
            f"{raw_episode_dir}. This usually means raw episode folders and LeRobot "
            f"data/videos are out of sync. Use a new --repo-id or repair/move the "
            f"stale raw directory before appending."
        )


def _parse_episode_index(name: str) -> int | None:
    try:
        return int(name.split("_")[-1])
    except (ValueError, IndexError):
        return None


def _validate_output_fps(args: argparse.Namespace) -> None:
    if abs(float(args.fps) - float(args.hz)) > 1e-6:
        raise ValueError(
            f"--fps must match --hz for one-to-one LeRobot row/video alignment "
            f"(got --hz {args.hz:g}, --fps {args.fps:g})."
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record SDK teleop episodes (LeRobot format).")
    parser.add_argument(
        "--config", type=Path, default=Path("teleop_data_collection/configs/dataset_v1.yaml")
    )
    parser.add_argument("--camera-serial", default=None)
    parser.add_argument("--task", required=True, help="Natural-language task description.")
    parser.add_argument("--operator", default=None)
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=None,
        help="Root directory for the dataset (default: config output_root).",
    )
    parser.add_argument("--repo-id", default="edulite_a3", help="Dataset repository name.")
    parser.add_argument("--hz", type=float, default=15.0, help="Recording loop rate (Hz).")
    parser.add_argument(
        "--fps",
        type=int,
        default=15,
        help="LeRobot/video FPS. Must match --hz so rows and video frames share one time base.",
    )
    parser.add_argument("--max-duration", type=float, default=0,
                       help="Auto-stop after N seconds (0 = unlimited, stop with Ctrl+C).")
    parser.add_argument("--max-steps", type=int, default=0,
                       help="Auto-stop after N steps (0 = unlimited).")
    parser.add_argument("--success", action="store_true", default=False,
                       help="Mark this episode as success (default: not success).")
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--no-align", action="store_true")
    parser.add_argument("--save-point-cloud", action="store_true")
    parser.add_argument(
        "--save-pngs",
        action="store_true",
        help="Also save individual PNG frames (legacy alias for --save-raw-images).",
    )
    parser.add_argument(
        "--save-raw-images",
        action="store_true",
        help="Save per-step color/depth/depth_vis PNGs for debugging. Off by default to keep episodes small.",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show live wrist/side RGB camera preview during collection.",
    )
    parser.add_argument(
        "--preview-scale",
        type=float,
        default=0.5,
        help="Scale factor for the live preview window (default: 0.5).",
    )
    parser.add_argument(
        "--preview-window",
        default="EL-A3 teleop cameras",
        help="OpenCV window name for --preview.",
    )
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
    parser.add_argument(
        "--headset-preview",
        action="store_true",
        help="Stream low-rate wrist/side RGB preview back to the Pico WebXR headset page.",
    )
    parser.add_argument(
        "--headset-preview-url",
        default="https://127.0.0.1:8443/preview",
        help="Preview POST endpoint served by pico3_webxr_pose_receiver.py.",
    )
    parser.add_argument(
        "--headset-preview-fps",
        type=float,
        default=10.0,
        help="Maximum headset preview streaming rate in Hz (default: 10).",
    )
    parser.add_argument(
        "--headset-preview-scale",
        type=float,
        default=0.35,
        help="Scale factor for headset preview image before JPEG encoding (default: 0.35).",
    )
    parser.add_argument(
        "--headset-preview-quality",
        type=int,
        default=70,
        help="JPEG quality for headset preview stream, 1-95 (default: 70).",
    )
    parser.add_argument(
        "--headset-preview-file",
        type=Path,
        default=None,
        help="Write latest headset preview JPEG atomically to this file for an external relay process.",
    )
    parser.add_argument("--state-file", default="/tmp/robot_latest_state.json",
                       help=f"Path to robot state JSON exported by {TELEOP_CONTROLLER}")
    parser.add_argument("--episode-id", default=None)
    parser.add_argument("--task-category", default=None)
    parser.add_argument("--session-id", default=None)
    parser.add_argument("--workspace-id", default=None)
    parser.add_argument("--scene-id", default=None)
    parser.add_argument("--notes", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    _validate_output_fps(args)
    cfg = load_config(args.config)
    dataset_cfg = cfg.dataset
    robot_cfg = cfg.robot
    camera_cfg = cfg.camera
    teleop_cfg = cfg.teleop
    camera_configs = _resolve_camera_configs(
        camera_cfg,
        camera_serial=args.camera_serial,
        no_align=args.no_align,
    )
    image_observation_keys = [cam["name"] for cam in camera_configs]

    # ---- dataset root (LeRobot layout: datasets/<repo_id>/ ) ---------------
    dataset_root = args.dataset_root or Path(
        dataset_cfg.get("output_root", "teleop_data_collection/datasets")
    )
    dataset_root = dataset_root / args.repo_id

    # Determine zero-based LeRobot episode index.
    episode_id, ep_idx = _resolve_episode_identity(dataset_root, args.episode_id)
    episode_dir = dataset_root / episode_id

    # ---- robot state file (exported by the Pico JointCtrl controller) -------
    state_file = Path(args.state_file)
    save_raw_images = bool(
        args.save_raw_images
        or args.save_pngs
        or dataset_cfg.get("save_raw_images", False)
    )

    # ---- LeRobot EpisodeWriter (root = dataset level) --------------------
    writer = EpisodeWriter(
        root=dataset_root,          # shared dataset root
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
                "headset_preview": bool(args.headset_preview),
            },
            "code": {
                "teleop_receiver": "pico3_webxr_pose_receiver.py",
                "teleop_controller": TELEOP_CONTROLLER,
                "collection_script": "teleop_data_collection/scripts/record_sdk_episode.py",
            },
            "environment": {
                "camera_serial": args.camera_serial or camera_cfg.get("serial"),
                "camera_serials": {
                    cam["name"]: cam.get("serial") for cam in camera_configs
                },
                "state_file": str(state_file),
                "timestamp": utc_now_iso(),
            },
            "quality": {},
            "labels": {},
        },
        fps=int(args.fps),
        image_observation_keys=image_observation_keys,
    )

    # ---- signal handling ---------------------------------------------------
    shutdown = threading.Event()

    def on_signal(_sig, _frame):
        shutdown.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # ---- cameras ------------------------------------------------------------
    cameras = {
        cam_cfg["name"]: make_camera(cam_cfg)
        for cam_cfg in camera_configs
    }
    camera_cfg_by_name = {cam_cfg["name"]: cam_cfg for cam_cfg in camera_configs}

    pico_path = Path(teleop_cfg.get("pose_file", "/tmp/pico_latest_pose.json"))

    # ---- recording loop ----------------------------------------------------
    step_idx = 0
    prev_q = None
    prev_ee = None
    prev_episode_done = False   # rising-edge detection
    arm_was_healthy = False     # must see healthy state before allowing stop
    start_wall_ns = time.time_ns()
    last_headset_preview_at = 0.0
    headset_preview_warned = False

    keyboard_stop = KeyboardStopWatcher(
        args.keyboard_stop_key,
        fail_key=args.keyboard_fail_key,
    )
    with keyboard_stop, ExitStack() as stack:
        for camera in cameras.values():
            stack.enter_context(camera)
        for camera in cameras.values():
            camera.warmup(frame_count=int(args.warmup), timeout_ms=int(args.timeout_ms))
        preview_enabled = bool(args.preview)
        episode_success = args.success
        try:
            while not shutdown.is_set():
                keyboard_action = keyboard_stop.poll_action()
                if keyboard_action is not None:
                    episode_success = keyboard_action == "success"
                    result = "success" if episode_success else "failure"
                    print(f"\nKeyboard {result} requested. Ending episode.")
                    shutdown.set()
                    break

                # Auto-stop checks
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
                pico = read_pico_pose(pico_path)

                # ---- process images -------------------------------------------------
                image_frames: dict[str, np.ndarray] = {}
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
                        rgb_path = Path(image_bundle["color"])
                        depth_raw_path = Path(image_bundle["depth_raw"])
                        depth_vis_path = Path(image_bundle["depth_vis"])
                        dst_rgb = rgb_path
                        dst_depth = depth_raw_path
                        dst_vis = depth_vis_path

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

                if preview_enabled:
                    try:
                        if _show_live_preview(
                            image_frames,
                            image_observation_keys,
                            scale=args.preview_scale,
                            window_name=args.preview_window,
                        ):
                            print("\nPreview window requested stop. Ending episode.")
                            episode_success = True
                            shutdown.set()
                            break
                    except cv2.error as exc:
                        preview_enabled = False
                        print(f"\n[WARNING] Live preview disabled: {exc}")

                if args.headset_preview or args.headset_preview_file is not None:
                    now = time.monotonic()
                    interval = 1.0 / max(float(args.headset_preview_fps), 1e-6)
                    if now - last_headset_preview_at >= interval:
                        last_headset_preview_at = now
                        try:
                            preview_rgb = _build_live_preview_image(
                                image_frames,
                                image_observation_keys,
                                scale=args.headset_preview_scale,
                            )
                            payload = _encode_preview_jpeg(
                                preview_rgb,
                                quality=args.headset_preview_quality,
                            )
                            if args.headset_preview_file is not None:
                                _write_atomic_bytes(args.headset_preview_file, payload)
                            if args.headset_preview:
                                _post_headset_preview(
                                    payload,
                                    url=args.headset_preview_url,
                                )
                        except (OSError, RuntimeError, urllib.error.URLError, cv2.error) as exc:
                            if not headset_preview_warned:
                                headset_preview_warned = True
                                print(f"\n[WARNING] Headset preview stream disabled: {exc}")

                # ---- robot state (read from shared file, no CAN) -----------------
                rs = read_robot_state(state_file)
                if not rs.valid:
                    # State file not ready yet from the Pico JointCtrl controller
                    continue
                # Track whether arm was ever healthy during this episode
                if not rs.episode_done and not arm_was_healthy:
                    arm_was_healthy = True
                # Rising-edge: arm healthy→disabled transition
                if rs.episode_done and not prev_episode_done and arm_was_healthy:
                    print("\nA long-press detected — arm disabled. Ending episode.")
                    episode_success = True
                    break
                prev_episode_done = rs.episode_done
                # Skip frames while arm is still estop from last session
                if not arm_was_healthy:
                    continue
                qpos = rs.qpos
                qvel = rs.qvel
                tau = rs.tau
                ee_pose = rs.ee_pose
                robot_status = rs.robot_status
                motor_states = rs.motor_states or {}
                can_stats = rs.can or {}

                # ---- action deltas --------------------------------------------------
                action_joint_delta = (
                    None
                    if prev_q is None
                    else [
                        qpos[i] - prev_q[i] for i in range(min(len(qpos), len(prev_q)))
                    ]
                )
                action_ee_delta = None
                if prev_ee is not None:
                    action_ee_delta = [
                        ee_pose["x"] - prev_ee["x"],
                        ee_pose["y"] - prev_ee["y"],
                        ee_pose["z"] - prev_ee["z"],
                        ee_pose["rx"] - prev_ee["rx"],
                        ee_pose["ry"] - prev_ee["ry"],
                        ee_pose["rz"] - prev_ee["rz"],
                    ]
                action_gripper_delta = (
                    None
                    if prev_q is None
                    else (
                        qpos[6] - prev_q[6]
                        if len(qpos) > 6 and len(prev_q) > 6
                        else None
                    )
                )

                # ---- PICO teleop ----------------------------------------------------
                pico_pose = None
                pico_buttons = None
                pico_axes = None
                pico_viewer = None
                pico_raw = None
                pico_seq = None
                pico_received_at = None
                if pico is not None:
                    pico_raw = pico.raw
                    pico_seq = pico.seq
                    pico_received_at = pico.received_at
                    src = pico.source("right") or pico.source("left")
                    pico_pose = src.get("grip") if src else None
                    pico_buttons = {
                        "left": pico.buttons("left"),
                        "right": pico.buttons("right"),
                    }
                    pico_axes = {
                        "left": pico.axes("left"),
                        "right": pico.axes("right"),
                    }
                    pico_viewer = pico.raw.get("viewer")

                # ---- assemble step -------------------------------------------------
                primary_camera_name = (
                    DEFAULT_CAMERA_NAME
                    if DEFAULT_CAMERA_NAME in camera_artifacts
                    else image_observation_keys[0]
                )
                primary_artifact = camera_artifacts[primary_camera_name]
                primary_frame = primary_artifact["frame"]
                step: dict[str, Any] = {
                    # -- LeRobot video frames (numpy arrays) --
                    "image_frames": image_frames,
                    # -- LeRobot state --
                    "qpos": qpos,
                    "qvel": qvel,
                    "tau": tau,
                    "ee_pose": [ee_pose["x"], ee_pose["y"], ee_pose["z"],
                               ee_pose["rx"], ee_pose["ry"], ee_pose["rz"]],
                    "gripper_pos": qpos[6] if len(qpos) > 6 else 0.0,
                    "pico_pose": pico_pose,
                    "action_joint_delta": action_joint_delta,
                    "action_gripper_delta": action_gripper_delta,
                    "done": 0,      # will be set to 1 on the last frame
                    "success": 0,   # will be set on the last frame if --success
                    # -- debug / legacy fields --
                    "step_idx": step_idx,
                    "timestamp_ns": capture_wall_ns,
                    "step_time_s": (capture_wall_ns - start_wall_ns) / 1e9,
                    "frame_number": int(primary_frame.frame_number),
                    "camera_timestamp_ms": float(primary_frame.timestamp_ms),
                    "rgb_path": (
                        str(primary_artifact["rgb_path"])
                        if primary_artifact["rgb_path"] is not None else None
                    ),
                    "depth_path": (
                        str(primary_artifact["depth_path"])
                        if primary_artifact["depth_path"] is not None else None
                    ),
                    "rgb_vis_path": (
                        str(primary_artifact["depth_vis_path"])
                        if primary_artifact["depth_vis_path"] is not None else None
                    ),
                    "camera_paths": {
                        name: artifact["saved_paths"]
                        for name, artifact in camera_artifacts.items()
                    },
                    "ee_pose_dict": ee_pose,
                    "gripper_actual": qpos[6] if len(qpos) > 6 else None,
                    "gripper_vel": qvel[6] if len(qvel) > 6 else None,
                    "joint_enabled": robot_status.get("joint_enabled"),
                    "joint_faults": robot_status.get("joint_faults"),
                    "joint_mode_states": robot_status.get("joint_mode_states"),
                    "motor_states": motor_states,
                    "can": can_stats,
                    "robot_status": robot_status,
                    "action_ee_delta": action_ee_delta,
                    "pico_seq": pico_seq,
                    "pico_received_at": pico_received_at,
                    "pico_buttons": pico_buttons,
                    "pico_axes": pico_axes,
                    "pico_viewer": pico_viewer,
                    "pico_raw": pico_raw,
                    "raw_camera": camera_frame_dict(primary_frame),
                    "raw_cameras": {
                        name: camera_frame_dict(artifact["frame"])
                        for name, artifact in camera_artifacts.items()
                    },
                }

                # ---- optional point-cloud -------------------------------------------
                if args.save_point_cloud:
                    cloud = primary_frame.to_point_cloud(
                        max_depth_m=_camera_depth_vis_max_m(
                            camera_cfg_by_name[primary_camera_name],
                            fallback=_camera_depth_vis_max_m(camera_cfg),
                        ),
                        stride=1,
                        include_color=True,
                    )
                    cloud_path = cloud.save_npz(
                        writer.point_cloud_dir / f"step_{step_idx:06d}.npz"
                    )
                    step["point_cloud_npz_path"] = str(cloud_path)
                    step["point_cloud_npz_sha256"] = sha256_file(cloud_path)

                # ---- write step ----------------------------------------------------
                writer.add_step(step)

                prev_q = qpos
                prev_ee = ee_pose
                step_idx += 1

                # ---- rate-limit ----------------------------------------------------
                elapsed = time.monotonic() - loop_start
                target = 1.0 / max(float(args.hz), 1e-6)
                if elapsed < target:
                    time.sleep(target - elapsed)
        finally:
            if args.preview:
                try:
                    cv2.destroyWindow(args.preview_window)
                except cv2.error:
                    pass
            writer.meta["success"] = bool(episode_success)
            writer.meta["termination_reason"] = "success" if episode_success else "stopped"
            writer.finalize()

    if step_idx > 0:
        status = "success" if episode_success else "done"
        print(f"Saved episode {episode_id} [{status}] to {dataset_root}")
        print(f"  frames: {step_idx}")
        if args.max_duration > 0:
            print(f"  max-duration: {args.max_duration}s")
        print(f"  format: LeRobot v3.0 compatible")
        print(f"  raw logs: {episode_dir}/raw/")
    else:
        print(f"Skipped empty episode {episode_id}; no frames were saved.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
