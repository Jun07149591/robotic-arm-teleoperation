"""
LeRobot-compatible episode writer.

Outputs data in a layout that can be loaded by ``LeRobotDataset(root=...)``.

Directory layout per dataset root::

    datasets/<repo_id>/
      meta/
        info.json              # feature schema, fps, robot_type, codebase_version
        tasks.jsonl            # task text -> task_index mapping (one line per unique task)
      data/
        chunk-000/
          episode_000000.parquet
      videos/
        chunk-000/
          observation.images.front/
            episode_000000.mp4
          observation.images.depth/
            episode_000000.mp4
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from .gamepad import (
    GAMEPAD_OBSERVATION_DIM,
    XBOX_OBSERVATION_DIM,
    XBOX_OBSERVATION_NAMES,
    flatten_gamepad_observation,
    flatten_xbox_observation,
)
from .utils import ensure_dir, utc_now_iso

# ---------------------------------------------------------------------------
# Feature schema (matches typical LeRobot policy input / output)
# ---------------------------------------------------------------------------
# The number of joints is determined at runtime from the first step.
# Gripper is always the last element (index 6 for a 7-DoF arm + gripper).
DEFAULT_NUM_JOINTS = 7  # 6 arm joints + 1 gripper


def build_features(
    num_joints: int = DEFAULT_NUM_JOINTS,
    *,
    image_observation_keys: list[str] | tuple[str, ...] | None = None,
    image_shape: tuple[int, int, int] = (480, 640, 3),
    controller_observation: str = "pico",
) -> dict[str, dict]:
    """Return a LeRobot ``features`` dictionary for the default EDULITE A3 schema.

    Args:
        num_joints: Total number of joints including gripper (default 7).
    """
    state_dim = num_joints * 3 + 6 + 1  # qpos + qvel + tau + ee_pose(6) + gripper_pos
    state_names = []
    for suffix in ("qpos", "qvel", "tau"):
        for i in range(num_joints):
            state_names.append(f"{suffix}_j{i}")
    for axis in ("x", "y", "z", "rx", "ry", "rz"):
        state_names.append(f"ee_{axis}")
    state_names.append("gripper_pos")

    action_dim = num_joints  # joint delta + gripper delta
    action_names = [f"delta_j{i}" for i in range(action_dim)]

    pico_names = [
        "grip_x", "grip_y", "grip_z", "grip_rx", "grip_ry", "grip_rz", "grip_w",
        "btn_a", "btn_b", "btn_trigger", "btn_grip",
        "axis_x", "axis_y", "js_x", "js_y",
    ]
    gamepad_names = (
        [f"axis_{i}" for i in range(16)]
        + [f"button_{i}" for i in range(12)]
        + ["valid", "speed_level", "mode_normal", "episode_done"]
    )

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": state_names,
        },
        "action": {
            "dtype": "float32",
            "shape": (action_dim,),
            "names": action_names,
        },
        "timestamp": {"dtype": "float32", "shape": (1,), "names": None},
        "frame_index": {"dtype": "int64", "shape": (1,), "names": None},
        "episode_index": {"dtype": "int64", "shape": (1,), "names": None},
        "index": {"dtype": "int64", "shape": (1,), "names": None},
        "task_index": {"dtype": "int64", "shape": (1,), "names": None},
        "next.done": {"dtype": "bool", "shape": (1,), "names": None},
        "next.success": {"dtype": "bool", "shape": (1,), "names": None},
    }
    if controller_observation == "pico":
        features["observation.pico"] = {
            "dtype": "float32",
            "shape": (len(pico_names),),
            "names": pico_names,
        }
    elif controller_observation == "gamepad":
        features["observation.gamepad"] = {
            "dtype": "float32",
            "shape": (GAMEPAD_OBSERVATION_DIM,),
            "names": gamepad_names,
        }
    elif controller_observation == "xbox":
        features["observation.xbox"] = {
            "dtype": "float32",
            "shape": (XBOX_OBSERVATION_DIM,),
            "names": XBOX_OBSERVATION_NAMES,
        }
    elif controller_observation != "none":
        raise ValueError(
            "controller_observation must be one of: pico, xbox, gamepad, none"
        )
    for image_key in image_observation_keys or ("front", "depth"):
        features[f"observation.images.{image_key}"] = {
            "dtype": "video",
            "shape": image_shape,
            "names": ["height", "width", "channels"],
        }
    return features


# ---------------------------------------------------------------------------
# Video encoding helpers
# ---------------------------------------------------------------------------
def encode_mp4(
    frames: list[np.ndarray],
    output_path: Path,
    fps: int = 30,
    *,
    codec: str | None = None,
    quality: int = 8,
) -> Path:
    """Encode a list of RGB frames (H, W, 3) uint8 to an MP4 file.

    Uses ``imageio-ffmpeg`` (FFMPEG) under the hood.  Falls back to OpenCV's
    ``VideoWriter`` if imageio is unavailable.

    Args:
        frames: List of uint8 numpy arrays, each (H, W, 3) RGB.
        output_path: Destination ``.mp4`` file.
        fps: Frames per second.
        codec: FFMPEG codec name (e.g. ``"libx264"``).  Default: auto-select
               based on platform.
        quality: CRF value for libx264 (lower = better, 17-28 is typical).

    Returns:
        The output path.
    """
    if not frames:
        raise ValueError("No frames to encode")

    ensure_dir(output_path.parent)
    rgb_frames = [_ensure_uint8_rgb(f) for f in frames]

    # -- primary: imageio (ffmpeg) -------------------------------------------
    try:
        import imageio

        writer = imageio.get_writer(
            output_path,
            fps=int(fps),
            codec=codec or "libx264",
            quality=quality,
            macro_block_size=1,
            output_params=["-preset", "fast", "-crf", str(quality)],
        )
        for frame in rgb_frames:
            writer.append_data(frame)
        writer.close()
        return output_path
    except Exception:
        pass  # fall through to OpenCV fallback

    # -- fallback: OpenCV VideoWriter ---------------------------------------
    _encode_mp4_cv2(rgb_frames, output_path, fps)
    return output_path


def _encode_mp4_cv2(
    frames: list[np.ndarray],
    output_path: Path,
    fps: int,
) -> None:
    """OpenCV VideoWriter fallback — tries several FourCC codes."""
    h, w = frames[0].shape[:2]
    for codec in ("XVID", "mp4v", "MJPG", "avc1", "x264"):
        fourcc = cv2.VideoWriter_fourcc(*codec)
        out = cv2.VideoWriter(str(output_path), fourcc, float(fps), (w, h))
        if not out.isOpened():
            out.release()
            continue
        for frame in frames:
            rgb = np.asarray(frame, dtype=np.uint8)
            if rgb.shape[:2] != (h, w):
                rgb = cv2.resize(rgb, (w, h))
            out.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        out.release()
        return
    raise RuntimeError(f"Failed to open any VideoWriter codec for {output_path}")


# ---------------------------------------------------------------------------
# Core writer
# ---------------------------------------------------------------------------
class LeRobotEpisodeWriter:
    """Write a single episode into a LeRobot-compatible dataset directory.

    Usage::

        writer = LeRobotEpisodeWriter(
            root=Path("datasets/edulite_a3"),
            episode_index=0,
            task="pick up the cube",
            fps=15,
            robot_type="edulite_a3",
        )
        for step in steps:
            writer.add_step(step)
        writer.finalize()
    """

    def __init__(
        self,
        root: Path,
        episode_index: int,
        task: str,
        fps: int = 30,
        robot_type: str = "edulite_a3",
        codebase_version: str = "v3.0",
        features: dict[str, dict] | None = None,
        image_observation_keys: list[str] | tuple[str, ...] | None = None,
        controller_observation: str = "pico",
    ):
        self.root = Path(root)
        self.episode_index = int(episode_index)
        self.task = task
        self.fps = int(fps)
        self.robot_type = robot_type
        self.codebase_version = codebase_version
        self.image_observation_keys = list(image_observation_keys or ("front", "depth"))
        self.image_feature_keys = [
            f"observation.images.{key}" for key in self.image_observation_keys
        ]
        self.controller_observation = controller_observation
        self.controller_feature_key = (
            f"observation.{controller_observation}"
            if controller_observation in ("pico", "xbox", "gamepad")
            else None
        )
        self.features = features or build_features(
            image_observation_keys=self.image_observation_keys,
            controller_observation=controller_observation,
        )

        # Paths. The official LeRobot writer creates these lazily at finalize().
        self.meta_dir = self.root / "meta"
        self.data_dir = self.root / "data" / "chunk-000"
        episode_tag = f"episode_{self.episode_index:06d}"
        self._video_dirs = {
            feature_key: self.root / "videos" / feature_key / "chunk-000"
            for feature_key in self.image_feature_keys
        }
        self._video_relpaths = {
            feature_key: (
                Path("videos") / feature_key / "chunk-000" / f"{episode_tag}.mp4"
            ).as_posix()
            for feature_key in self.image_feature_keys
        }
        self.tasks_path = self.meta_dir / "tasks.jsonl"

        # Buffers
        self._frame_count = 0
        self._index_offset = self._compute_index_offset()
        self._tabular: list[dict[str, Any]] = []
        self._video_frames: dict[str, list[np.ndarray]] = {
            feature_key: [] for feature_key in self.image_feature_keys
        }
        self._finalized = False

        # Bookkeeping
        self._video_key_map = dict(self._video_dirs)

        # Official LeRobot v3 assigns task indices when the episode is saved.
        self._task_index = 0

    # ---- info.json ---------------------------------------------------------
    def _write_info(self) -> None:
        """Write ``meta/info.json`` if it doesn't exist, otherwise update counts."""
        info_path = self.meta_dir / "info.json"
        if info_path.exists():
            self._update_info()
            return

        info = {
            "codebase_version": self.codebase_version,
            "robot_type": self.robot_type,
            "total_episodes": 0,
            "total_frames": 0,
            "total_tasks": 0,
            "chunks_size": 100,
            "data_files_size_in_mb": 200,
            "video_files_size_in_mb": 200,
            "fps": self.fps,
            "splits": {"train": "0:0"},
            "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
            "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
            "features": self.features,
        }
        dump_json_pretty(info_path, info)

    def _resolve_task_index(self) -> int:
        """Ensure *task* appears in ``meta/tasks.jsonl``, return its 0-based index."""
        tasks = _load_tasks(self.tasks_path)
        existing = {t["task"]: t["task_index"] for t in tasks}
        if self.task not in existing:
            new_index = len(existing)
            tasks.append({"task": self.task, "task_index": new_index})
            _write_tasks(self.tasks_path, tasks)
            return new_index
        else:
            return existing[self.task]

    # ---- add_step ----------------------------------------------------------
    def add_step(self, step: dict[str, Any]) -> None:
        """Add one timestep.

        ``step`` is a dictionary with at least:
            - ``qpos``, ``qvel``, ``tau``: list[float]
            - ``ee_pose``: list[float] (6 elements)
            - ``gripper_pos``: float
            - ``image_frames``: dict[str, np.ndarray] keyed by image_observation_keys
            - legacy ``rgb_frame`` / ``depth_frame`` are accepted for front/depth
            - ``pico_pose``: dict/list | None (None -> zero-filled)
            - ``action_joint_delta``: list[float] | None
            - ``action_gripper_delta``: float | None
            - ``done``: int (0 or 1)
            - ``success``: int (0 or 1)
        """
        assert not self._finalized, "Cannot add_step after finalize()"

        ts = self._frame_count / max(self.fps, 1)

        # Build observation.state vector
        qpos = _pad_or_trim(_to_list(step.get("qpos", [])), DEFAULT_NUM_JOINTS)
        qvel = _pad_or_trim(_to_list(step.get("qvel", [])), DEFAULT_NUM_JOINTS)
        tau = _pad_or_trim(_to_list(step.get("tau", [])), DEFAULT_NUM_JOINTS)
        ee = _pad_or_trim(_to_list(step.get("ee_pose", [])), 6)
        gripper = float(step.get("gripper_pos", 0.0))
        state = np.array(qpos + qvel + tau + ee + [gripper], dtype=np.float32)

        controller_arr = None
        if self.controller_observation == "pico":
            controller_arr = _flatten_pico(
                step.get("pico_pose", None),
                step.get("pico_buttons", None),
                step.get("pico_axes", None),
            )
        elif self.controller_observation == "gamepad":
            controller_arr = flatten_gamepad_observation(
                step.get("gamepad_state", None)
            )
        elif self.controller_observation == "xbox":
            controller_arr = flatten_xbox_observation(
                step.get("gamepad_state", None)
            )

        # Action deltas
        action_joint = _to_list(step.get("action_joint_delta", None) or [])
        action_grip = float(step.get("action_gripper_delta", 0.0) or 0.0)
        if len(action_joint) >= DEFAULT_NUM_JOINTS:
            action_values = action_joint[:DEFAULT_NUM_JOINTS]
        else:
            action_values = action_joint + [action_grip]
        action_arr = np.array(
            _pad_or_trim(action_values, DEFAULT_NUM_JOINTS),
            dtype=np.float32,
        )

        done = int(step.get("done", 0))
        success = int(step.get("success", 0))

        # Tabular record
        row = {
            "observation.state": state,
            "action": action_arr,
            "next.done": bool(done),
            "next.success": bool(success),
            "timestamp": float(ts),
            "frame_index": int(self._frame_count),
            "episode_index": int(self.episode_index),
            "index": int(self._index_offset + self._frame_count),
            "task_index": int(self._task_index),
        }
        if self.controller_feature_key is not None and controller_arr is not None:
            row[self.controller_feature_key] = controller_arr
        for feature_key in self.image_feature_keys:
            row[feature_key] = {
                "path": self._video_relpaths[feature_key],
                "timestamp": float(ts),
            }
        self._tabular.append(row)

        # Video frames
        image_frames = self._extract_image_frames(step)
        for feature_key in self.image_feature_keys:
            frame = image_frames.get(feature_key)
            if frame is None:
                raise ValueError(f"Missing frame for {feature_key}")
            self._video_frames[feature_key].append(_ensure_uint8_rgb(frame))

        self._frame_count += 1

    # ---- finalize ----------------------------------------------------------
    def finalize(self, *, done: int = 1, success: int = 0) -> None:
        """Encode videos, write Parquet, and finalise metadata."""
        if self._finalized:
            return
        self._finalized = True

        # Guard: empty episode (no frames recorded)
        if self._frame_count == 0:
            return

        self._tabular[-1]["next.done"] = bool(done)
        self._tabular[-1]["next.success"] = bool(success)

        dataset = _open_official_lerobot_dataset(
            root=self.root,
            repo_id=self.root.name,
            fps=self.fps,
            robot_type=self.robot_type,
            features=self.features,
        )
        try:
            for i, row in enumerate(self._tabular):
                frame = {
                    "observation.state": row["observation.state"],
                    "action": row["action"],
                    "next.done": np.array([row["next.done"]], dtype=np.bool_),
                    "next.success": np.array([row["next.success"]], dtype=np.bool_),
                    "task": self.task,
                }
                if self.controller_feature_key is not None:
                    frame[self.controller_feature_key] = row[self.controller_feature_key]
                for feature_key in self.image_feature_keys:
                    frame[feature_key] = _resize_to_feature_shape(
                        self._video_frames[feature_key][i],
                        self.features[feature_key],
                    )
                dataset.add_frame(frame)
            dataset.save_episode(parallel_encoding=False)
        finally:
            dataset.finalize()

    def _extract_image_frames(self, step: dict[str, Any]) -> dict[str, Any]:
        image_frames: dict[str, Any] = {}
        raw_image_frames = step.get("image_frames")
        if isinstance(raw_image_frames, dict):
            for key, frame in raw_image_frames.items():
                key_str = str(key)
                feature_key = (
                    key_str
                    if key_str.startswith("observation.images.")
                    else f"observation.images.{key_str}"
                )
                image_frames[feature_key] = frame

        if "observation.images.front" not in image_frames and step.get("rgb_frame") is not None:
            image_frames["observation.images.front"] = step.get("rgb_frame")
        if "observation.images.depth" not in image_frames and step.get("depth_frame") is not None:
            image_frames["observation.images.depth"] = step.get("depth_frame")
        return image_frames

    def _write_episode_meta(
        self, parquet_path: Path, video_paths: dict[str, str]
    ) -> None:
        """Write a per-episode metadata file so LeRobot can discover the episode."""
        meta = {
            "episode_index": self.episode_index,
            "length": self._frame_count,
            "tasks": [self.task],
            "task_indices": [self._task_index],
            "data_path": str(parquet_path.relative_to(self.root)),
            "video_paths": video_paths,
            "timestamp": utc_now_iso(),
        }
        episodes_dir = ensure_dir(self.meta_dir / "episodes")
        dump_json_pretty(
            episodes_dir / f"episode_{self.episode_index:06d}.json",
            meta,
        )
        self._write_episodes_jsonl(meta)

    def _update_info(self) -> None:
        info_path = self.meta_dir / "info.json"
        if info_path.exists():
            info = json.loads(info_path.read_text(encoding="utf-8"))
        else:
            info = {}
        episodes = self._load_episode_metas()

        info["total_episodes"] = len(episodes)
        info["total_frames"] = sum(int(e.get("length", 0)) for e in episodes)
        info["total_tasks"] = len(_load_tasks(self.tasks_path))
        info["total_chunks"] = 1 if episodes else 0
        info["splits"] = {"train": f"0:{len(episodes)}"}
        # Refresh features in case they changed
        info["codebase_version"] = self.codebase_version
        info["robot_type"] = self.robot_type
        info["fps"] = self.fps
        info["features"] = self.features
        info["data_path"] = "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
        info["video_path"] = "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
        dump_json_pretty(info_path, info)

    def _compute_index_offset(self) -> int:
        offset = 0
        for meta in self._load_episode_metas():
            if int(meta.get("episode_index", -1)) < self.episode_index:
                offset += int(meta.get("length", 0))
        return offset

    def _load_episode_metas(self) -> list[dict[str, Any]]:
        episodes_jsonl = self.meta_dir / "episodes.jsonl"
        if episodes_jsonl.exists():
            episodes = []
            for line in episodes_jsonl.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if line:
                    episodes.append(json.loads(line))
            return sorted(episodes, key=lambda e: int(e.get("episode_index", -1)))

        episode_dir = self.meta_dir / "episodes"
        episodes = []
        if episode_dir.exists():
            for p in sorted(episode_dir.glob("episode_*.json")):
                episodes.append(json.loads(p.read_text(encoding="utf-8")))
        return episodes

    def _write_episodes_jsonl(self, meta: dict[str, Any]) -> None:
        episodes = {
            int(e.get("episode_index", -1)): {
                "episode_index": int(e.get("episode_index", -1)),
                "tasks": list(e.get("tasks", [])),
                "length": int(e.get("length", 0)),
            }
            for e in self._load_episode_metas()
            if int(e.get("episode_index", -1)) >= 0
        }
        episodes[self.episode_index] = {
            "episode_index": self.episode_index,
            "tasks": [self.task],
            "length": self._frame_count,
        }
        path = self.meta_dir / "episodes.jsonl"
        with path.open("w", encoding="utf-8") as f:
            for idx in sorted(episodes):
                f.write(json.dumps(episodes[idx], ensure_ascii=False, separators=(",", ":")) + "\n")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _to_list(val) -> list[float]:
    """Convert *val* to a list of floats, or return empty list."""
    if val is None:
        return []
    if isinstance(val, np.ndarray):
        return val.tolist()
    if isinstance(val, (list, tuple)):
        return [float(v) for v in val]
    return [float(val)]


def _pad_or_trim(values: list[float], size: int) -> list[float]:
    if len(values) >= size:
        return values[:size]
    return values + [0.0] * (size - len(values))


def _open_official_lerobot_dataset(
    *,
    root: Path,
    repo_id: str,
    fps: int,
    robot_type: str,
    features: dict[str, dict],
):
    """Open an official LeRobot v3 writer without deleting an existing root."""
    from lerobot.datasets.dataset_metadata import (
        CODEBASE_VERSION,
        INFO_PATH,
        LeRobotDatasetMetadata,
        _validate_feature_names,
        create_empty_dataset_info,
        write_json,
    )
    from lerobot.datasets.dataset_writer import DatasetWriter
    from lerobot.datasets.lerobot_dataset import LeRobotDataset, resolve_vcodec
    from lerobot.datasets.utils import DEFAULT_FEATURES

    root = Path(root)
    info_path = root / INFO_PATH
    user_features = {
        key: value
        for key, value in features.items()
        if key not in DEFAULT_FEATURES
    }

    if info_path.exists():
        info = json.loads(info_path.read_text(encoding="utf-8"))
        if info.get("codebase_version") != CODEBASE_VERSION:
            raise ValueError(
                f"Existing dataset at {root} is {info.get('codebase_version')}; "
                f"expected LeRobot {CODEBASE_VERSION}. Convert or move it before appending."
            )
        return LeRobotDataset.resume(
            repo_id=repo_id,
            root=root,
            vcodec="h264",
            video_backend="pyav",
        )

    if not root.exists():
        return LeRobotDataset.create(
            repo_id=repo_id,
            root=root,
            fps=fps,
            robot_type=robot_type,
            features=user_features,
            use_videos=True,
            vcodec="h264",
            video_backend="pyav",
        )

    root.mkdir(parents=True, exist_ok=True)
    metadata = LeRobotDatasetMetadata.__new__(LeRobotDatasetMetadata)
    metadata.repo_id = repo_id
    metadata._requested_root = root
    metadata.root = root
    metadata.tasks = None
    metadata.subtasks = None
    metadata.episodes = None
    metadata.stats = None
    merged_features = {**user_features, **DEFAULT_FEATURES}
    _validate_feature_names(merged_features)
    metadata.info = create_empty_dataset_info(
        CODEBASE_VERSION,
        fps,
        merged_features,
        use_videos=True,
        robot_type=robot_type,
    )
    write_json(metadata.info, metadata.root / INFO_PATH)
    metadata.revision = None
    metadata._pq_writer = None
    metadata.latest_episode = None
    metadata._metadata_buffer = []
    metadata._metadata_buffer_size = 10
    metadata._finalized = False

    dataset = LeRobotDataset.__new__(LeRobotDataset)
    dataset.repo_id = repo_id
    dataset._requested_root = root
    dataset.root = root
    dataset.revision = None
    dataset.tolerance_s = 1e-4
    dataset.image_transforms = None
    dataset.delta_timestamps = None
    dataset.episodes = None
    dataset._video_backend = "pyav"
    dataset._batch_encoding_size = 1
    dataset._vcodec = resolve_vcodec("h264")
    dataset._encoder_threads = None
    dataset.meta = metadata
    dataset.reader = None
    dataset.writer = DatasetWriter(
        meta=metadata,
        root=root,
        vcodec=dataset._vcodec,
        encoder_threads=None,
        batch_encoding_size=1,
    )
    dataset._is_finalized = False
    return dataset


def _ensure_uint8_rgb(frame: np.ndarray) -> np.ndarray:
    """Ensure frame is uint8 HWC RGB."""
    arr = np.asarray(frame)
    if arr.dtype != np.uint8:
        if arr.max() <= 1.0:
            arr = (arr * 255).astype(np.uint8)
        else:
            arr = arr.astype(np.uint8)
    if arr.ndim == 2:
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
    elif arr.shape[2] == 4:
        arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2RGB)
    elif arr.shape[2] == 1:
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
    return arr


def _resize_to_feature_shape(frame: np.ndarray, feature: dict[str, Any]) -> np.ndarray:
    arr = _ensure_uint8_rgb(frame)
    shape = tuple(feature.get("shape", ()))
    if len(shape) != 3:
        return arr
    height, width, channels = shape
    if channels != 3:
        return arr
    if arr.shape[:2] != (height, width):
        arr = cv2.resize(arr, (int(width), int(height)), interpolation=cv2.INTER_AREA)
    return arr


def _pack_column(values: list[Any]) -> pa.Array:
    """Pack a list of numpy arrays into a PyArrow array, preserving dtype."""
    if not values:
        return pa.array([])
    first = values[0]
    if isinstance(first, np.ndarray):
        if first.ndim == 0:
            return pa.array([v.item() for v in values])
        else:
            pa_dtype = pa.from_numpy_dtype(first.dtype)
            return pa.array([v.tolist() for v in values], type=pa.list_(pa_dtype))
    return pa.array(values)


def _flatten_pico(
    pico_pose: Any,
    pico_buttons: Any = None,
    pico_axes: Any = None,
) -> np.ndarray:
    values = [0.0] * 15

    if isinstance(pico_pose, dict):
        pos = pico_pose.get("position", pico_pose)
        ori = pico_pose.get("orientation", pico_pose)
        values[0] = float(pos.get("x", pico_pose.get("x", 0.0)) or 0.0)
        values[1] = float(pos.get("y", pico_pose.get("y", 0.0)) or 0.0)
        values[2] = float(pos.get("z", pico_pose.get("z", 0.0)) or 0.0)
        values[3] = float(ori.get("x", pico_pose.get("rx", 0.0)) or 0.0)
        values[4] = float(ori.get("y", pico_pose.get("ry", 0.0)) or 0.0)
        values[5] = float(ori.get("z", pico_pose.get("rz", 0.0)) or 0.0)
        values[6] = float(ori.get("w", pico_pose.get("qw", pico_pose.get("w", 0.0))) or 0.0)
    elif pico_pose is not None:
        flat = _to_list(pico_pose)
        for i, value in enumerate(flat[:7]):
            values[i] = float(value)

    buttons = []
    if isinstance(pico_buttons, dict):
        buttons = pico_buttons.get("right") or pico_buttons.get("left") or []
    elif isinstance(pico_buttons, list):
        buttons = pico_buttons
    for out_i, btn_i in enumerate((4, 5, 0, 1), start=7):
        if btn_i < len(buttons) and isinstance(buttons[btn_i], dict):
            values[out_i] = float(buttons[btn_i].get("value", 0.0) or 0.0)

    axes = []
    if isinstance(pico_axes, dict):
        axes = pico_axes.get("right") or pico_axes.get("left") or []
    elif isinstance(pico_axes, list):
        axes = pico_axes
    for out_i, axis_i in enumerate(range(4), start=11):
        if axis_i < len(axes):
            values[out_i] = float(axes[axis_i] or 0.0)

    return np.array(values, dtype=np.float32)


def dump_json_pretty(path: Path, payload: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return path


def _load_tasks(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    tasks = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            tasks.append(json.loads(line))
    return tasks


def _write_tasks(path: Path, tasks: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for t in tasks:
            f.write(json.dumps(t, ensure_ascii=False, separators=(",", ":")) + "\n")
