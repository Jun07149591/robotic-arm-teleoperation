"""
Episode writer — unified API that delegates to the LeRobot-compatible backend.

The public API (``add_step`` / ``finalize``) is kept compatible with the
original JSONL-based ``EpisodeWriter`` so existing collection scripts work
with minimal changes.
"""

from __future__ import annotations

from pathlib import Path
import shutil
from typing import Any

from .lerobot_writer import LeRobotEpisodeWriter
from .utils import append_jsonl, dump_json, ensure_dir


class EpisodeWriter:
    """Write one episode in LeRobot-compatible format while preserving raw logs.

    ``root`` is the **dataset root** (e.g. ``datasets/my_dataset/``) — shared
    ``meta/``, ``data/``, ``videos/`` live here.  Per-episode raw JSONL logs
    go into ``root / <episode_id> / raw/``.

    Usage::

        writer = EpisodeWriter(
            root=Path("datasets/my_dataset"),
            episode_id="episode_000001",
            meta={...},
        )
        for step in steps:
            writer.add_step(step)
        writer.finalize()
    """

    def __init__(
        self,
        root: Path,
        episode_id: str,
        meta: dict[str, Any],
        *,
        fps: int = 30,
        robot_type: str = "edulite_a3",
        image_observation_keys: list[str] | tuple[str, ...] | None = None,
        controller_observation: str = "pico",
    ):
        self.root = ensure_dir(root)          # dataset root (shared)
        self.episode_id = episode_id
        self.meta = dict(meta)
        self.fps = int(fps)
        self.robot_type = robot_type

        # Parse episode index from id (e.g., "episode_000001" → 1)
        try:
            ep_idx = int(episode_id.split("_")[-1])
        except (ValueError, IndexError):
            ep_idx = 0
        self._episode_index = ep_idx

        # Per-episode sub-directories (for raw logs + optional PNG backups)
        self._ep_dir = ensure_dir(self.root / episode_id)
        self.rgb_dir = ensure_dir(self._ep_dir / "rgb")
        self.depth_dir = ensure_dir(self._ep_dir / "depth")
        self.rgb_vis_dir = ensure_dir(self._ep_dir / "rgb_vis")
        self.point_cloud_dir = ensure_dir(self._ep_dir / "point_cloud")
        self.raw_dir = ensure_dir(self._ep_dir / "raw")

        # Task
        task = self.meta.get("task_text", "unknown")

        # LeRobot backend (shared dataset-level output)
        self._lerobot = LeRobotEpisodeWriter(
            root=self.root,
            episode_index=ep_idx,
            task=task,
            fps=fps,
            robot_type=robot_type,
            image_observation_keys=image_observation_keys,
            controller_observation=controller_observation,
        )

        # Raw JSONL log paths
        self._raw_camera_log = self.raw_dir / "camera_frames.jsonl"
        self._raw_robot_log = self.raw_dir / "robot_states.jsonl"
        self._raw_pico_log = self.raw_dir / "pico_packets.jsonl"
        self._raw_controller_log = self.raw_dir / "controller_inputs.jsonl"

        self.steps: list[dict[str, Any]] = []
        self._finalized = False

    def add_step(self, step: dict[str, Any]) -> None:
        """Record one step.

        ``step`` may contain the full original record (JSONL fields) plus
        numpy image arrays injected by the recording loop:

        - ``rgb_frame``: np.ndarray (H, W, 3) uint8
        - ``depth_frame``: np.ndarray (H, W, 3) uint8 (colour-mapped)
        """
        if self._finalized:
            raise RuntimeError("Cannot add_step after finalize")

        self.steps.append(step)

        # LeRobot backend
        self._lerobot.add_step(step)

        # Legacy JSONL logs (JSON-serializable subset)
        append_jsonl(
            self._raw_camera_log,
            step.get("raw_cameras") or step.get("raw_camera", {}),
        )
        append_jsonl(
            self._raw_robot_log,
            {
                "timestamp_ns": step.get("timestamp_ns"),
                "qpos": step.get("qpos"),
                "qvel": step.get("qvel"),
                "tau": step.get("tau"),
                "ee_pose": step.get("ee_pose"),
                "robot_status": step.get("robot_status"),
            },
        )
        pico_raw = step.get("pico_raw")
        if pico_raw is not None:
            append_jsonl(self._raw_pico_log, pico_raw)
        controller_raw = step.get("controller_raw")
        if controller_raw is not None:
            append_jsonl(self._raw_controller_log, controller_raw)

    def finalize(self) -> None:
        """Encode videos, write Parquet, and persist metadata."""
        if self._finalized:
            return
        self._finalized = True

        if len(self.steps) == 0:
            print("  [WARNING] Episode has no frames — nothing saved.")
            shutil.rmtree(self._ep_dir, ignore_errors=True)
            return

        # Set termination flags from meta
        done = True
        success = bool(self.meta.get("success"))

        self._lerobot.finalize(done=done, success=success)

        # Backward-compat meta.json (per-episode)
        meta_out = dict(self.meta)
        meta_out["episode_id"] = self.episode_id
        meta_out["num_steps"] = len(self.steps)
        from .utils import utc_now_iso

        meta_out["finalized_at"] = utc_now_iso()
        meta_out["lerobot_format"] = True
        dump_json(self._ep_dir / "meta.json", meta_out)
