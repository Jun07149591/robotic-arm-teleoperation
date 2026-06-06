from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from contextlib import redirect_stdout
import io

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from teleop_data_collection.lib.config import resolve_camera_config
from teleop_data_collection.lib.lerobot_writer import LeRobotEpisodeWriter, build_features
from teleop_data_collection.lib.robot import read_robot_state
from teleop_data_collection.lib.utils import dump_json
from teleop_data_collection.scripts.convert_to_lerobot import (
    convert_episode,
    load_step_image_frames,
)
from teleop_data_collection.scripts.preview_episode import (
    build_preview_image,
    build_preview_image_from_episode_dir,
)
from teleop_data_collection.scripts.record_sdk_episode import (
    TELEOP_CONTROLLER,
    _build_live_preview_image,
    _encode_preview_jpeg,
    _resolve_camera_configs,
    _resolve_episode_identity,
)
from teleop_data_collection.scripts.inspect_episode import main as inspect_episode_main


REPO_ROOT = Path(__file__).resolve().parents[2]
os.environ.setdefault("HF_HOME", "/tmp/hf_home_lerobot_tests")
os.environ.setdefault("HF_DATASETS_CACHE", "/tmp/hf_datasets_lerobot_tests")


class LeRobotCompatibilityTest(unittest.TestCase):
    def test_recording_metadata_uses_jointctrl_controller(self) -> None:
        self.assertEqual(TELEOP_CONTROLLER, "el_a3_sdk/demo/pico_control_jointctrl.py")

    def test_writer_outputs_matching_rows_frames_and_scalar_columns(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "edulite_a3"
            writer = LeRobotEpisodeWriter(
                root=root,
                episode_index=0,
                task="pick book",
                fps=15,
                robot_type="edulite_a3",
            )
            for i in range(3):
                frame = np.full((32, 48, 3), i * 40, dtype=np.uint8)
                writer.add_step(
                    {
                        "qpos": [float(i)] * 7,
                        "qvel": [0.1] * 7,
                        "tau": [0.2] * 7,
                        "ee_pose": [0.0, 1.0, 2.0, 0.1, 0.2, 0.3],
                        "gripper_pos": 0.5,
                        "pico_pose": {
                            "position": {"x": 1, "y": 2, "z": 3},
                            "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.4},
                        },
                        "pico_buttons": {"right": [{"value": 0}, {"value": 1}, {}, {}, {"value": 1}, {"value": 0}]},
                        "pico_axes": {"right": [0.1, 0.2, 0.3, 0.4]},
                        "action_joint_delta": [0.0] * 7,
                        "action_gripper_delta": 0.0,
                        "rgb_frame": frame,
                        "depth_frame": frame,
                    }
                )
            writer.finalize(done=True, success=True)

            parquet_path = root / "data" / "chunk-000" / "file-000.parquet"
            table = pq.read_table(parquet_path)
            self.assertEqual(table.num_rows, 3)

            for column in ("timestamp", "frame_index", "episode_index", "index", "task_index", "next.done", "next.success"):
                self.assertFalse(
                    table.schema.field(column).type.__class__.__name__.startswith("List"),
                    f"{column} must be a scalar column",
                )

            features = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))["features"]
            self.assertEqual(features["observation.state"]["shape"][0], 28)
            self.assertEqual(len(table["observation.state"][0].as_py()), 28)
            self.assertEqual(features["observation.pico"]["shape"][0], 15)
            self.assertEqual(len(table["observation.pico"][0].as_py()), 15)
            self.assertTrue(table["next.done"][-1].as_py())
            self.assertTrue(table["next.success"][-1].as_py())

            front_video = root / "videos" / "observation.images.front" / "chunk-000" / "file-000.mp4"
            cap = cv2.VideoCapture(str(front_video))
            try:
                self.assertEqual(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), table.num_rows)
            finally:
                cap.release()

            info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
            self.assertEqual(info["codebase_version"], "v3.0")
            self.assertEqual(info["total_episodes"], 1)
            self.assertEqual(info["total_frames"], 3)
            self.assertEqual(
                info["data_path"],
                "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
            )
            self.assertEqual(
                info["video_path"],
                "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
            )
            self.assertTrue((root / "meta" / "tasks.parquet").exists())
            self.assertTrue((root / "meta" / "episodes" / "chunk-000" / "file-000.parquet").exists())
            self.assertTrue((root / "meta" / "stats.json").exists())

            from lerobot.datasets.lerobot_dataset import LeRobotDataset

            dataset = LeRobotDataset("local/edulite_a3_test", root=root, video_backend="pyav")
            self.assertEqual(len(dataset), 3)
            self.assertEqual(dataset[0]["observation.state"].shape[0], 28)
            self.assertTrue(bool(dataset[2]["next.done"]))

    def test_build_features_state_names_match_shape(self) -> None:
        features = build_features()
        self.assertEqual(len(features["observation.state"]["names"]), features["observation.state"]["shape"][0])
        self.assertEqual(len(features["observation.pico"]["names"]), features["observation.pico"]["shape"][0])
        self.assertEqual(len(features["action"]["names"]), features["action"]["shape"][0])

    def test_writer_supports_two_rgb_camera_features_without_depth_feature(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "edulite_a3_two_cam"
            writer = LeRobotEpisodeWriter(
                root=root,
                episode_index=0,
                task="pick book",
                fps=15,
                robot_type="edulite_a3",
                image_observation_keys=["wrist", "side"],
            )
            wrist_frame = np.full((32, 48, 3), 60, dtype=np.uint8)
            side_frame = np.full((32, 48, 3), 120, dtype=np.uint8)
            writer.add_step(
                {
                    "qpos": [0.0] * 7,
                    "qvel": [0.0] * 7,
                    "tau": [0.0] * 7,
                    "ee_pose": [0.0] * 6,
                    "gripper_pos": 0.0,
                    "pico_pose": None,
                    "action_joint_delta": [0.0] * 7,
                    "action_gripper_delta": 0.0,
                    "image_frames": {
                        "wrist": wrist_frame,
                        "side": side_frame,
                    },
                }
            )
            writer.finalize(done=True, success=True)

            info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
            features = info["features"]
            self.assertIn("observation.images.wrist", features)
            self.assertIn("observation.images.side", features)
            self.assertNotIn("observation.images.depth", features)

            for key in ("wrist", "side"):
                video_path = root / "videos" / f"observation.images.{key}" / "chunk-000" / "file-000.mp4"
                self.assertTrue(video_path.exists(), video_path)

            from lerobot.datasets.lerobot_dataset import LeRobotDataset

            dataset = LeRobotDataset("local/edulite_a3_two_cam_test", root=root, video_backend="pyav")
            sample = dataset[0]
            self.assertIn("observation.images.wrist", sample)
            self.assertIn("observation.images.side", sample)
            self.assertNotIn("observation.images.depth", sample)

    def test_stale_robot_state_is_invalid(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "robot_latest_state.json"
            path.write_text(
                json.dumps(
                    {
                        "timestamp_ns": time.time_ns() - int(5e9),
                        "qpos": [1.0] * 7,
                    }
                ),
                encoding="utf-8",
            )
            self.assertFalse(read_robot_state(path).valid)

    def test_resolve_episode_identity_is_zero_based_and_uses_lerobot_meta(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assertEqual(_resolve_episode_identity(root, None), ("episode_000000", 0))
            episodes = root / "meta" / "episodes"
            episodes.mkdir(parents=True)
            (episodes / "episode_000000.json").write_text("{}", encoding="utf-8")
            (episodes / "episode_000002.json").write_text("{}", encoding="utf-8")
            self.assertEqual(_resolve_episode_identity(root, None), ("episode_000003", 3))
            (root / "meta" / "episodes.jsonl").write_text(
                '{"episode_index":4,"tasks":["x"],"length":1}\n',
                encoding="utf-8",
            )
            self.assertEqual(_resolve_episode_identity(root, None), ("episode_000005", 5))
            v3_dir = root / "meta" / "episodes" / "chunk-000"
            v3_dir.mkdir(parents=True, exist_ok=True)
            pq.write_table(pa.table({"episode_index": [7], "length": [1]}), v3_dir / "file-000.parquet")
            self.assertEqual(_resolve_episode_identity(root, None), ("episode_000008", 8))
            self.assertEqual(_resolve_episode_identity(root, "episode_000008"), ("episode_000008", 8))
            with self.assertRaises(ValueError):
                _resolve_episode_identity(root, "episode_000009")

    def test_convert_script_help_runs_from_outside_repo(self) -> None:
        script = REPO_ROOT / "teleop_data_collection" / "scripts" / "convert_to_lerobot.py"
        result = subprocess.run(
            [sys.executable, str(script), "--help"],
            cwd="/tmp",
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_realsense_script_help_runs_from_outside_repo(self) -> None:
        script_names = [
            "check_realsense.py",
            "export_realsense_intrinsics.py",
            "export_realsense_extrinsic_template.py",
            "check_d405.py",
        ]
        for script_name in script_names:
            script = REPO_ROOT / "teleop_data_collection" / "scripts" / script_name
            result = subprocess.run(
                [sys.executable, str(script), "--help"],
                cwd="/tmp",
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(result.returncode, 0, f"{script_name}: {result.stderr}")

    def test_resolve_camera_configs_prefers_named_multi_camera_setup(self) -> None:
        cfg = {
            "serial": "legacy_serial",
            "cameras": [
                {"name": "wrist", "serial": "260322277792", "width": 640, "height": 480},
                {"name": "side", "serial": "260722303162", "width": 640, "height": 480},
            ],
        }
        cameras = _resolve_camera_configs(cfg, camera_serial=None, no_align=False)
        self.assertEqual([cam["name"] for cam in cameras], ["wrist", "side"])
        self.assertEqual(
            {cam["name"]: cam["serial"] for cam in cameras},
            {"wrist": "260322277792", "side": "260722303162"},
        )

    def test_resolve_camera_config_selects_named_camera_from_multi_camera_config(self) -> None:
        camera = resolve_camera_config(
            {
                "cameras": [
                    {"name": "wrist", "serial": "260322277792"},
                    {"name": "side", "serial": "260722303162"},
                ]
            },
            camera_name="side",
        )
        self.assertEqual(camera["name"], "side")
        self.assertEqual(camera["serial"], "260722303162")

    def test_inspect_episode_reports_multi_camera_summary_from_lerobot_episode_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            episode_dir = Path(tmp) / "episode_000000"
            raw_dir = episode_dir / "raw"
            raw_dir.mkdir(parents=True)
            dump_json(
                episode_dir / "meta.json",
                {
                    "episode_id": "episode_000000",
                    "task_text": "pick book",
                    "success": True,
                    "cameras": [
                        {"name": "wrist", "serial": "260322277792"},
                        {"name": "side", "serial": "260722303162"},
                    ],
                },
            )
            (raw_dir / "camera_frames.jsonl").write_text(
                "\n".join(
                    [
                        json.dumps(
                            {
                                "wrist": {"frame_number": 1, "timestamp_ms": 10.0},
                                "side": {"frame_number": 3, "timestamp_ms": 12.0},
                            }
                        ),
                        json.dumps(
                            {
                                "wrist": {"frame_number": 2, "timestamp_ms": 20.0},
                                "side": {"frame_number": 4, "timestamp_ms": 22.0},
                            }
                        ),
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            argv_backup = sys.argv
            sys.argv = ["inspect_episode.py", str(episode_dir)]
            stdout = io.StringIO()
            try:
                with redirect_stdout(stdout):
                    exit_code = inspect_episode_main()
            finally:
                sys.argv = argv_backup

            self.assertEqual(exit_code, 0)
            payload = json.loads(stdout.getvalue())
            self.assertEqual(payload["episode_id"], "episode_000000")
            self.assertEqual(payload["num_steps"], 2)
            self.assertEqual(payload["camera_names"], ["wrist", "side"])
            self.assertEqual(payload["camera_frame_counts"], {"wrist": 2, "side": 2})

    def test_load_step_image_frames_reads_dual_camera_paths(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            episode_dir = Path(tmp) / "episode_000001"
            episode_dir.mkdir()
            wrist_path = episode_dir / "wrist.png"
            side_path = episode_dir / "side.png"
            cv2.imwrite(str(wrist_path), np.full((8, 8, 3), 32, dtype=np.uint8))
            cv2.imwrite(str(side_path), np.full((8, 8, 3), 96, dtype=np.uint8))

            frames = load_step_image_frames(
                episode_dir,
                {
                    "camera_paths": {
                        "wrist": {"color": "wrist.png"},
                        "side": {"color": "side.png"},
                    }
                },
                ["wrist", "side"],
            )

            self.assertEqual(sorted(frames.keys()), ["side", "wrist"])
            self.assertEqual(frames["wrist"].shape, (8, 8, 3))
            self.assertEqual(int(frames["side"][0, 0, 0]), 96)

    def test_convert_episode_writes_dual_camera_dataset_at_output_root(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            episode_dir = root / "episode_000123"
            episode_dir.mkdir()
            dump_json(
                episode_dir / "meta.json",
                {
                    "task_text": "pick book",
                    "cameras": [
                        {"name": "wrist", "serial": "260322277792"},
                        {"name": "side", "serial": "260722303162"},
                    ],
                },
            )
            wrist_path = episode_dir / "wrist.png"
            side_path = episode_dir / "side.png"
            cv2.imwrite(str(wrist_path), np.full((8, 8, 3), 32, dtype=np.uint8))
            cv2.imwrite(str(side_path), np.full((8, 8, 3), 96, dtype=np.uint8))
            (episode_dir / "steps.jsonl").write_text(
                json.dumps(
                    {
                        "qpos": [0.0] * 7,
                        "qvel": [0.0] * 7,
                        "tau": [0.0] * 7,
                        "ee_pose": [0.0] * 6,
                        "camera_paths": {
                            "wrist": {"color": "wrist.png"},
                            "side": {"color": "side.png"},
                        },
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            output_root = root / "converted"
            count = convert_episode(episode_dir, output_root, episode_index=0, fps=15)

            self.assertEqual(count, 1)
            self.assertTrue((output_root / "videos" / "observation.images.wrist").exists())
            self.assertTrue((output_root / "videos" / "observation.images.side").exists())
            self.assertFalse((output_root / "lerobot_output").exists())

    def test_build_preview_image_combines_wrist_and_side_frames(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            episode_dir = Path(tmp) / "episode_000001"
            episode_dir.mkdir()
            wrist = np.full((20, 30, 3), 20, dtype=np.uint8)
            side = np.full((10, 15, 3), 120, dtype=np.uint8)
            cv2.imwrite(str(episode_dir / "wrist.png"), cv2.cvtColor(wrist, cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(episode_dir / "side.png"), cv2.cvtColor(side, cv2.COLOR_RGB2BGR))

            preview = build_preview_image(
                episode_dir,
                {
                    "camera_paths": {
                        "wrist": {"color": "wrist.png"},
                        "side": {"color": "side.png"},
                    }
                },
                ["wrist", "side"],
            )

            self.assertIsNotNone(preview)
            assert preview is not None
            self.assertEqual(preview.shape[0], 20)
            self.assertGreater(preview.shape[1], 30)

    def test_build_live_preview_image_combines_camera_frames(self) -> None:
        wrist = np.full((20, 30, 3), 20, dtype=np.uint8)
        side = np.full((10, 15, 3), 120, dtype=np.uint8)

        preview = _build_live_preview_image(
            {"wrist": wrist, "side": side},
            ["wrist", "side"],
            scale=0.5,
        )

        self.assertEqual(preview.shape[0], 10)
        self.assertGreater(preview.shape[1], 20)
        self.assertTrue(np.any(preview[:, :10] == 20))
        self.assertTrue(np.any(preview[:, -10:] == 120))

    def test_encode_preview_jpeg_outputs_jpeg_bytes(self) -> None:
        frame = np.full((20, 30, 3), 80, dtype=np.uint8)

        payload = _encode_preview_jpeg(frame, quality=70)

        self.assertGreater(len(payload), 100)
        self.assertEqual(payload[:2], b"\xff\xd8")
        self.assertEqual(payload[-2:], b"\xff\xd9")

    def test_build_preview_image_from_lerobot_episode_dir_reads_raw_camera_images(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            episode_dir = Path(tmp) / "episode_000001"
            (episode_dir / "raw" / "wrist").mkdir(parents=True)
            (episode_dir / "raw" / "side").mkdir(parents=True)
            dump_json(
                episode_dir / "meta.json",
                {
                    "episode_id": "episode_000001",
                    "cameras": [{"name": "wrist"}, {"name": "side"}],
                },
            )
            wrist = np.full((20, 30, 3), 20, dtype=np.uint8)
            side = np.full((20, 25, 3), 120, dtype=np.uint8)
            cv2.imwrite(
                str(episode_dir / "raw" / "wrist" / "step_000000_wrist_color.png"),
                cv2.cvtColor(wrist, cv2.COLOR_RGB2BGR),
            )
            cv2.imwrite(
                str(episode_dir / "raw" / "side" / "step_000000_side_color.png"),
                cv2.cvtColor(side, cv2.COLOR_RGB2BGR),
            )

            preview = build_preview_image_from_episode_dir(episode_dir)

            self.assertIsNotNone(preview)
            assert preview is not None
            self.assertEqual(preview.shape[0], 20)
            self.assertGreater(preview.shape[1], 30)


if __name__ == "__main__":
    raise SystemExit(unittest.main())
