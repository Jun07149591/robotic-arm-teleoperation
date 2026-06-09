from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from pathlib import Path
from contextlib import redirect_stdout
import io

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from teleop_data_collection.lib.config import resolve_camera_config
from teleop_data_collection.lib.episode import EpisodeWriter
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
from teleop_data_collection.scripts.record_xbox_episode import (
    _make_episode_writer as make_xbox_episode_writer,
    _validate_output_fps as validate_xbox_output_fps,
    _should_continue_after_episode,
    parse_args as parse_xbox_record_args,
)
from teleop_data_collection.scripts.export_ros_xbox_state import (
    _apply_controller_profile,
    _load_ros_dependencies,
    parse_args as parse_xbox_export_args,
)
from teleop_data_collection.scripts.inspect_episode import main as inspect_episode_main


REPO_ROOT = Path(__file__).resolve().parents[2]
os.environ.setdefault("HF_HOME", "/tmp/hf_home_lerobot_tests")
os.environ.setdefault("HF_DATASETS_CACHE", "/tmp/hf_datasets_lerobot_tests")


class LeRobotCompatibilityTest(unittest.TestCase):
    def test_recording_metadata_uses_jointctrl_controller(self) -> None:
        self.assertEqual(TELEOP_CONTROLLER, "el_a3_sdk/demo/pico_control_jointctrl.py")

    def test_xbox_record_args_support_continuous_collection(self) -> None:
        args = parse_xbox_record_args([
            "--task", "pick up the object",
            "--continuous",
            "--max-episodes", "3",
        ])
        self.assertTrue(args.continuous)
        self.assertEqual(args.max_episodes, 3)
        self.assertEqual(args.keyboard_stop_key, "q")
        self.assertEqual(args.keyboard_fail_key, "f")

        args = parse_xbox_record_args([
            "--task", "pick up the object",
            "--keyboard-stop-key", "e",
            "--keyboard-fail-key", "x",
        ])
        self.assertEqual(args.keyboard_stop_key, "e")
        self.assertEqual(args.keyboard_fail_key, "x")

    def test_xbox_recording_requires_output_fps_to_match_sample_hz(self) -> None:
        args = parse_xbox_record_args(["--task", "pick up the object", "--hz", "15", "--fps", "15"])
        validate_xbox_output_fps(args)

        args = parse_xbox_record_args(["--task", "pick up the object", "--hz", "15", "--fps", "30"])
        with self.assertRaises(ValueError):
            validate_xbox_output_fps(args)

    def test_keyboard_stop_key_matching_supports_q_escape_and_disable(self) -> None:
        from teleop_data_collection.lib.keyboard import key_matches_stop, normalize_stop_key

        self.assertEqual(normalize_stop_key("q"), "q")
        self.assertEqual(normalize_stop_key("esc"), "\x1b")
        self.assertTrue(key_matches_stop("q", "q"))
        self.assertTrue(key_matches_stop("\x1b", "esc"))
        self.assertTrue(key_matches_stop("f", "f"))
        self.assertFalse(key_matches_stop("q", ""))

    def test_xbox_continuous_stop_decision(self) -> None:
        self.assertFalse(
            _should_continue_after_episode(
                continuous=False,
                saved_episodes=1,
                max_episodes=0,
                shutdown_requested=False,
                episode_success=True,
            )
        )
        self.assertTrue(
            _should_continue_after_episode(
                continuous=True,
                saved_episodes=1,
                max_episodes=0,
                shutdown_requested=False,
                episode_success=True,
            )
        )
        self.assertFalse(
            _should_continue_after_episode(
                continuous=True,
                saved_episodes=3,
                max_episodes=3,
                shutdown_requested=False,
                episode_success=True,
            )
        )
        self.assertFalse(
            _should_continue_after_episode(
                continuous=True,
                saved_episodes=1,
                max_episodes=0,
                shutdown_requested=True,
                episode_success=True,
            )
        )
        self.assertFalse(
            _should_continue_after_episode(
                continuous=True,
                saved_episodes=1,
                max_episodes=0,
                shutdown_requested=False,
                episode_success=False,
                continue_after_keyboard_failure=False,
            )
        )
        self.assertTrue(
            _should_continue_after_episode(
                continuous=True,
                saved_episodes=1,
                max_episodes=0,
                shutdown_requested=False,
                episode_success=False,
                continue_after_keyboard_failure=True,
            )
        )

    def test_xbox_episode_writer_uses_xbox_controller_observation(self) -> None:
        args = parse_xbox_record_args(["--task", "pick up the object"])
        with tempfile.TemporaryDirectory() as tmp:
            writer = make_xbox_episode_writer(
                args=args,
                dataset_cfg={},
                robot_cfg={},
                camera_cfg={},
                camera_configs=[{"name": "wrist"}],
                image_observation_keys=["wrist"],
                dataset_root=Path(tmp),
                episode_id="episode_000000",
                ep_idx=0,
                state_file=Path("/tmp/robot_latest_state.json"),
                gamepad_file=Path("/tmp/xbox_latest_input.json"),
                save_raw_images=False,
            )

            self.assertEqual(writer._lerobot.controller_observation, "xbox")
            self.assertIn("observation.xbox", writer._lerobot.features)
            self.assertNotIn("observation.gamepad", writer._lerobot.features)

    def test_ros_xbox_exporter_uses_profile_button_indices(self) -> None:
        args = parse_xbox_export_args(["--profile", "zikway_3537_1041"])

        _apply_controller_profile(args)

        self.assertEqual(args.profile, "zikway_3537_1041")
        self.assertEqual(args.speed_button_index, 0)
        self.assertEqual(args.start_button_index, 11)
        self.assertEqual(args.back_button_index, 10)
        self.assertEqual(args.zero_torque_button_index, 4)

    def test_ros_xbox_exporter_reports_active_python_on_ros_import_failure(self) -> None:
        real_import = __import__

        def fake_import(name, globals=None, locals=None, fromlist=(), level=0):
            if name == "rclpy":
                raise ImportError("No module named 'rclpy._rclpy_pybind11'")
            return real_import(name, globals, locals, fromlist, level)

        with patch("builtins.__import__", side_effect=fake_import):
            with self.assertRaises(SystemExit) as ctx:
                _load_ros_dependencies()

        message = str(ctx.exception)
        self.assertIn("Python executable:", message)
        self.assertIn(sys.executable, message)
        self.assertIn("/usr/bin/python3 teleop_data_collection/scripts/export_ros_xbox_state.py", message)

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

    def test_empty_episode_finalize_removes_raw_episode_directory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "edulite_a3"
            writer = EpisodeWriter(
                root=root,
                episode_id="episode_000000",
                meta={"task_text": "empty episode", "success": True},
                image_observation_keys=["wrist"],
                controller_observation="xbox",
            )
            episode_dir = root / "episode_000000"
            self.assertTrue(episode_dir.exists())

            writer.finalize()

            self.assertFalse(episode_dir.exists())
            self.assertFalse((root / "data").exists())
            self.assertFalse((root / "videos").exists())

    def test_build_features_state_names_match_shape(self) -> None:
        features = build_features()
        self.assertEqual(len(features["observation.state"]["names"]), features["observation.state"]["shape"][0])
        self.assertEqual(len(features["observation.pico"]["names"]), features["observation.pico"]["shape"][0])
        self.assertEqual(len(features["action"]["names"]), features["action"]["shape"][0])

    def test_build_features_can_select_xbox_controller_observation(self) -> None:
        pico_features = build_features(controller_observation="pico")
        self.assertIn("observation.pico", pico_features)
        self.assertNotIn("observation.xbox", pico_features)

        xbox_features = build_features(controller_observation="xbox")
        self.assertIn("observation.xbox", xbox_features)
        self.assertNotIn("observation.pico", xbox_features)
        self.assertEqual(xbox_features["observation.xbox"]["shape"][0], 20)
        self.assertEqual(
            len(xbox_features["observation.xbox"]["names"]),
            xbox_features["observation.xbox"]["shape"][0],
        )
        self.assertEqual(
            xbox_features["observation.xbox"]["names"],
            [
                "lx", "ly", "rx", "ry", "lt", "rt", "dpad_x", "dpad_y",
                "btn_a", "btn_b", "btn_x", "btn_y", "btn_lb", "btn_rb",
                "btn_back", "btn_start", "valid", "speed_level",
                "mode_normal", "episode_done",
            ],
        )

    def test_gamepad_state_reader_rejects_missing_and_stale_files(self) -> None:
        from teleop_data_collection.lib.gamepad import read_gamepad_state

        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "xbox_latest_input.json"
            self.assertFalse(read_gamepad_state(missing).valid)

            missing.write_text(
                json.dumps({"timestamp_ns": time.time_ns() - int(5e9), "axes": [1.0]}),
                encoding="utf-8",
            )
            self.assertFalse(read_gamepad_state(missing).valid)

    def test_flatten_xbox_observation_is_named_and_compact(self) -> None:
        from teleop_data_collection.lib.gamepad import GamepadState, flatten_xbox_observation

        state = GamepadState(
            valid=True,
            timestamp_ns=time.time_ns(),
            axes=[0.1, -0.2, -1.0, 0.3, -0.4, 1.0, -1.0, 1.0],
            buttons=[1, 0, 1, 0, 1, 0, 1, 1],
            speed_level=3,
            mode="normal",
            profile="xbox_default",
            device="/dev/input/js0",
            episode_done=True,
        )

        obs = flatten_xbox_observation(state)
        self.assertEqual(obs.shape, (20,))
        self.assertAlmostEqual(float(obs[0]), 0.1, places=6)
        self.assertAlmostEqual(float(obs[1]), -0.2, places=6)
        self.assertAlmostEqual(float(obs[2]), 0.3, places=6)
        self.assertAlmostEqual(float(obs[3]), -0.4, places=6)
        self.assertEqual(float(obs[4]), 0.0)
        self.assertEqual(float(obs[5]), 1.0)
        self.assertEqual(float(obs[6]), -1.0)
        self.assertEqual(float(obs[7]), 1.0)
        self.assertEqual(float(obs[8]), 1.0)
        self.assertEqual(float(obs[10]), 1.0)
        self.assertEqual(float(obs[12]), 1.0)
        self.assertEqual(float(obs[14]), 1.0)
        self.assertEqual(float(obs[15]), 1.0)
        self.assertEqual(float(obs[17]), 3.0)
        self.assertEqual(float(obs[18]), 1.0)
        self.assertEqual(float(obs[19]), 1.0)

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

    def test_writer_supports_xbox_controller_feature(self) -> None:
        from teleop_data_collection.lib.gamepad import GamepadState

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "edulite_a3_xbox"
            writer = LeRobotEpisodeWriter(
                root=root,
                episode_index=0,
                task="pick book",
                fps=15,
                robot_type="edulite_a3",
                image_observation_keys=["wrist", "side"],
                controller_observation="xbox",
            )
            frame = np.full((32, 48, 3), 80, dtype=np.uint8)
            writer.add_step(
                {
                    "qpos": [0.0] * 7,
                    "qvel": [0.0] * 7,
                    "tau": [0.0] * 7,
                    "ee_pose": [0.0] * 6,
                    "gripper_pos": 0.0,
                    "gamepad_state": GamepadState(
                        valid=True,
                        timestamp_ns=time.time_ns(),
                        axes=[0.25],
                        buttons=[1],
                        speed_level=2,
                        mode="normal",
                    ),
                    "action_joint_delta": [0.0] * 7,
                    "action_gripper_delta": 0.0,
                    "image_frames": {"wrist": frame, "side": frame},
                }
            )
            writer.finalize(done=True, success=True)

            info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
            self.assertIn("observation.xbox", info["features"])
            self.assertNotIn("observation.pico", info["features"])

            table = pq.read_table(root / "data" / "chunk-000" / "file-000.parquet")
            self.assertIn("observation.xbox", table.column_names)
            self.assertEqual(len(table["observation.xbox"][0].as_py()), 20)

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

    def test_resolve_episode_identity_ignores_raw_episode_gaps(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "meta").mkdir(parents=True)
            (root / "meta" / "info.json").write_text(
                json.dumps({"total_episodes": 25}),
                encoding="utf-8",
            )
            (root / "episode_000027").mkdir()
            self.assertEqual(_resolve_episode_identity(root, None), ("episode_000025", 25))
            (root / "episode_000025" / "raw").mkdir(parents=True)
            with self.assertRaisesRegex(ValueError, "raw directory already exists"):
                _resolve_episode_identity(root, None)

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
