#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_ROOT = REPO_ROOT / "el_a3_sdk"
for _path in (str(REPO_ROOT), str(SDK_ROOT)):
    while _path in sys.path:
        sys.path.remove(_path)
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SDK_ROOT))

from teleop_data_collection.lib.config import load_config, resolve_camera_config
from teleop_data_collection.lib.utils import dump_json, utc_now_iso


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write a configured RealSense extrinsic calibration template.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("teleop_data_collection/configs/dataset_v1.yaml"),
    )
    parser.add_argument("--camera-name", default="wrist")
    parser.add_argument("--serial", default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--source-frame", default=None)
    parser.add_argument("--target-frame", default="robot_base")
    parser.add_argument("--method", default="manual_handeye_or_marker")
    parser.add_argument("--notes", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)
    camera_cfg = dict(resolve_camera_config(cfg.camera, args.camera_name))
    serial = args.serial or camera_cfg.get("serial")
    source_frame = args.source_frame or camera_cfg.get("source_frame", "color_optical_frame")
    output_path = args.output or Path(
        f"teleop_data_collection/calibrations/{args.camera_name}_extrinsic.json"
    )

    payload = {
        "schema_version": "realsense_extrinsic_v1",
        "camera_name": args.camera_name,
        "camera_model": camera_cfg.get("model", "Intel RealSense"),
        "serial": serial,
        "source_frame": source_frame,
        "target_frame": args.target_frame,
        "method": args.method,
        "frame_convention": "source_to_target",
        "reference_object": None,
        "matrix": [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "xyz_m": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "timestamp": utc_now_iso(),
        "num_samples": 0,
        "sample_method": args.method,
        "calibration_error_m": None,
        "reprojection_error": None,
        "robot_frame": args.target_frame,
        "camera_frame": source_frame,
        "notes": args.notes,
    }
    dump_json(output_path, payload)
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
