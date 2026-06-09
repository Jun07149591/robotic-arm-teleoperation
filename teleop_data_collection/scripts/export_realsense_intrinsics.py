#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import asdict
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_ROOT = REPO_ROOT / "el_a3_sdk"
for _path in (str(REPO_ROOT), str(SDK_ROOT)):
    while _path in sys.path:
        sys.path.remove(_path)
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SDK_ROOT))

from teleop_data_collection.lib.camera import make_camera
from teleop_data_collection.lib.config import load_config, resolve_camera_config
from teleop_data_collection.lib.utils import dump_json, utc_now_iso


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a configured RealSense intrinsics JSON.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("teleop_data_collection/configs/dataset_v1.yaml"),
    )
    parser.add_argument("--camera-name", default="wrist")
    parser.add_argument("--serial", default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--fps", type=int, default=None)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--no-align", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)
    camera_cfg = dict(resolve_camera_config(cfg.camera, args.camera_name))
    serial = args.serial or camera_cfg.get("serial")
    width = int(args.width or camera_cfg.get("width", 640))
    height = int(args.height or camera_cfg.get("height", 480))
    fps = int(args.fps or camera_cfg.get("fps", 30))
    output_path = args.output or Path(
        f"teleop_data_collection/calibrations/{args.camera_name}_intrinsics.json"
    )

    camera = make_camera(
        {
            "serial": serial,
            "width": width,
            "height": height,
            "fps": fps,
            "align_depth_to_color": (
                bool(camera_cfg.get("align_depth_to_color", True)) and not args.no_align
            ),
            "depth_width": camera_cfg.get("depth_width"),
            "depth_height": camera_cfg.get("depth_height"),
        }
    )
    with camera:
        camera.warmup(frame_count=args.warmup, timeout_ms=args.timeout_ms)
        frame = camera.get_frame(timeout_ms=args.timeout_ms)
        payload = {
            "schema_version": "realsense_intrinsics_v1",
            "camera_name": args.camera_name,
            "camera_model": camera_cfg.get("model", "Intel RealSense"),
            "serial": serial,
            "firmware_version": None,
            "stream": {
                "color": {
                    "width": frame.intrinsics.width,
                    "height": frame.intrinsics.height,
                    "fps": fps,
                    "format": "bgr8",
                },
                "depth": {
                    "width": frame.depth_raw.shape[1],
                    "height": frame.depth_raw.shape[0],
                    "fps": fps,
                    "format": "z16",
                },
                "depth_aligned_to_color": bool(frame.depth_aligned_to_color),
            },
            "intrinsics": asdict(frame.intrinsics),
            "depth_scale": float(frame.depth_scale),
            "timestamp": utc_now_iso(),
            "calibration_state": "factory_intrinsics",
            "depth_alignment": "color" if frame.depth_aligned_to_color else "depth",
            "source_frame": camera_cfg.get("source_frame"),
        }
    dump_json(output_path, payload)
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
