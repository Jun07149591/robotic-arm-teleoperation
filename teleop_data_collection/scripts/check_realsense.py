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

from teleop_data_collection.lib.camera import make_camera
from teleop_data_collection.lib.config import load_config, resolve_camera_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check a configured RealSense camera connection and stream profile.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("teleop_data_collection/configs/dataset_v1.yaml"),
    )
    parser.add_argument("--camera-name", default="wrist")
    parser.add_argument("--serial", default=None)
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
    camera = make_camera({
        "serial": args.serial or camera_cfg.get("serial"),
        "width": int(args.width or camera_cfg.get("width", 640)),
        "height": int(args.height or camera_cfg.get("height", 480)),
        "fps": int(args.fps or camera_cfg.get("fps", 30)),
        "align_depth_to_color": (
            bool(camera_cfg.get("align_depth_to_color", True)) and not args.no_align
        ),
        "depth_width": camera_cfg.get("depth_width"),
        "depth_height": camera_cfg.get("depth_height"),
    })
    with camera:
        print(f"camera_name={camera_cfg.get('name', args.camera_name)}")
        print(f"camera_model={camera_cfg.get('model')}")
        print(f"serial={args.serial or camera_cfg.get('serial')}")
        print(f"depth_scale={camera.depth_scale:.8f}")
        camera.warmup(frame_count=args.warmup, timeout_ms=args.timeout_ms)
        frame = camera.get_frame(timeout_ms=args.timeout_ms)
        print(f"frame={frame.frame_number} ts_ms={frame.timestamp_ms:.3f}")
        print(f"intrinsics={frame.intrinsics}")
        print(f"color_shape={frame.color_bgr.shape}")
        print(f"depth_shape={frame.depth_raw.shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
