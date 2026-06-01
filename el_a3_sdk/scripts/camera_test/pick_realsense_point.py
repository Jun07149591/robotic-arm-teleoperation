"""Capture a RealSense point cloud, visualize it, and pick one target point.

Run from the repository root:

    python scripts/camera_test/pick_realsense_point.py

Open3D point picking:

    Shift + left click   select a point
    Shift + right click  undo selection
    Q or Esc             finish picking
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
from typing import Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from el_a3_sdk.realsense import (
    RealSenseD435,
    RigidTransform,
    pick_target,
)


DEFAULT_SAVE_DIR = Path("recordings/realsense")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="采集 RealSense 点云，在 Open3D 中选取一个目标点。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    camera = parser.add_argument_group("camera")
    camera.add_argument("--serial", default=None, help="指定 RealSense 序列号；只有一台相机时可不填。")
    camera.add_argument("--width", type=int, default=640, help="彩色和深度流宽度。")
    camera.add_argument("--height", type=int, default=480, help="彩色和深度流高度。")
    camera.add_argument("--fps", type=int, default=30, help="采集帧率。")
    camera.add_argument("--warmup", type=int, default=30, help="丢弃前 N 帧等待自动曝光稳定。")
    camera.add_argument("--timeout-ms", type=int, default=5000, help="等待相机帧的超时时间。")
    camera.add_argument("--no-align", action="store_true", help="不把深度图对齐到彩色图。")

    cloud = parser.add_argument_group("point cloud")
    cloud.add_argument("--depth-min", type=float, default=0.0, help="点云保留的最小深度，单位米。")
    cloud.add_argument("--depth-max", "--max-depth", dest="depth_max", type=float, default=2.0, help="点云保留的最大深度，单位米。")
    cloud.add_argument("--stride", type=int, default=1, help="点云采样步长；1 表示保留所有有效深度点。")
    cloud.add_argument("--no-color", action="store_true", help="不使用 RGB 给点云上色。")
    cloud.add_argument("--voxel-size", type=float, default=0.0, help="Open3D 体素降采样尺寸，单位米；0 表示不降采样。")

    picker = parser.add_argument_group("picker")
    picker.add_argument("--point-size", type=float, default=1.0, help="Open3D 选点窗口中的点大小。")
    picker.add_argument("--no-flip-view", dest="flip_view", action="store_false", default=True, help="Open3D 窗口保留 RealSense 原始坐标朝向。")
    picker.add_argument("--window-width", type=int, default=1280, help="Open3D 窗口宽度。")
    picker.add_argument("--window-height", type=int, default=720, help="Open3D 窗口高度。")

    save = parser.add_argument_group("save")
    save.add_argument("--save-dir", type=Path, default=DEFAULT_SAVE_DIR, help="输出目录。")
    save.add_argument("--prefix", default="realsense_pick", help="保存文件名前缀。")
    save.add_argument("--save-json", type=Path, default=None, help="保存选点结果 JSON；默认保存到 save-dir。")
    save.add_argument("--save-npz", action="store_true", help="保存原始点云 NPZ。")
    save.add_argument("--save-ply", action="store_true", help="保存原始点云 PLY。")
    save.add_argument("--save-images", action="store_true", help="保存彩色图、原始深度图和深度伪彩色图。")

    transform = parser.add_argument_group("optional transform")
    transform.add_argument("--transform-json", type=Path, default=None, help="可选：加载 RigidTransform JSON，并额外输出 target_point_m。")
    transform.add_argument("--camera-to-target-xyz", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"), help="直接指定目标坐标系下的相机原点平移，单位米。")
    transform.add_argument("--camera-to-target-rpy-deg", type=float, nargs=3, default=None, metavar=("ROLL", "PITCH", "YAW"), help="直接指定相机到目标坐标系的 RPY，单位度。")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.depth_min < 0:
        raise RuntimeError("--depth-min 必须大于等于 0。")
    if args.depth_max <= args.depth_min:
        raise RuntimeError("--depth-max 必须大于 --depth-min。")
    if args.stride < 1:
        raise RuntimeError("--stride 必须大于等于 1。")
    if args.voxel_size < 0:
        raise RuntimeError("--voxel-size 必须大于等于 0。")
    if (args.camera_to_target_xyz is None) != (args.camera_to_target_rpy_deg is None):
        raise RuntimeError("--camera-to-target-xyz 和 --camera-to-target-rpy-deg 必须同时提供。")


def make_transform(args: argparse.Namespace) -> Optional[RigidTransform]:
    if args.transform_json is not None:
        return RigidTransform.from_json(args.transform_json)
    if args.camera_to_target_xyz is not None:
        return RigidTransform.from_xyz_rpy_deg(
            args.camera_to_target_xyz,
            args.camera_to_target_rpy_deg,
        )
    return None


def filter_depth_min(point_cloud, depth_min: float):
    if depth_min <= 0:
        return point_cloud

    keep = np.asarray(point_cloud.points_xyz_m)[:, 2] >= depth_min
    return type(point_cloud)(
        points_xyz_m=np.asarray(point_cloud.points_xyz_m)[keep],
        colors_rgb=None if point_cloud.colors_rgb is None else np.asarray(point_cloud.colors_rgb)[keep],
        pixels_uv=np.asarray(point_cloud.pixels_uv)[keep],
        intrinsics=point_cloud.intrinsics,
        timestamp_ms=point_cloud.timestamp_ms,
        frame_number=point_cloud.frame_number,
    )


def downsample_for_pick(point_cloud, voxel_size: float):
    if voxel_size <= 0:
        return point_cloud

    open3d_cloud = point_cloud.to_open3d()
    downsampled = open3d_cloud.voxel_down_sample(voxel_size)
    points = np.asarray(downsampled.points, dtype=np.float32)
    colors = None
    if downsampled.has_colors():
        colors = (np.asarray(downsampled.colors) * 255.0).clip(0, 255).astype(np.uint8)

    return type(point_cloud)(
        points_xyz_m=points,
        colors_rgb=colors,
        pixels_uv=np.full((len(points), 2), -1, dtype=np.int32),
        intrinsics=point_cloud.intrinsics,
        timestamp_ms=point_cloud.timestamp_ms,
        frame_number=point_cloud.frame_number,
    )


def print_frame_summary(frame, point_cloud) -> None:
    valid = frame.valid_depth_mask()
    valid_depth = frame.depth_m[valid]
    print("已采集 RealSense RGB-D 帧")
    print(f"  frame_number: {frame.frame_number}")
    print(f"  timestamp_ms: {frame.timestamp_ms:.3f}")
    print(f"  depth_scale: {frame.depth_scale:.8f} m/unit")
    print(
        "  intrinsics: "
        f"{frame.intrinsics.width}x{frame.intrinsics.height}, "
        f"fx={frame.intrinsics.fx:.3f}, fy={frame.intrinsics.fy:.3f}, "
        f"ppx={frame.intrinsics.ppx:.3f}, ppy={frame.intrinsics.ppy:.3f}"
    )
    if valid_depth.size:
        print(
            "  valid_depth_m: "
            f"count={valid_depth.size}, min={float(np.min(valid_depth)):.4f}, "
            f"mean={float(np.mean(valid_depth)):.4f}, max={float(np.max(valid_depth)):.4f}"
        )
    else:
        print("  valid_depth_m: count=0")
    print(f"  point_count: {point_cloud.size}")


def save_outputs(frame, point_cloud, selected, args: argparse.Namespace) -> None:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = f"{args.prefix}_{timestamp}_frame{frame.frame_number}"
    args.save_dir.mkdir(parents=True, exist_ok=True)

    if args.save_images:
        image_paths = frame.save_images(
            args.save_dir,
            prefix=prefix,
            depth_vis_max_m=args.depth_max,
        )
        for name, path in image_paths.items():
            print(f"已保存 {name}: {path}")

    if args.save_npz:
        npz_path = point_cloud.save_npz(args.save_dir / f"{prefix}_point_cloud.npz")
        print(f"已保存点云 NPZ: {npz_path}")

    if args.save_ply:
        ply_path = point_cloud.save_ply(args.save_dir / f"{prefix}_point_cloud.ply")
        print(f"已保存点云 PLY: {ply_path}")

    json_path = args.save_json or (args.save_dir / f"{prefix}_selected_point.json")
    payload = {
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "frame_number": int(frame.frame_number),
        "timestamp_ms": float(frame.timestamp_ms),
        "depth_aligned_to_color": bool(frame.depth_aligned_to_color),
        "intrinsics": {
            "width": frame.intrinsics.width,
            "height": frame.intrinsics.height,
            "fx": frame.intrinsics.fx,
            "fy": frame.intrinsics.fy,
            "ppx": frame.intrinsics.ppx,
            "ppy": frame.intrinsics.ppy,
        },
        **selected.to_dict(),
    }
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"已保存选点 JSON: {json_path}")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        if args.save_json is not None and not args.save_json.is_absolute():
            args.save_json = (Path.cwd() / args.save_json).resolve()
        transform = make_transform(args)
        if transform is not None:
            print("使用相机到目标坐标系变换矩阵：")
            print(transform.matrix)

        with RealSenseD435(
            width=args.width,
            height=args.height,
            fps=args.fps,
            serial=args.serial,
            align_depth_to_color=not args.no_align,
        ) as camera:
            print(f"当前 Python 解释器: {sys.executable}")
            print(f"RealSense 已启动，depth scale = {camera.depth_scale:.8f} m/unit")
            camera.warmup(frame_count=args.warmup, timeout_ms=args.timeout_ms)
            frame = camera.get_frame(timeout_ms=args.timeout_ms)

        point_cloud = frame.to_point_cloud(
            max_depth_m=args.depth_max,
            stride=args.stride,
            include_color=not args.no_color,
        )
        point_cloud = filter_depth_min(point_cloud, args.depth_min)
        if point_cloud.size == 0:
            raise RuntimeError("点云为空，请调整深度范围或相机视角。")

        pick_cloud = downsample_for_pick(point_cloud, args.voxel_size)
        print_frame_summary(frame, pick_cloud)
        if pick_cloud.size != point_cloud.size:
            print(f"  raw_point_count: {point_cloud.size}")

        selected = pick_target(
            pick_cloud,
            transform=transform,
            point_size=args.point_size,
            flip_view=args.flip_view,
            window_name="RealSense 点云选点",
            width=args.window_width,
            height=args.window_height,
        )

        print("\n选点结果")
        print(f"  picked_index: {selected.picked_index}")
        if selected.pixel_uv is not None and selected.pixel_uv.size == 2:
            print(f"  pixel_uv: {selected.pixel_uv.tolist()}")
        print(f"  camera_point_m: {np.array2string(selected.camera_point_m, precision=6, suppress_small=True)}")
        print(f"  camera_point_mm: {np.array2string(selected.camera_point_m * 1000.0, precision=3, suppress_small=True)}")
        if selected.target_point_m is not None:
            print(f"  target_point_m: {np.array2string(selected.target_point_m, precision=6, suppress_small=True)}")
            print(f"  target_point_mm: {np.array2string(selected.target_point_m * 1000.0, precision=3, suppress_small=True)}")

        save_outputs(frame, point_cloud, selected, args)
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted by user", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
