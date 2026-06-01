"""Find a target book spine in RealSense RGB frames.

Run from the repository root:

    python scripts/camera_test/realsense_book_spine_demo.py --template /home/boreas/project/lingzu_arm/EDULITE_A3/el_a3_sdk/assets/book_photos/net2.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any, Dict

import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from el_a3_sdk.realsense import (
    BookSpineMatchConfig,
    BookSpineMatcher,
    RealSenseD435,
    draw_book_spine_overlay,
)


DEFAULT_SAVE_DIR = Path("recordings/realsense/book_spine")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="使用 RealSense RGB 实时框选目标书脊模板。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    camera = parser.add_argument_group("camera")
    camera.add_argument("--serial", default=None, help="指定 RealSense 序列号；只有一台相机时可不填。")
    camera.add_argument("--width", type=int, default=1280, help="彩色和深度流宽度。")
    camera.add_argument("--height", type=int, default=720, help="彩色和深度流高度。")
    camera.add_argument("--fps", type=int, default=30, help="采集帧率。")
    camera.add_argument("--warmup", type=int, default=30, help="丢弃前 N 帧等待自动曝光稳定。")
    camera.add_argument("--timeout-ms", type=int, default=5000, help="等待相机帧的超时时间。")
    camera.add_argument("--no-align", action="store_true", help="不把深度图对齐到彩色图。")

    matcher = parser.add_argument_group("matcher")
    matcher.add_argument("--template", type=Path, required=True, help="目标书脊模板图片路径。")
    matcher.add_argument("--config-json", type=Path, default=None, help="可选：加载匹配参数 JSON。")
    matcher.add_argument("--match-confidence", type=float, default=None, help="匹配置信度阈值，0-1。")
    matcher.add_argument("--center-tolerance", type=float, default=None, help="中心容差比例，例如 0.03。")
    matcher.add_argument("--frame-max-side", type=int, default=None, help="匹配前限制帧最大边；0 表示不缩小。")
    matcher.add_argument("--template-max-side", type=int, default=None, help="模板图片最大边。")
    matcher.add_argument("--min-good-matches", type=int, default=None, help="最少有效特征匹配数量。")
    matcher.add_argument("--min-inliers", type=int, default=None, help="Homography 最少内点数量。")
    matcher.add_argument("--search-scales", default=None, help="搜索尺度，逗号分隔，例如 1,1.5,2。")
    matcher.add_argument("--max-scaled-frame-side", type=int, default=None, help="多尺度放大后的最大边；0 表示不限制。")
    matcher.add_argument("--disable-clahe", action="store_true", help="禁用灰度对比增强。")

    output = parser.add_argument_group("output")
    output.add_argument("--window-name", default="RealSense Book Spine", help="OpenCV 窗口名称。")
    output.add_argument("--save-dir", type=Path, default=DEFAULT_SAVE_DIR, help="保存截图和 JSON 的目录。")
    output.add_argument("--save-on-detect", action="store_true", help="首次稳定居中后保存一张结果图和 JSON。")
    output.add_argument("--no-window", action="store_true", help="不显示窗口，只在终端输出结果。")
    return parser.parse_args()


def load_config(args: argparse.Namespace) -> BookSpineMatchConfig:
    values: Dict[str, Any] = {}
    if args.config_json is not None:
        with args.config_json.open("r", encoding="utf-8") as file:
            loaded = json.load(file)
        if not isinstance(loaded, dict):
            raise RuntimeError("--config-json 内容必须是 JSON object。")
        values.update(loaded)
    if args.match_confidence is not None:
        values["match_confidence"] = args.match_confidence
    if args.center_tolerance is not None:
        values["center_tolerance_ratio"] = args.center_tolerance
    if args.frame_max_side is not None:
        values["frame_max_side"] = args.frame_max_side
    if args.template_max_side is not None:
        values["template_max_side"] = args.template_max_side
    if args.min_good_matches is not None:
        values["min_good_matches"] = args.min_good_matches
    if args.min_inliers is not None:
        values["min_inliers"] = args.min_inliers
    if args.search_scales is not None:
        values["search_scales"] = args.search_scales
    if args.max_scaled_frame_side is not None:
        values["max_scaled_frame_side"] = args.max_scaled_frame_side
    if args.disable_clahe:
        values["use_clahe"] = False
    config = BookSpineMatchConfig.from_mapping(values)
    config.validate()
    return config


def validate_args(args: argparse.Namespace) -> None:
    if not args.template.exists():
        raise RuntimeError(f"模板图片不存在：{args.template}")
    if args.width <= 0 or args.height <= 0:
        raise RuntimeError("--width 和 --height 必须为正数。")
    if args.fps <= 0:
        raise RuntimeError("--fps 必须为正数。")
    if args.warmup < 0:
        raise RuntimeError("--warmup 必须大于等于 0。")


def save_detection(save_dir: Path, frame_bgr, result, frame_number: int) -> None:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = f"book_spine_{timestamp}_frame{frame_number}"
    save_dir.mkdir(parents=True, exist_ok=True)
    image_path = save_dir / f"{prefix}.png"
    json_path = save_dir / f"{prefix}.json"
    cv2.imwrite(str(image_path), frame_bgr)
    payload = {
        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "frame_number": int(frame_number),
        **result.to_dict(),
    }
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"已保存识别结果: {image_path}")
    print(f"已保存识别 JSON: {json_path}")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        config = load_config(args)
        matcher = BookSpineMatcher.from_template_path(args.template, config)
        print(f"已加载模板: {args.template}")
        print(f"启用匹配后端: {', '.join(backend.name for backend in matcher.backends)}")

        saved_detection = False
        previous_time = time.time()
        with RealSenseD435(
            width=args.width,
            height=args.height,
            fps=args.fps,
            serial=args.serial,
            align_depth_to_color=not args.no_align,
        ) as camera:
            print(f"RealSense 已启动，depth scale = {camera.depth_scale:.8f} m/unit")
            camera.warmup(frame_count=args.warmup, timeout_ms=args.timeout_ms)
            print("开始识别。窗口中按 q 或 Esc 退出。")

            for frame in camera.iter_frames(timeout_ms=args.timeout_ms):
                result = matcher.match_frame(frame.color_bgr)
                display = draw_book_spine_overlay(
                    frame.color_bgr,
                    result,
                )

                now = time.time()
                fps = 1.0 / max(now - previous_time, 1e-6)
                previous_time = now
                cv2.putText(
                    display,
                    f"FPS: {fps:.1f}  match={result.match_confidence * 100.0:.1f}%",
                    (20, 108),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                if result.accepted:
                    bbox = result.bbox_xyxy
                    print(
                        "FOUND "
                        f"frame={frame.frame_number} "
                        f"bbox={bbox} "
                        f"conf={result.match_confidence:.3f} "
                        f"centered={result.stable_centered}"
                    )

                if args.save_on_detect and result.stable_centered and not saved_detection:
                    save_detection(args.save_dir, display, result, frame.frame_number)
                    saved_detection = True

                if not args.no_window:
                    cv2.imshow(args.window_name, display)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break

        cv2.destroyAllWindows()
        return 0
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted by user", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
