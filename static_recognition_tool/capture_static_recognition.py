from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from book_spine_core import DEFAULT_TEMPLATE_PATH, BookSpineRecognizer, format_result_summary, normalize_algorithm_name
from camera_source import FrameSource
from cnn_backup import DEFAULT_CNN_MODEL_PATH, TinySpineBackup


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture one still frame and run book-spine recognition.")
    parser.add_argument("--source", default="realsense", help="realsense, webcam index, image path, or video path.")
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE_PATH, help="Template image path.")
    parser.add_argument("--model", type=Path, default=DEFAULT_CNN_MODEL_PATH, help="Tiny CNN ONNX model path.")
    parser.add_argument("--algorithm", default="sift", help="sift, orb, surf, cnn, or hybrid.")
    parser.add_argument("--threshold", type=float, default=30.0, help="Feature detection threshold.")
    parser.add_argument("--cnn-threshold", type=float, default=0.45, help="CNN probability threshold.")
    parser.add_argument("--min-matches", type=int, default=6, help="Minimum good matches required.")
    parser.add_argument("--min-inliers", type=int, default=5, help="Minimum RANSAC inliers required.")
    parser.add_argument("--width", type=int, default=1920, help="Capture width.")
    parser.add_argument("--height", type=int, default=1080, help="Capture height.")
    parser.add_argument("--fps", type=int, default=30, help="Capture FPS.")
    parser.add_argument("--warmup-frames", type=int, default=30, help="Frames to discard before capture.")
    parser.add_argument(
        "--raw-output",
        type=Path,
        default=SCRIPT_DIR / "static_capture.jpg",
        help="Saved raw still frame.",
    )
    parser.add_argument(
        "--annotated-output",
        type=Path,
        default=SCRIPT_DIR / "static_recognition.jpg",
        help="Saved annotated recognition result.",
    )
    return parser.parse_args()


def capture_frame(args: argparse.Namespace):
    source = FrameSource(
        source=args.source,
        width=args.width,
        height=args.height,
        fps=args.fps,
        prefer_realsense=True,
    )
    try:
        backend = source.open()
        frame = None
        for _ in range(max(1, args.warmup_frames)):
            ok, frame = source.read()
            if not ok:
                frame = None
                time.sleep(0.02)
        if frame is None:
            raise RuntimeError("No frame captured.")
        return backend, frame
    finally:
        source.release()


def recognize(args: argparse.Namespace, frame):
    algorithm = args.algorithm.strip().lower()
    if algorithm == "cnn":
        recognizer = TinySpineBackup(model_path=args.model, threshold=args.cnn_threshold)
        return recognizer.annotate(frame)

    if algorithm == "hybrid":
        feature = BookSpineRecognizer(
            template_path=args.template,
            algorithm="sift",
            threshold=args.threshold,
            min_matches=args.min_matches,
            min_inliers=args.min_inliers,
        )
        annotated, result = feature.process_frame(frame)
        if result.detected:
            return annotated, result
        cnn = TinySpineBackup(model_path=args.model, threshold=args.cnn_threshold)
        return cnn.annotate(frame)

    feature_algorithm = normalize_algorithm_name(algorithm)
    recognizer = BookSpineRecognizer(
        template_path=args.template,
        algorithm=feature_algorithm,
        threshold=args.threshold,
        min_matches=args.min_matches,
        min_inliers=args.min_inliers,
    )
    return recognizer.process_frame(frame)


def main() -> int:
    args = parse_args()
    backend, frame = capture_frame(args)
    cv2.imwrite(str(args.raw_output), frame)
    annotated, result = recognize(args, frame)
    cv2.imwrite(str(args.annotated_output), annotated)

    print(f"Source: {backend}")
    print(f"Raw: {args.raw_output.resolve()}")
    print(f"Annotated: {args.annotated_output.resolve()}")
    print(format_result_summary(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

