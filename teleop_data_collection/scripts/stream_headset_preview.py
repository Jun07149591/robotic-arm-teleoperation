#!/usr/bin/env python3
"""Relay latest preview JPEG to the Pico WebXR receiver.

This process does not open RealSense cameras. It only reads a JPEG file written
by ``record_sdk_episode.py --headset-preview-file`` and POSTs it to
``pico3_webxr_pose_receiver.py /preview``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import ssl
import time
import urllib.error
import urllib.request


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Relay teleop camera preview JPEG to Pico headset.")
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("/tmp/teleop_headset_preview.jpg"),
        help="Preview JPEG written by record_sdk_episode.py.",
    )
    parser.add_argument(
        "--url",
        default="https://127.0.0.1:8443/preview",
        help="pico3_webxr_pose_receiver.py preview endpoint.",
    )
    parser.add_argument("--fps", type=float, default=10.0, help="Maximum relay FPS.")
    parser.add_argument("--timeout", type=float, default=0.2, help="HTTP POST timeout in seconds.")
    return parser.parse_args()


def post_preview(path: Path, url: str, timeout: float) -> int:
    payload = path.read_bytes()
    request = urllib.request.Request(
        url,
        data=payload,
        headers={"Content-Type": "image/jpeg"},
        method="POST",
    )
    context = ssl._create_unverified_context() if url.startswith("https://") else None
    with urllib.request.urlopen(request, timeout=timeout, context=context) as response:
        response.read(1)
        return int(response.status)


def main() -> int:
    args = parse_args()
    interval = 1.0 / max(float(args.fps), 1e-6)
    last_mtime_ns = -1
    last_print = 0.0
    sent = 0
    print(f"relay input: {args.input}")
    print(f"relay url:   {args.url}")
    while True:
        loop_start = time.monotonic()
        try:
            stat = args.input.stat()
            if stat.st_mtime_ns != last_mtime_ns and stat.st_size > 0:
                status = post_preview(args.input, args.url, float(args.timeout))
                last_mtime_ns = stat.st_mtime_ns
                sent += 1
                now = time.monotonic()
                if now - last_print >= 1.0:
                    last_print = now
                    print(f"relayed frames={sent} size={stat.st_size} status={status}", flush=True)
        except FileNotFoundError:
            pass
        except (OSError, urllib.error.URLError) as exc:
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                print(f"relay warning: {exc}", flush=True)
        elapsed = time.monotonic() - loop_start
        if elapsed < interval:
            time.sleep(interval - elapsed)


if __name__ == "__main__":
    raise SystemExit(main())
