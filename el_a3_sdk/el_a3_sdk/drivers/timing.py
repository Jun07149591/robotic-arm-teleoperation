"""Timing helpers shared by CAN driver backends and control loops."""

import time


def busy_wait_us(us: int):
    """Spin-wait for approximately `us` microseconds."""
    target = time.perf_counter() + us * 1e-6
    while time.perf_counter() < target:
        pass


__all__ = ["busy_wait_us"]
