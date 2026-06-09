from __future__ import annotations

import select
import sys
import termios
import tty
from types import TracebackType
from typing import TextIO


def normalize_stop_key(key: str | None) -> str:
    if key is None:
        return ""
    key = str(key)
    if key.lower() in ("esc", "escape"):
        return "\x1b"
    if key.lower() in ("none", "off", "disabled"):
        return ""
    return key[:1]


def key_matches_stop(char: str, stop_key: str | None) -> bool:
    key = normalize_stop_key(stop_key)
    return bool(key) and char == key


class KeyboardStopWatcher:
    """Non-blocking success/failure key watcher for terminal recording loops."""

    def __init__(
        self,
        stop_key: str | None = "q",
        *,
        fail_key: str | None = None,
        stream: TextIO | None = None,
    ) -> None:
        self.stop_key = normalize_stop_key(stop_key)
        self.fail_key = normalize_stop_key(fail_key)
        self.stream = stream or sys.stdin
        self._fd: int | None = None
        self._old_settings: list | None = None
        self.enabled = False

    def __enter__(self) -> "KeyboardStopWatcher":
        if not (self.stop_key or self.fail_key) or not self.stream.isatty():
            return self
        self._fd = self.stream.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self.enabled = True
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
        self.enabled = False

    def poll(self) -> bool:
        return self.poll_action() == "success"

    def poll_action(self) -> str | None:
        if not self.enabled:
            return None
        ready, _, _ = select.select([self.stream], [], [], 0)
        if not ready:
            return None
        char = self.stream.read(1)
        if key_matches_stop(char, self.stop_key):
            return "success"
        if key_matches_stop(char, self.fail_key):
            return "failure"
        return None
