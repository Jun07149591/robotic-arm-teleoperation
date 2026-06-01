"""Linux joystick API 读取器（纯内核接口，无外部依赖）"""

import struct
import os
import time
import threading
import logging
from typing import List, Optional

logger = logging.getLogger("el_a3_sdk.joystick")


class LinuxJoystick:
    """通过 Linux joystick API (/dev/input/jsX) 读取手柄输入

    不依赖任何外部库，仅使用 Linux 内核 joystick 接口。
    在后台线程中持续读取事件并更新共享状态。
    """

    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02
    JS_EVENT_INIT = 0x80
    EVENT_FORMAT = "IhBB"
    EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

    MAX_AXES = 8
    MAX_BUTTONS = 16

    def __init__(self, device: str = "/dev/input/js0"):
        self.device = device
        self.axes: List[float] = [0.0] * self.MAX_AXES
        self.buttons: List[int] = [0] * self.MAX_BUTTONS
        self.connected = False
        self._fd: Optional[int] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def connect(self) -> bool:
        try:
            self._fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            logger.error("无法打开手柄设备 %s: %s", self.device, e)
            return False

        self.connected = True
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop, daemon=True, name="joystick_reader")
        self._thread.start()
        return True

    def disconnect(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
        self.connected = False

    def _read_loop(self):
        while self._running:
            try:
                data = os.read(self._fd, self.EVENT_SIZE)
                if len(data) != self.EVENT_SIZE:
                    continue
                _ts, value, etype, number = struct.unpack(self.EVENT_FORMAT, data)
                etype &= ~self.JS_EVENT_INIT
                if etype == self.JS_EVENT_AXIS and number < self.MAX_AXES:
                    self.axes[number] = value / 32767.0
                elif etype == self.JS_EVENT_BUTTON and number < self.MAX_BUTTONS:
                    self.buttons[number] = value
            except BlockingIOError:
                time.sleep(0.002)
            except OSError:
                self.connected = False
                break
