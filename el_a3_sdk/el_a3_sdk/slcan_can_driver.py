"""
EL-A3 SDK SLCAN CAN 驱动层

基于 pyserial + SLCAN (Lawicel) ASCII 协议的 Robstride 电机通信驱动。
与 RobstrideCanDriver 具有相同的公开 API，可在 Windows / Linux / macOS 上
通过 USB-CAN 适配器（SLCAN 固件）与电机通信。

SLCAN 帧格式（扩展帧，29 位 ID）:
  发送/接收: T<8hex_id><1digit_dlc><2*dlc hex_data>\r
"""

import struct
import time
import threading
import logging
from typing import Dict, Optional, Callable

import serial

from el_a3_sdk.protocol import (
    CommType, MotorType, RunMode, MotorParams, ParamIndex,
    MOTOR_PARAMS, DEFAULT_MOTOR_TYPE_MAP,
)
from el_a3_sdk.data_types import (
    MotorFeedback, ParamReadResult, FirmwareVersion,
)
from el_a3_sdk.utils import float_to_uint16, uint16_to_float

logger = logging.getLogger("el_a3_sdk.slcan_driver")

CAN_EFF_MASK = 0x1FFFFFFF

SLCAN_BITRATE_MAP = {
    10000: b"S0",
    20000: b"S1",
    50000: b"S2",
    100000: b"S3",
    125000: b"S4",
    250000: b"S5",
    500000: b"S6",
    800000: b"S7",
    1000000: b"S8",
}


def _busy_wait_us(us: int):
    """微秒级忙等待"""
    target = time.perf_counter() + us * 1e-6
    while time.perf_counter() < target:
        pass


class SlcanCanDriver:
    """
    Robstride 电机 SLCAN 底层驱动 (Windows / 跨平台)

    功能与 RobstrideCanDriver 完全对等：
    - 串口 + SLCAN 协议管理（连接/断开）
    - 帧收发（带重试）
    - 后台接收线程 + 反馈帧解析
    - 电机使能/失能/设零/运控指令
    - 参数读写
    """

    def __init__(self, serial_port: str, host_can_id: int = 0xFD,
                 motor_type_map: Optional[Dict[int, MotorType]] = None,
                 serial_baudrate: int = 2000000,
                 can_bitrate: int = 1000000):
        self.serial_port = serial_port
        self.host_can_id = host_can_id
        self.motor_type_map = motor_type_map or dict(DEFAULT_MOTOR_TYPE_MAP)
        self.serial_baudrate = serial_baudrate
        self.can_bitrate = can_bitrate

        # for interface.py compatibility (it reads driver.can_name in a few places)
        self.can_name = serial_port

        self._serial: Optional[serial.Serial] = None
        self._feedbacks: Dict[int, MotorFeedback] = {}
        self._param_results: Dict[int, ParamReadResult] = {}
        self._firmware_versions: Dict[int, FirmwareVersion] = {}
        self._fault_details: Dict[int, int] = {}
        self._awaiting_version: Dict[int, bool] = {}

        self._recv_running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()

        self._feedback_callback: Optional[Callable[[MotorFeedback], None]] = None

        self._frame_count = 0
        self._fps_start_time = time.time()
        self._current_fps = 0.0

        self._tx_ok_count = 0
        self._tx_fail_count = 0
        self._tx_fail_window_count = 0
        self._tx_ok_window_count = 0
        self._tx_window_start = time.time()
        self._tx_fail_rate = 0.0
        self._tx_warn_time = 0.0

    # ========== 连接管理 ==========

    def connect(self) -> bool:
        """打开串口并初始化 SLCAN 通道"""
        try:
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=0.1,
                write_timeout=0.1,
            )
            time.sleep(0.05)

            # 先关闭可能已打开的通道
            self._slcan_cmd(b"C")
            time.sleep(0.02)

            # 设置 CAN 波特率
            bitrate_cmd = SLCAN_BITRATE_MAP.get(self.can_bitrate)
            if bitrate_cmd is None:
                logger.error("不支持的 CAN 波特率: %d", self.can_bitrate)
                self._serial.close()
                self._serial = None
                return False
            self._slcan_cmd(bitrate_cmd)
            time.sleep(0.02)

            # 打开 CAN 通道
            self._slcan_cmd(b"O")
            time.sleep(0.02)

            # 清空接收缓冲区
            self._serial.reset_input_buffer()

            logger.info("SLCAN 接口 %s 已连接 (串口 %d bps, CAN %d bps)",
                        self.serial_port, self.serial_baudrate, self.can_bitrate)
            return True
        except Exception as e:
            logger.error("SLCAN 接口 %s 连接失败: %s", self.serial_port, e)
            if self._serial and self._serial.is_open:
                self._serial.close()
            self._serial = None
            return False

    def disconnect(self):
        """关闭 SLCAN 通道和串口"""
        self.stop_receive_thread()
        if self._serial and self._serial.is_open:
            try:
                self._slcan_cmd(b"C")
                time.sleep(0.02)
            except Exception:
                pass
            self._serial.close()
            logger.info("SLCAN 接口 %s 已断开", self.serial_port)
        self._serial = None

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def _slcan_cmd(self, cmd: bytes):
        """发送 SLCAN 命令 (自动追加 \\r)"""
        if self._serial and self._serial.is_open:
            self._serial.write(cmd + b"\r")

    # ========== 接收线程 ==========

    def start_receive_thread(self):
        if self._recv_running:
            return
        self._recv_running = True
        self._recv_thread = threading.Thread(
            target=self._receive_loop, daemon=True,
            name=f"slcan_recv_{self.serial_port}",
        )
        self._recv_thread.start()
        logger.info("SLCAN 接收线程已启动: %s", self.serial_port)

    def stop_receive_thread(self):
        self._recv_running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
            self._recv_thread = None
            logger.info("SLCAN 接收线程已停止: %s", self.serial_port)

    def _receive_loop(self):
        buf = b""
        while self._recv_running:
            try:
                if not self._serial or not self._serial.is_open:
                    time.sleep(0.01)
                    continue
                chunk = self._serial.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\r" in buf:
                    line, buf = buf.split(b"\r", 1)
                    if line:
                        self._parse_slcan_line(line)
            except serial.SerialTimeoutException:
                continue
            except Exception as e:
                if self._recv_running:
                    logger.warning("SLCAN 接收错误 [%s]: %s", self.serial_port, e)
                    time.sleep(0.01)

    def _parse_slcan_line(self, line: bytes):
        """解析单条 SLCAN ASCII 帧"""
        if len(line) < 1:
            return

        frame_type = line[0:1]

        if frame_type == b"T":
            # 扩展帧: T<8hex_id><1dlc><2*dlc hex_data>
            if len(line) < 10:
                return
            try:
                can_id = int(line[1:9], 16)
                dlc = int(line[9:10], 16)
                if dlc < 0 or dlc > 8:
                    return
                expected_len = 10 + dlc * 2
                if len(line) < expected_len:
                    return
                data = bytes.fromhex(line[10:10 + dlc * 2].decode("ascii"))
            except (ValueError, UnicodeDecodeError):
                return
            self._dispatch_frame(can_id & CAN_EFF_MASK, data)

        elif frame_type == b"t":
            # 标准帧: t<3hex_id><1dlc><2*dlc hex_data>
            if len(line) < 5:
                return
            try:
                can_id = int(line[1:4], 16)
                dlc = int(line[4:5], 16)
                if dlc < 0 or dlc > 8:
                    return
                expected_len = 5 + dlc * 2
                if len(line) < expected_len:
                    return
                data = bytes.fromhex(line[5:5 + dlc * 2].decode("ascii"))
            except (ValueError, UnicodeDecodeError):
                return
            self._dispatch_frame(can_id, data)

    def _dispatch_frame(self, can_id: int, data: bytes):
        """分发已解码的 CAN 帧到对应的解析器（与 RobstrideCanDriver._parse_frame 等价）"""
        comm_type = (can_id >> 24) & 0x1F
        motor_id = (can_id >> 8) & 0xFF

        if comm_type == CommType.FEEDBACK and 1 <= motor_id <= 16:
            if self._awaiting_version.get(motor_id, False):
                self._awaiting_version[motor_id] = False
                self._parse_version_response(data, motor_id)
            else:
                self._parse_motor_feedback(can_id, data, motor_id)
        elif comm_type == CommType.READ_PARAM:
            self._parse_param_read_response(can_id, data, motor_id)
        elif comm_type == CommType.FAULT_FEEDBACK:
            self._parse_fault_feedback(data, motor_id)

        self._frame_count += 1
        now = time.time()
        elapsed = now - self._fps_start_time
        if elapsed >= 1.0:
            self._current_fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start_time = now

    # ========== 帧解析 (与 RobstrideCanDriver 逻辑一致) ==========

    def _parse_motor_feedback(self, can_id: int, data: bytes, motor_id: int):
        motor_type = self.motor_type_map.get(motor_id, MotorType.RS00)
        params = MOTOR_PARAMS[motor_type]

        if len(data) < 8:
            return

        pos_raw, vel_raw, torque_raw, temp_raw = struct.unpack(">HHHH", data[:8])

        fb = MotorFeedback()
        fb.motor_id = motor_id
        fb.position = uint16_to_float(pos_raw, params.p_min, params.p_max)
        fb.velocity = uint16_to_float(vel_raw, params.v_min, params.v_max)
        fb.torque = uint16_to_float(torque_raw, params.t_min, params.t_max)
        fb.temperature = temp_raw / 10.0
        fb.mode_state = (can_id >> 22) & 0x03
        fb.fault_code = (can_id >> 16) & 0x3F
        fb.is_valid = True
        fb.timestamp = time.time()

        with self._lock:
            self._feedbacks[motor_id] = fb

        if self._feedback_callback:
            try:
                self._feedback_callback(fb)
            except Exception:
                pass

    def _parse_param_read_response(self, can_id: int, data: bytes, motor_id: int):
        if len(data) < 8:
            return
        status = (can_id >> 16) & 0xFF
        param_index = struct.unpack("<H", data[0:2])[0]
        raw = bytes(data[4:8])
        value = struct.unpack("<f", raw)[0]

        result = ParamReadResult(
            motor_id=motor_id,
            param_index=param_index,
            value=value,
            success=(status == 0),
            timestamp=time.time(),
            raw_bytes=raw,
        )
        with self._lock:
            self._param_results[motor_id] = result

    def _parse_fault_feedback(self, data: bytes, motor_id: int):
        if len(data) < 4:
            return
        fault_val = struct.unpack("<I", data[0:4])[0]
        with self._lock:
            self._fault_details[motor_id] = fault_val

    def _parse_version_response(self, data: bytes, motor_id: int):
        if len(data) < 7:
            return
        version_bytes = data[2:7]
        version_str = ".".join(str(b) for b in version_bytes)
        with self._lock:
            self._firmware_versions[motor_id] = FirmwareVersion(
                motor_id=motor_id,
                version_bytes=version_bytes,
                version_str=version_str,
                timestamp=time.time(),
            )

    # ========== 帧构造与发送 ==========

    def _build_extended_can_id(self, comm_type: int, data_area2: int, target_id: int) -> int:
        """构建 29 位扩展 CAN ID (不含 EFF 标志，SLCAN 用 'T' 前缀区分)"""
        return ((comm_type & 0x1F) << 24) | ((data_area2 & 0xFFFF) << 8) | (target_id & 0xFF)

    def _send_frame(self, can_id: int, data: bytes, retries: int = 5) -> bool:
        """通过 SLCAN 发送扩展 CAN 帧，带重试"""
        if not self._serial or not self._serial.is_open:
            return False

        # 去掉可能存在的 EFF 标志位
        can_id_29 = can_id & CAN_EFF_MASK
        data = data.ljust(8, b"\x00")[:8]
        hex_data = data.hex().upper()
        slcan_frame = f"T{can_id_29:08X}8{hex_data}\r".encode("ascii")

        with self._send_lock:
            for attempt in range(retries):
                try:
                    self._serial.write(slcan_frame)
                    self._tx_ok_count += 1
                    self._tx_ok_window_count += 1
                    return True
                except serial.SerialTimeoutException:
                    _busy_wait_us(200 << attempt)
                    continue
                except Exception as e:
                    if attempt < retries - 1:
                        _busy_wait_us(200 << attempt)
                        continue
                    logger.error("SLCAN 发送帧失败 [%s]: %s", self.serial_port, e)
                    self._tx_fail_count += 1
                    self._tx_fail_window_count += 1
                    self._maybe_warn_tx_fail()
                    return False
        self._tx_fail_count += 1
        self._tx_fail_window_count += 1
        self._maybe_warn_tx_fail()
        return False

    def _maybe_warn_tx_fail(self):
        now = time.time()
        window = now - self._tx_window_start
        if window >= 1.0:
            total = self._tx_ok_window_count + self._tx_fail_window_count
            self._tx_fail_rate = (
                self._tx_fail_window_count / total if total > 0 else 0.0
            )
            self._tx_ok_window_count = 0
            self._tx_fail_window_count = 0
            self._tx_window_start = now

        if self._tx_fail_rate > 0.05 and now - self._tx_warn_time > 5.0:
            self._tx_warn_time = now
            logger.warning(
                "SLCAN TX 失败率 %.1f%% [%s] (累计成功 %d / 失败 %d)",
                self._tx_fail_rate * 100, self.serial_port,
                self._tx_ok_count, self._tx_fail_count,
            )

    # ========== 电机控制指令 ==========

    def enable_motor(self, motor_id: int) -> bool:
        can_id = self._build_extended_can_id(CommType.ENABLE, self.host_can_id, motor_id)
        return self._send_frame(can_id, bytes(8))

    def disable_motor(self, motor_id: int, clear_fault: bool = False) -> bool:
        can_id = self._build_extended_can_id(CommType.DISABLE, self.host_can_id, motor_id)
        data = bytes([1 if clear_fault else 0]) + bytes(7)
        return self._send_frame(can_id, data)

    def set_zero_position(self, motor_id: int) -> bool:
        can_id = self._build_extended_can_id(CommType.SET_ZERO, self.host_can_id, motor_id)
        data = bytes([1]) + bytes(7)
        return self._send_frame(can_id, data)

    def send_motion_control(self, motor_id: int,
                            position: float, velocity: float,
                            kp: float, kd: float,
                            torque: float = 0.0) -> bool:
        motor_type = self.motor_type_map.get(motor_id, MotorType.RS00)
        params = MOTOR_PARAMS[motor_type]

        pos_raw = float_to_uint16(position, params.p_min, params.p_max)
        vel_raw = float_to_uint16(velocity, params.v_min, params.v_max)
        kp_raw = float_to_uint16(kp, params.kp_min, params.kp_max)
        kd_raw = float_to_uint16(kd, params.kd_min, params.kd_max)
        torque_raw = float_to_uint16(torque, params.t_min, params.t_max)

        can_id = self._build_extended_can_id(CommType.MOTION_CONTROL, torque_raw, motor_id)
        data = struct.pack(">HHHH", pos_raw, vel_raw, kp_raw, kd_raw)
        return self._send_frame(can_id, data)

    # ========== 参数读写 ==========

    def write_parameter(self, motor_id: int, param_index: int, value: float) -> bool:
        can_id = self._build_extended_can_id(CommType.WRITE_PARAM, self.host_can_id, motor_id)
        value_bytes = struct.pack("<f", value)
        data = struct.pack("<HBx", param_index, 0) + value_bytes
        return self._send_frame(can_id, data)

    def write_parameter_int(self, motor_id: int, param_index: int, value: int) -> bool:
        can_id = self._build_extended_can_id(CommType.WRITE_PARAM, self.host_can_id, motor_id)
        value_bytes = struct.pack("<I", value & 0xFFFFFFFF)
        data = struct.pack("<HBx", param_index, 0) + value_bytes
        return self._send_frame(can_id, data)

    def read_parameter(self, motor_id: int, param_index: int,
                       timeout: float = 0.5) -> Optional[ParamReadResult]:
        with self._lock:
            self._param_results.pop(motor_id, None)

        can_id = self._build_extended_can_id(CommType.READ_PARAM, self.host_can_id, motor_id)
        data = struct.pack("<HBx", param_index, 0) + bytes(4)
        if not self._send_frame(can_id, data):
            return None

        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                result = self._param_results.get(motor_id)
                if result and result.param_index == param_index:
                    return result
            time.sleep(0.005)
        return None

    def set_run_mode(self, motor_id: int, mode: RunMode) -> bool:
        return self.write_parameter_int(motor_id, ParamIndex.RUN_MODE, int(mode))

    def set_velocity_limit(self, motor_id: int, limit: float) -> bool:
        return self.write_parameter(motor_id, ParamIndex.LIMIT_SPD, limit)

    def set_position_csp(self, motor_id: int, position: float) -> bool:
        return self.write_parameter(motor_id, ParamIndex.LOC_REF, position)

    def set_pp_velocity(self, motor_id: int, vel_max: float) -> bool:
        return self.write_parameter(motor_id, ParamIndex.VEL_MAX, vel_max)

    def set_pp_acceleration(self, motor_id: int, acc_set: float) -> bool:
        return self.write_parameter(motor_id, ParamIndex.ACC_SET, acc_set)

    def set_position_pp(self, motor_id: int, position: float) -> bool:
        return self.write_parameter(motor_id, ParamIndex.LOC_REF, position)

    def save_parameters(self, motor_id: int) -> bool:
        can_id = self._build_extended_can_id(CommType.SAVE_PARAMS, self.host_can_id, motor_id)
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        return self._send_frame(can_id, data)

    # ========== 查询指令 ==========

    def query_firmware_version(self, motor_id: int,
                               timeout: float = 0.5) -> Optional[FirmwareVersion]:
        with self._lock:
            self._firmware_versions.pop(motor_id, None)

        self._awaiting_version[motor_id] = True
        can_id = self._build_extended_can_id(CommType.READ_VERSION, self.host_can_id, motor_id)
        data = bytes([0x00, 0xC4]) + bytes(6)
        if not self._send_frame(can_id, data):
            return None

        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                result = self._firmware_versions.get(motor_id)
                if result:
                    return result
            time.sleep(0.005)
        return None

    # ========== 反馈数据访问 ==========

    def get_feedback(self, motor_id: int) -> Optional[MotorFeedback]:
        with self._lock:
            return self._feedbacks.get(motor_id)

    def get_all_feedbacks(self) -> Dict[int, MotorFeedback]:
        with self._lock:
            return dict(self._feedbacks)

    def get_fault_detail(self, motor_id: int) -> int:
        with self._lock:
            return self._fault_details.get(motor_id, 0)

    def get_can_fps(self) -> float:
        if time.time() - self._fps_start_time > 2.0:
            return 0.0
        return self._current_fps

    def get_tx_stats(self):
        return self._tx_ok_count, self._tx_fail_count, self._tx_fail_rate

    def check_bus_health(self) -> str:
        """SLCAN 模式下通过 'F' 命令查询状态标志，降级返回"""
        if not self._serial or not self._serial.is_open:
            return "UNKNOWN"
        try:
            with self._send_lock:
                self._serial.write(b"F\r")
                time.sleep(0.02)
                resp = self._serial.read(10)
            if resp and resp.startswith(b"F"):
                flags = int(resp[1:3], 16) if len(resp) >= 3 else 0
                if flags & 0x20:
                    return "BUS-OFF"
                if flags & 0x40:
                    return "ERROR-PASSIVE"
                return "ERROR-ACTIVE"
        except Exception:
            pass
        return "UNKNOWN"

    @property
    def is_bus_healthy(self) -> bool:
        state = self.check_bus_health()
        return state in ("ERROR-ACTIVE", "UNKNOWN")

    def set_feedback_callback(self, callback: Optional[Callable[[MotorFeedback], None]]):
        self._feedback_callback = callback
