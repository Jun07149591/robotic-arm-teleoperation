"""
EL-A3 SDK CAN 驱动层

基于 Linux SocketCAN 的 Robstride 电机通信驱动。
从 scripts/teleop_master_slave.py 中的 RobstrideCanInterface 重构而来，
增加了参数读取、版本查询、故障帧解析等功能。
"""

import select
import socket
import struct
import time
import threading
import logging
from typing import Dict, Optional, Callable

from el_a3_sdk.protocol import (
    CommType, MotorType, RunMode, MotorParams, ParamIndex,
    MOTOR_PARAMS, DEFAULT_MOTOR_TYPE_MAP,
)
from el_a3_sdk.data_types import (
    MotorFeedback, ParamReadResult, FirmwareVersion,
)
from el_a3_sdk.drivers.timing import busy_wait_us
from el_a3_sdk.utils import float_to_uint16, uint16_to_float

logger = logging.getLogger("el_a3_sdk.drivers.socketcan")


# CAN 帧结构: can_id (4B) + dlc (1B) + padding (3B) + data (8B) = 16B
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = 16
CAN_EFF_FLAG = 0x80000000   # 扩展帧标志
CAN_EFF_MASK = 0x1FFFFFFF   # 29 位 ID 掩码

SOL_CAN_RAW = 101            # <linux/can/raw.h>
CAN_RAW_RECV_OWN_MSGS = 4    # 禁用后不再接收自身发出帧的回环


class RobstrideCanDriver:
    """
    Robstride 电机 Socket CAN 底层驱动

    负责：
    - CAN socket 管理（连接/断开）
    - 帧收发（带重试）
    - 后台接收线程 + 反馈帧解析
    - 电机使能/失能/设零/运控指令
    - 参数读写
    """

    def __init__(self, can_name: str = "can0", host_can_id: int = 0xFD,
                 motor_type_map: Optional[Dict[int, MotorType]] = None):
        self.can_name = can_name
        self.host_can_id = host_can_id
        self.motor_type_map = motor_type_map or dict(DEFAULT_MOTOR_TYPE_MAP)

        self._socket: Optional[socket.socket] = None
        self._feedbacks: Dict[int, MotorFeedback] = {}
        self._param_results: Dict[int, ParamReadResult] = {}
        self._firmware_versions: Dict[int, FirmwareVersion] = {}
        self._fault_details: Dict[int, int] = {}
        self._awaiting_version: Dict[int, bool] = {}

        self._recv_running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()

        # 反馈帧回调（可选，用于外部监听）
        self._feedback_callback: Optional[Callable[[MotorFeedback], None]] = None

        # 帧率统计
        self._frame_count = 0
        self._fps_start_time = time.time()
        self._current_fps = 0.0

        # TX 发送统计
        self._tx_ok_count = 0
        self._tx_fail_count = 0
        self._tx_fail_window_count = 0
        self._tx_ok_window_count = 0
        self._tx_window_start = time.time()
        self._tx_fail_rate = 0.0
        self._tx_warn_time = 0.0

    # ========== 连接管理 ==========

    def connect(self) -> bool:
        """创建并绑定 Socket CAN"""
        try:
            self._socket = socket.socket(
                socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW
            )
            self._socket.bind((self.can_name,))
            self._socket.setsockopt(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                                    struct.pack("i", 0))
            self._socket.settimeout(0.1)
            logger.info("CAN 接口 %s 已连接", self.can_name)
            return True
        except Exception as e:
            logger.error("CAN 接口 %s 连接失败: %s", self.can_name, e)
            self._socket = None
            return False

    def disconnect(self):
        """关闭 CAN socket"""
        self.stop_receive_thread()
        if self._socket:
            self._socket.close()
            self._socket = None
            logger.info("CAN 接口 %s 已断开", self.can_name)

    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    # ========== 接收线程 ==========

    def start_receive_thread(self):
        """启动后台接收线程"""
        if self._recv_running:
            return
        self._recv_running = True
        self._recv_thread = threading.Thread(
            target=self._receive_loop, daemon=True, name=f"can_recv_{self.can_name}"
        )
        self._recv_thread.start()
        logger.info("接收线程已启动: %s", self.can_name)

    def stop_receive_thread(self):
        """停止接收线程"""
        self._recv_running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
            self._recv_thread = None
            logger.info("接收线程已停止: %s", self.can_name)

    def _receive_loop(self):
        while self._recv_running:
            try:
                frame = self._socket.recv(CAN_FRAME_SIZE)
                if len(frame) == CAN_FRAME_SIZE:
                    self._parse_frame(frame)
            except (socket.timeout, TimeoutError):
                continue
            except Exception as e:
                if self._recv_running:
                    logger.warning("接收帧错误 [%s]: %s", self.can_name, e)

    def _parse_frame(self, raw: bytes):
        can_id_raw, dlc = struct.unpack("=IB", raw[:5])
        data = raw[8:16]

        if not (can_id_raw & CAN_EFF_FLAG):
            return
        can_id = can_id_raw & CAN_EFF_MASK

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

        # 帧率统计
        self._frame_count += 1
        now = time.time()
        elapsed = now - self._fps_start_time
        if elapsed >= 1.0:
            self._current_fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start_time = now

    def _parse_motor_feedback(self, can_id: int, data: bytes, motor_id: int):
        """解析 Type 2 电机反馈帧"""
        motor_type = self.motor_type_map.get(motor_id, MotorType.RS00)
        params = MOTOR_PARAMS[motor_type]

        pos_raw, vel_raw, torque_raw, temp_raw = struct.unpack(">HHHH", data)

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
        """解析 Type 17 参数读取应答"""
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
        """解析 Type 21 故障反馈帧"""
        fault_val = struct.unpack("<I", data[0:4])[0]
        with self._lock:
            self._fault_details[motor_id] = fault_val

    def _parse_version_response(self, data: bytes, motor_id: int):
        """解析 Type 26 版本号应答"""
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
        """构建 29 位扩展 CAN ID + EFF 标志"""
        can_id = ((comm_type & 0x1F) << 24) | ((data_area2 & 0xFFFF) << 8) | (target_id & 0xFF)
        return can_id | CAN_EFF_FLAG

    def _send_frame(self, can_id: int, data: bytes, retries: int = 5) -> bool:
        """发送 CAN 帧，带指数退避重试 (200us → 3.2ms)"""
        if not self._socket:
            return False
        frame = struct.pack(CAN_FRAME_FMT, can_id, 8, data.ljust(8, b"\x00"))
        with self._send_lock:
            for attempt in range(retries):
                try:
                    self._socket.send(frame)
                    self._tx_ok_count += 1
                    self._tx_ok_window_count += 1
                    return True
                except (socket.timeout, TimeoutError):
                    busy_wait_us(200 << attempt)
                    continue
                except OSError as e:
                    if e.errno in (105, 11, 110):  # ENOBUFS / EAGAIN / ETIMEDOUT
                        busy_wait_us(200 << attempt)
                        continue
                    logger.error("发送 CAN 帧失败 [%s]: %s", self.can_name, e)
                    self._tx_fail_count += 1
                    self._tx_fail_window_count += 1
                    self._maybe_warn_tx_fail()
                    return False
        self._tx_fail_count += 1
        self._tx_fail_window_count += 1
        self._maybe_warn_tx_fail()
        return False

    def _maybe_warn_tx_fail(self):
        """滑动窗口统计失败率，超过 5% 时限频告警 (1 次/5秒)"""
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
                "CAN TX 失败率 %.1f%% [%s] (累计成功 %d / 失败 %d)",
                self._tx_fail_rate * 100, self.can_name,
                self._tx_ok_count, self._tx_fail_count,
            )

    # ========== 电机控制指令 ==========

    def enable_motor(self, motor_id: int) -> bool:
        """使能电机（Type 3）"""
        can_id = self._build_extended_can_id(CommType.ENABLE, self.host_can_id, motor_id)
        return self._send_frame(can_id, bytes(8))

    def disable_motor(self, motor_id: int, clear_fault: bool = False) -> bool:
        """停止电机（Type 4），可选清除故障"""
        can_id = self._build_extended_can_id(CommType.DISABLE, self.host_can_id, motor_id)
        data = bytes([1 if clear_fault else 0]) + bytes(7)
        return self._send_frame(can_id, data)

    def set_zero_position(self, motor_id: int) -> bool:
        """设置当前位置为零位（Type 6）"""
        can_id = self._build_extended_can_id(CommType.SET_ZERO, self.host_can_id, motor_id)
        data = bytes([1]) + bytes(7)
        return self._send_frame(can_id, data)

    def send_motion_control(self, motor_id: int,
                            position: float, velocity: float,
                            kp: float, kd: float,
                            torque: float = 0.0) -> bool:
        """
        发送运控模式指令（Type 1）

        Args:
            motor_id: 电机 ID
            position: 目标位置 (rad)，电机坐标系
            velocity: 目标速度 (rad/s)
            kp: 位置增益
            kd: 速度增益
            torque: 前馈力矩 (Nm)
        """
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
        """写入单个 float 参数（Type 18，掉电丢失）"""
        can_id = self._build_extended_can_id(CommType.WRITE_PARAM, self.host_can_id, motor_id)
        value_bytes = struct.pack("<f", value)
        data = struct.pack("<HBx", param_index, 0) + value_bytes
        return self._send_frame(can_id, data)

    def write_parameter_int(self, motor_id: int, param_index: int, value: int) -> bool:
        """写入整数参数（uint8/uint16/uint32，Type 18）

        run_mode (0x7005) 等 uint8 参数必须用此方法写入，
        否则 float 编码的低字节为 0x00 导致写入值错误。
        """
        can_id = self._build_extended_can_id(CommType.WRITE_PARAM, self.host_can_id, motor_id)
        value_bytes = struct.pack("<I", value & 0xFFFFFFFF)
        data = struct.pack("<HBx", param_index, 0) + value_bytes
        return self._send_frame(can_id, data)

    def read_parameter(self, motor_id: int, param_index: int,
                       timeout: float = 0.5) -> Optional[ParamReadResult]:
        """
        读取单个参数（Type 17），阻塞等待应答

        Args:
            motor_id: 电机 ID
            param_index: 参数索引
            timeout: 等待超时（秒）

        Returns:
            ParamReadResult or None
        """
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
        """设置运行模式（run_mode 为 uint8，必须用整数编码）"""
        return self.write_parameter_int(motor_id, ParamIndex.RUN_MODE, int(mode))

    def set_velocity_limit(self, motor_id: int, limit: float) -> bool:
        """设置 CSP 模式速度上限"""
        return self.write_parameter(motor_id, ParamIndex.LIMIT_SPD, limit)

    def set_position_csp(self, motor_id: int, position: float) -> bool:
        """设置 CSP 位置指令"""
        return self.write_parameter(motor_id, ParamIndex.LOC_REF, position)

    def set_pp_velocity(self, motor_id: int, vel_max: float) -> bool:
        """设置 PP 模式最大速度 (0x7024 vel_max, rad/s)"""
        return self.write_parameter(motor_id, ParamIndex.VEL_MAX, vel_max)

    def set_pp_acceleration(self, motor_id: int, acc_set: float) -> bool:
        """设置 PP 模式加速度 (0x7025 acc_set, rad/s²)"""
        return self.write_parameter(motor_id, ParamIndex.ACC_SET, acc_set)

    def set_position_pp(self, motor_id: int, position: float) -> bool:
        """设置 PP 位置指令 (0x7016 loc_ref, rad)"""
        return self.write_parameter(motor_id, ParamIndex.LOC_REF, position)

    def save_parameters(self, motor_id: int) -> bool:
        """保存所有参数到 flash（Type 22）"""
        can_id = self._build_extended_can_id(CommType.SAVE_PARAMS, self.host_can_id, motor_id)
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        return self._send_frame(can_id, data)

    # ========== 查询指令 ==========

    def query_firmware_version(self, motor_id: int,
                               timeout: float = 0.5) -> Optional[FirmwareVersion]:
        """查询电机固件版本（Type 26 请求，Type 2 应答）"""
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
        """获取单个电机反馈"""
        with self._lock:
            return self._feedbacks.get(motor_id)

    def get_all_feedbacks(self) -> Dict[int, MotorFeedback]:
        """获取所有电机反馈"""
        with self._lock:
            return dict(self._feedbacks)

    def get_fault_detail(self, motor_id: int) -> int:
        """获取电机详细故障码（Type 21）"""
        with self._lock:
            return self._fault_details.get(motor_id, 0)

    def get_can_fps(self) -> float:
        """获取 CAN 帧率（超过 2 秒无新帧自动归零）"""
        if time.time() - self._fps_start_time > 2.0:
            return 0.0
        return self._current_fps

    def get_tx_stats(self):
        """返回 (成功数, 失败数, 最近 1 秒失败率)"""
        return self._tx_ok_count, self._tx_fail_count, self._tx_fail_rate

    def check_bus_health(self) -> str:
        """返回 CAN 总线状态: 'ERROR-ACTIVE' / 'ERROR-PASSIVE' / 'BUS-OFF' / 'STOPPED' / 'UNKNOWN'"""
        try:
            with open(f"/sys/class/net/{self.can_name}/can_state") as f:
                return f.read().strip().upper()
        except (FileNotFoundError, PermissionError, OSError):
            return "UNKNOWN"

    @property
    def is_bus_healthy(self) -> bool:
        state = self.check_bus_health()
        return state in ("ERROR-ACTIVE", "UNKNOWN")

    def set_feedback_callback(self, callback: Optional[Callable[[MotorFeedback], None]]):
        """注册反馈帧回调"""
        self._feedback_callback = callback
