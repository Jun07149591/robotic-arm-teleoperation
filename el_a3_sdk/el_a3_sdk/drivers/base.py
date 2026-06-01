"""Common CAN driver contracts and factory helpers."""

from typing import Callable, Dict, Optional, Protocol

from el_a3_sdk.data_types import FirmwareVersion, MotorFeedback, ParamReadResult
from el_a3_sdk.protocol import MotorType, RunMode


class CanDriverProtocol(Protocol):
    """Protocol implemented by SocketCAN and SLCAN driver backends."""

    can_name: str

    def connect(self) -> bool: ...

    def disconnect(self): ...

    @property
    def is_connected(self) -> bool: ...

    def start_receive_thread(self): ...

    def stop_receive_thread(self): ...

    def enable_motor(self, motor_id: int) -> bool: ...

    def disable_motor(self, motor_id: int, clear_fault: bool = False) -> bool: ...

    def set_zero_position(self, motor_id: int) -> bool: ...

    def send_motion_control(
        self,
        motor_id: int,
        position: float,
        velocity: float,
        kp: float,
        kd: float,
        torque: float,
    ) -> bool: ...

    def write_parameter(self, motor_id: int, param_index: int, value: float) -> bool: ...

    def write_parameter_int(self, motor_id: int, param_index: int, value: int) -> bool: ...

    def read_parameter(
        self,
        motor_id: int,
        param_index: int,
        timeout: float = 0.1,
    ) -> Optional[ParamReadResult]: ...

    def set_run_mode(self, motor_id: int, mode: RunMode) -> bool: ...

    def set_velocity_limit(self, motor_id: int, limit: float) -> bool: ...

    def set_position_csp(self, motor_id: int, position: float) -> bool: ...

    def set_pp_velocity(self, motor_id: int, vel_max: float) -> bool: ...

    def set_pp_acceleration(self, motor_id: int, acc_set: float) -> bool: ...

    def set_position_pp(self, motor_id: int, position: float) -> bool: ...

    def save_parameters(self, motor_id: int) -> bool: ...

    def query_firmware_version(
        self,
        motor_id: int,
        timeout: float = 0.2,
    ) -> Optional[FirmwareVersion]: ...

    def get_feedback(self, motor_id: int) -> Optional[MotorFeedback]: ...

    def get_all_feedbacks(self) -> Dict[int, MotorFeedback]: ...

    def get_fault_detail(self, motor_id: int) -> int: ...

    def get_can_fps(self) -> float: ...

    def get_tx_stats(self): ...

    def check_bus_health(self) -> str: ...

    @property
    def is_bus_healthy(self) -> bool: ...

    def set_feedback_callback(
        self,
        callback: Optional[Callable[[MotorFeedback], None]],
    ): ...


def create_can_driver(
    *,
    backend: str = "socketcan",
    can_name: str = "can0",
    host_can_id: int = 0xFD,
    motor_type_map: Optional[Dict[int, MotorType]] = None,
    serial_port: Optional[str] = None,
    serial_baudrate: int = 2000000,
    can_bitrate: int = 1000000,
) -> CanDriverProtocol:
    """Create a CAN driver backend without importing optional backends early."""

    normalized = (backend or "socketcan").strip().lower()

    if normalized in ("socketcan", "socket", "can"):
        from el_a3_sdk.drivers.socketcan import RobstrideCanDriver

        return RobstrideCanDriver(
            can_name=can_name,
            host_can_id=host_can_id,
            motor_type_map=motor_type_map,
        )

    if normalized == "slcan":
        from el_a3_sdk.drivers.slcan import SlcanCanDriver

        return SlcanCanDriver(
            serial_port=serial_port or can_name,
            host_can_id=host_can_id,
            motor_type_map=motor_type_map,
            serial_baudrate=serial_baudrate,
            can_bitrate=can_bitrate,
        )

    raise ValueError(f"Unsupported CAN backend: {backend!r}")


__all__ = ["CanDriverProtocol", "create_can_driver"]
