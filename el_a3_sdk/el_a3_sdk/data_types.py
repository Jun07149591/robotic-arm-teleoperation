"""
EL-A3 SDK 数据结构定义

对标 Piper SDK 的消息类型，但使用 SI 单位（rad, rad/s, Nm, m）。
"""

from dataclasses import dataclass, field
from typing import List, Optional
import time


@dataclass
class MotorFeedback:
    """单个电机反馈数据（来自 Type 2 反馈帧）"""
    motor_id: int = 0
    position: float = 0.0       # rad（电机坐标系）
    velocity: float = 0.0       # rad/s
    torque: float = 0.0         # Nm
    temperature: float = 0.0    # °C
    mode_state: int = 0         # 0=Reset, 1=Cali, 2=Motor
    fault_code: int = 0         # 6 位故障码
    is_valid: bool = False
    timestamp: float = 0.0      # 反馈时间戳


@dataclass
class ArmJointStates:
    """
    机械臂关节状态（对标 Piper GetArmJointMsgs）

    所有角度单位为 rad。支持 6 或 7 关节（joint_7 为夹爪）。
    """
    joint_1: float = 0.0
    joint_2: float = 0.0
    joint_3: float = 0.0
    joint_4: float = 0.0
    joint_5: float = 0.0
    joint_6: float = 0.0
    joint_7: float = 0.0
    timestamp: float = 0.0
    hz: float = 0.0

    def to_list(self, include_gripper: bool = True) -> List[float]:
        base = [self.joint_1, self.joint_2, self.joint_3,
                self.joint_4, self.joint_5, self.joint_6]
        if include_gripper:
            base.append(self.joint_7)
        return base

    @classmethod
    def from_list(cls, values: List[float], timestamp: float = 0.0):
        return cls(
            joint_1=values[0] if len(values) > 0 else 0.0,
            joint_2=values[1] if len(values) > 1 else 0.0,
            joint_3=values[2] if len(values) > 2 else 0.0,
            joint_4=values[3] if len(values) > 3 else 0.0,
            joint_5=values[4] if len(values) > 4 else 0.0,
            joint_6=values[5] if len(values) > 5 else 0.0,
            joint_7=values[6] if len(values) > 6 else 0.0,
            timestamp=timestamp,
        )


@dataclass
class ArmEndPose:
    """
    机械臂末端位姿（对标 Piper GetArmEndPoseMsgs）

    位置单位 m，姿态单位 rad。
    """
    x: float = 0.0    # m
    y: float = 0.0    # m
    z: float = 0.0    # m
    rx: float = 0.0   # rad
    ry: float = 0.0   # rad
    rz: float = 0.0   # rad
    timestamp: float = 0.0


@dataclass
class ArmStatus:
    """
    机械臂状态（对标 Piper GetArmStatus）

    汇总所有电机的状态信息。
    """
    ctrl_mode: int = 0          # 0=Standby, 1=CAN 控制
    arm_status: int = 0         # 0=正常, 1=急停, ...
    move_mode: int = 0          # 当前运动模式
    motion_status: int = 0      # 0=到达, 1=运动中
    timestamp: float = 0.0

    # 各关节使能状态（7 个电机: L1-L6 + L7 夹爪）
    joint_enabled: List[bool] = field(default_factory=lambda: [False] * 7)
    # 各关节故障码
    joint_faults: List[int] = field(default_factory=lambda: [0] * 7)
    # 各关节模式状态
    joint_mode_states: List[int] = field(default_factory=lambda: [0] * 7)

    @property
    def has_fault(self) -> bool:
        return any(f != 0 for f in self.joint_faults)

    @property
    def all_enabled(self) -> bool:
        return all(self.joint_enabled)


@dataclass
class MotorHighSpdInfo:
    """
    电机高速反馈信息（对标 Piper GetArmHighSpdInfoMsgs）

    从 Type 2 反馈帧直接获取。
    """
    motor_id: int = 0
    speed: float = 0.0        # rad/s
    current: float = 0.0      # A（需通过参数读取获得精确值）
    position: float = 0.0     # rad
    torque: float = 0.0       # Nm
    timestamp: float = 0.0


@dataclass
class MotorLowSpdInfo:
    """
    电机低速反馈信息（对标 Piper GetArmLowSpdInfoMsgs）

    需通过参数读取（Type 17）获取。
    """
    motor_id: int = 0
    voltage: float = 0.0       # V (VBUS)
    driver_temp: float = 0.0   # °C（暂不支持，Robstride 仅反馈绕组温度）
    motor_temp: float = 0.0    # °C
    fault_code: int = 0
    bus_current: float = 0.0   # A
    timestamp: float = 0.0


@dataclass
class MotorAngleLimitMaxVel:
    """电机角度限制与最大速度（对标 Piper GetCurrentMotorAngleLimitMaxVel）"""
    motor_num: int = 0
    max_angle_limit: float = 0.0   # rad
    min_angle_limit: float = 0.0   # rad
    max_joint_spd: float = 0.0     # rad/s


@dataclass
class MotorMaxAccLimit:
    """电机最大加速度限制"""
    motor_num: int = 0
    max_joint_acc: float = 0.0     # rad/s²


@dataclass
class ParamReadResult:
    """参数读取结果"""
    motor_id: int = 0
    param_index: int = 0
    value: float = 0.0
    success: bool = False
    timestamp: float = 0.0
    raw_bytes: bytes = b"\x00\x00\x00\x00"

    @property
    def value_uint8(self) -> int:
        return self.raw_bytes[0] if self.raw_bytes else 0

    @property
    def value_uint16(self) -> int:
        import struct as _st
        return _st.unpack_from("<H", self.raw_bytes, 0)[0] if len(self.raw_bytes) >= 2 else 0

    @property
    def value_uint32(self) -> int:
        import struct as _st
        return _st.unpack_from("<I", self.raw_bytes, 0)[0] if len(self.raw_bytes) >= 4 else 0


@dataclass
class FirmwareVersion:
    """固件版本信息"""
    motor_id: int = 0
    version_bytes: bytes = b""
    version_str: str = ""
    timestamp: float = 0.0


@dataclass
class DynamicsInfo:
    """动力学信息"""
    gravity_torques: List[float] = field(default_factory=list)
    mass_matrix: Optional[object] = None      # np.ndarray (nv x nv)
    jacobian: Optional[object] = None          # np.ndarray (6 x nv)
    coriolis_torques: List[float] = field(default_factory=list)
    timestamp: float = 0.0


@dataclass
class TrajectoryResult:
    """轨迹执行结果"""
    success: bool = False
    error_code: int = 0
    actual_positions: List[float] = field(default_factory=list)
    message: str = ""
    elapsed_time: float = 0.0
