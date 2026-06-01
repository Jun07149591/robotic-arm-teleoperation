"""
EL-A3 SDK 协议定义

基于 Robstride 私有协议（CAN 2.0 扩展帧，29 位 ID，1 Mbps）。
29 位 ID 结构：
    Bit28~24: 通信类型 (0-26)
    Bit23~8:  数据区2（主机 CAN ID / 前馈力矩等）
    Bit7~0:   目标地址（电机 CAN ID）
"""

from enum import IntEnum
import logging


class LogLevel(IntEnum):
    DEBUG = logging.DEBUG
    INFO = logging.INFO
    WARNING = logging.WARNING
    ERROR = logging.ERROR
    CRITICAL = logging.CRITICAL
    SILENT = 100


class CommType(IntEnum):
    """Robstride 私有协议通信类型"""
    GET_DEVICE_ID = 0
    MOTION_CONTROL = 1
    FEEDBACK = 2
    ENABLE = 3
    DISABLE = 4
    SET_ZERO = 6
    SET_CAN_ID = 7
    READ_PARAM = 17
    WRITE_PARAM = 18
    FAULT_FEEDBACK = 21
    SAVE_PARAMS = 22
    SET_BAUDRATE = 23
    SET_AUTO_REPORT = 24
    SET_PROTOCOL = 25
    READ_VERSION = 26


class MotorType(IntEnum):
    """电机型号"""
    RS00 = 0   # 关节 1-3: ±14 Nm, ±33 rad/s
    EL05 = 1   # 关节 4-7(配置A): ±6 Nm,  ±50 rad/s
    RS05 = 2   # 关节 4-7(配置B): ±5.5 Nm, ±50 rad/s


class RunMode(IntEnum):
    """电机运行模式"""
    MOTION_CONTROL = 0   # 运控模式（PD + 前馈力矩）
    POSITION_PP = 1      # 位置模式 (PP，梯形规划)
    VELOCITY = 2         # 速度模式
    CURRENT = 3          # 电流模式
    POSITION_CSP = 5     # 位置模式 (CSP，连续位置)


class ArmState(IntEnum):
    """机械臂状态机"""
    DISCONNECTED = 0
    IDLE = 1         # Connected but motors disabled
    ENABLED = 2      # Motors enabled, ready for commands
    RUNNING = 3      # Active motion (JointCtrl/MoveJ/MoveL)
    ZERO_TORQUE = 4  # Zero-torque / gravity compensation
    ERROR = 5


class ControlMode(IntEnum):
    """控制模式（对标 Piper SDK ctrl_mode）"""
    STANDBY = 0x00
    CAN_COMMAND = 0x01


class MoveMode(IntEnum):
    """运动模式（对标 Piper SDK move_mode）"""
    MOVE_J = 0x00        # 关节运动（运控模式，PD 控制）
    MOVE_CSP = 0x01      # 连续位置运动（CSP 模式）
    MOVE_VELOCITY = 0x02 # 速度运动
    MOVE_CURRENT = 0x03  # 电流/力矩运动
    MOVE_L = 0x04        # 直线运动（笛卡尔插值 + IK）
    MOVE_C = 0x05        # 圆弧运动（预留）


class ModeState(IntEnum):
    """电机模式状态（Type 2 反馈帧 Bit22~23）"""
    RESET = 0
    CALIBRATION = 1
    MOTOR = 2


class FaultBit(IntEnum):
    """故障位定义（Type 2 反馈帧 Bit16~21 / Type 21 详细故障）"""
    UNDER_VOLTAGE = 0
    PHASE_CURRENT = 1
    OVER_VOLTAGE = 3
    B_PHASE_OVERCURRENT = 4
    C_PHASE_OVERCURRENT = 5
    ENCODER_UNCALIBRATED = 7
    HARDWARE_ID = 8
    POSITION_INIT = 9
    STALL_OVERLOAD = 14
    A_PHASE_OVERCURRENT = 16


# 参数索引（通信类型 17/18 使用）
class ParamIndex:
    RUN_MODE = 0x7005
    IQ_REF = 0x7006
    SPD_REF = 0x700A
    LIMIT_TORQUE = 0x700B
    CUR_KP = 0x7010
    CUR_KI = 0x7011
    CUR_FILT_GAIN = 0x7014
    LOC_REF = 0x7016
    LIMIT_SPD = 0x7017
    LIMIT_CUR = 0x7018
    MECH_POS = 0x7019
    IQF = 0x701A
    MECH_VEL = 0x701B
    VBUS = 0x701C
    LOC_KP = 0x701E
    SPD_KP = 0x701F
    SPD_KI = 0x7020
    SPD_FILT_GAIN = 0x7021
    ACC_RAD = 0x7022
    VEL_MAX = 0x7024
    ACC_SET = 0x7025
    EPSCAN_TIME = 0x7026
    CAN_TIMEOUT = 0x7028
    ZERO_STA = 0x7029
    ADD_OFFSET = 0x702B


class MotorParams:
    """电机参数范围"""

    def __init__(self, p_min=-12.57, p_max=12.57,
                 v_min=-50.0, v_max=50.0,
                 t_min=-12.0, t_max=12.0,
                 kp_min=0.0, kp_max=500.0,
                 kd_min=0.0, kd_max=5.0):
        self.p_min = p_min
        self.p_max = p_max
        self.v_min = v_min
        self.v_max = v_max
        self.t_min = t_min
        self.t_max = t_max
        self.kp_min = kp_min
        self.kp_max = kp_max
        self.kd_min = kd_min
        self.kd_max = kd_max


MOTOR_PARAMS = {
    MotorType.RS00: MotorParams(v_min=-33.0, v_max=33.0, t_min=-14.0, t_max=14.0),
    MotorType.EL05: MotorParams(v_min=-50.0, v_max=50.0, t_min=-6.0, t_max=6.0),
    MotorType.RS05: MotorParams(v_min=-50.0, v_max=50.0, t_min=-5.5, t_max=5.5),
}

# 默认电机 ID -> 类型映射（1-3: RS00, 4-6: EL05）
DEFAULT_MOTOR_TYPE_MAP = {
    1: MotorType.RS00,
    2: MotorType.RS00,
    3: MotorType.RS00,
    4: MotorType.EL05,
    5: MotorType.EL05,
    6: MotorType.EL05,
    7: MotorType.EL05,
}

# 默认关节方向（与 xacro 配置一致）
DEFAULT_JOINT_DIRECTIONS = {
    1: -1.0,  # L1
    2:  1.0,  # L2
    3: -1.0,  # L3
    4:  1.0,  # L4
    5: -1.0,  # L5
    6:  1.0,  # L6
    7:  1.0,  # L7 (Gripper)
}

# 默认关节偏移（rad）
DEFAULT_JOINT_OFFSETS = {
    1: 0.0,
    2: 0.0,
    3: 0.0,
    4: 0.0,
    5: 0.0,
    6: 0.0,
    7: 0.0,
}

# 默认关节限位（rad）- 与 URDF/xacro 及 joint_limits.yaml 一致
DEFAULT_JOINT_LIMITS = {
    1: (-2.79253, 2.79253),  # L1: ±160°
    2: (0.0, 3.66519),       # L2: 0°~210°
    3: (-4.01426, 0.0),      # L3: -230°~0°
    4: (-1.5708, 1.5708),    # L4: ±90°
    5: (-1.5708, 1.5708),    # L5: ±90°
    6: (-1.5708, 1.5708),    # L6: ±90°
    7: (-1.5708, 1.5708),    # L7: ±90° (Gripper)
}
