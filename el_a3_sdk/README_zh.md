# EL-A3 机械臂 Python SDK

> **EL-A3** 是一款 7 自由度（6 臂关节 + L7 夹爪）桌面级机械臂，通过 CAN 总线驱动 Robstride 电机。本 SDK 提供纯 Python 控制接口，内置 200Hz 后台控制循环（EMA 平滑、速度前馈、Pinocchio RNEA 重力补偿、自适应阻尼零力矩模式），支持 S-curve 轨迹规划、笛卡尔控制、多臂管理。**无需 ROS 依赖。**

---

## 目录

- [架构](#架构)
- [硬件要求](#硬件要求)
- [安装](#安装)
  - [Debugger 上位机](#3-安装-debugger-上位机可选)
- [快速开始](#快速开始)
- [控制循环](#控制循环)
- [API 参考](#api-参考)
- [电机通信协议](#电机通信协议)
- [故障排除](#故障排除)
- [目录结构](#目录结构)

---

## 架构

```
┌──────────────────────────────────────────────────────┐
│                     用户应用                           │
│   JointCtrl / MoveJ / MoveL / EndPoseCtrl / ...     │
└──────────────────────┬───────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────┐
│              ELA3Interface                             │
│         200Hz 后台控制循环                              │
│  ┌─────────┐ ┌──────────┐ ┌────────────┐            │
│  │ EMA     │ │ 速度前馈  │ │ Pinocchio  │            │
│  │ 位置平滑 │ │ 4-MA+EMA │ │ RNEA 重力  │            │
│  └─────────┘ └──────────┘ └────────────┘            │
│  ┌──────────────┐ ┌──────────────────────┐          │
│  │ 关节限位保护   │ │ 自适应 Kd 零力矩模式  │          │
│  └──────────────┘ └──────────────────────┘          │
│  ┌──────────────┐ ┌──────────────────────┐          │
│  │ S-curve 轨迹  │ │ 笛卡尔 IK (Pinocchio)│          │
│  └──────────────┘ └──────────────────────┘          │
└──────────────────────┬───────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────┐
│        drivers.create_can_driver()                    │
│        SocketCAN / SLCAN · CAN 2.0 扩展帧             │
└──────────────────────┬───────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────┐
│               Robstride 电机 (7x)                      │
│     τ = Kp(θ_t - θ) + Kd(ω_t - ω) + τ_ff            │
│     RS00 (L1-L3) + EL05/RS05 (L4-L7)                 │
└──────────────────────────────────────────────────────┘
```

### 状态机

```
DISCONNECTED ──ConnectPort()──▶ IDLE ──EnableArm()──▶ ENABLED
                                 ▲                      │  ▲
                          DisableArm()          JointCtrl()/MoveJ()
                                 │                      ▼  │
                                 │                   RUNNING
                                 │              ZeroTorqueMode(True)
                                 │                      ▼
                                 ├───────────── ZERO_TORQUE
                                 │         ZeroTorqueMode(False) → ENABLED
                          EmergencyStop()
                                 ▼
                               ERROR
```

---

## 硬件要求

- **EL-A3 机械臂** (7 个 Robstride 电机)
- **CAN 适配器**: CANdle / gs_usb 兼容设备
- **电源**: 24V/48V 直流电源
- **PC**: Ubuntu 22.04+ x86_64

### 电机配置

| 关节 | 电机ID | 型号 | 力矩限制 | 速度限制 | 位置限制 | 方向 |
|------|--------|------|----------|----------|----------|------|
| L1 | 1 | RS00 | ±14 Nm | ±33 rad/s | ±2.79 rad (±160°) | -1 |
| L2 | 2 | RS00 | ±14 Nm | ±33 rad/s | 0~3.67 rad (0°~210°) | +1 |
| L3 | 3 | RS00 | ±14 Nm | ±33 rad/s | -4.01~0 rad (-230°~0°) | -1 |
| L4 | 4 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |
| L5 | 5 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | -1 |
| L6 | 6 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |
| L7（夹爪） | 7 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |

---

## 安装

### 1. 设置 CAN 接口

```bash
sudo ip link set can0 up type can bitrate 1000000
# 或使用脚本
sudo ./scripts/setup_can.sh can0 1000000
```

### 2. 安装 SDK

```bash
cd el_a3_sdk
pip install -e .              # 基本安装
pip install -e ".[dynamics]"  # 含 Pinocchio 动力学支持
pip install -e ".[camera]"    # 含 RealSense 点云可视化/选点支持
```

依赖：`numpy`、`pyyaml`。可选：`pinocchio` (`pip install pin`) 用于 FK/IK/重力补偿；`pyrealsense2`、`open3d`、`opencv-python` 用于 RealSense 点云可视化/选点。

### 3. 安装 Debugger 上位机（可选）

Debugger 提供 PyQt6 GUI 界面，内含基于 **PyVista** (VTK) 的 3D URDF 可视化、关节拖拽控制、实时监控等功能。

```bash
# Ubuntu 系统依赖（OpenGL / Mesa，3D 渲染必需）
sudo apt install -y libgl1-mesa-glx libgl1-mesa-dev libxrender1 libxcb-xinerama0

# 安装 Debugger 全部 Python 依赖
pip install -e ".[debugger]"
```

这会安装以下包（及其依赖）：

| 包 | 用途 |
|---|---|
| `pyqt6` | GUI 框架 |
| `pyqtgraph` | 实时数据曲线绘制 |
| `pyvista` | 3D 网格渲染 / URDF 可视化（底层依赖 VTK） |
| `pyvistaqt` | 将 PyVista 渲染器嵌入 PyQt6 窗口 |

安装完成后即可启动：

```bash
el-a3-debugger
```

---

## 快速开始

### 基础关节控制

```python
from el_a3_sdk import ELA3Interface

arm = ELA3Interface(can_name="can0")
arm.ConnectPort()
arm.EnableArm()

# 直接关节控制
arm.JointCtrl(0.0, 0.5, -0.3, 0.0, 0.0, 0.0)

# 读取反馈
print(arm.GetArmJointMsgs())
print(arm.GetArmEndPoseMsgs())

arm.DisableArm()
arm.DisconnectPort()
```

### 使用后台控制循环（推荐）

```python
from el_a3_sdk import ELA3Interface
import math, time

arm = ELA3Interface(can_name="can0")
arm.ConnectPort()
arm.EnableArm()

# 启动 200Hz 后台控制循环（EMA 平滑 + 速度前馈 + 重力补偿 + 限位保护）
arm.start_control_loop(rate_hz=200.0)

# S-curve 轨迹运动（阻塞）
arm.MoveJ([0.0, math.radians(30), math.radians(-30), 0.0, 0.0, 0.0], duration=2.0)

# S-curve 轨迹运动（非阻塞）
arm.MoveJ([0.0] * 6, duration=2.0, block=False)
arm.wait_for_motion()

# 实时关节控制（由控制循环平滑发送）
arm.JointCtrl(0.0, 0.5, -0.3, 0.0, 0.0, 0.0)

# 笛卡尔控制
arm.EndPoseCtrl(0.3, 0.0, 0.3, 0.0, 0.0, 0.0, duration=2.0)

# 零力矩示教模式（自适应 Kd + Pinocchio 重力补偿）
arm.ZeroTorqueModeWithGravity(True, kd=0.5)
time.sleep(10)
arm.ZeroTorqueModeWithGravity(False)

arm.stop_control_loop()
arm.DisableArm()
arm.DisconnectPort()
```

### 多臂管理

```python
from el_a3_sdk import ArmManager

mgr = ArmManager()
master = mgr.register_can_arm("master", can_name="can0")
slave = mgr.register_can_arm("slave", can_name="can1")

# 或从配置文件批量创建
mgr = ArmManager.from_config("config/multi_arm_config.yaml", auto_connect=True)

mgr.disconnect_all()
```

---

## 控制循环

`start_control_loop()` 启动 200Hz 后台线程，移植自 C++ `el_a3_hardware` 的核心控制逻辑：

| 特性 | 说明 |
|------|------|
| **EMA 位置平滑** | `smoothed = alpha * target + (1-alpha) * smoothed`，避免阶跃跳变 |
| **速度前馈** | 4-sample 移动平均 + 2阶 EMA + 加速度限制 + 平滑死区 + tanh 软限幅 |
| **重力补偿** | Pinocchio RNEA 计算前馈力矩，`gravity_feedforward_ratio` 控制补偿比例 |
| **关节限位保护** | 接近限位时硬停止钳位 |
| **自适应 Kd 零力矩** | 洛伦兹衰减：`kd = kd_min + (kd_max - kd_min) / (1 + (v/v_ref)^2)`，EMA 平滑 |
| **轨迹队列** | MoveJ/MoveL 规划后推入控制循环异步执行，支持阻塞/非阻塞 |

### 构造参数

```python
ELA3Interface(
    can_name="can0",
    host_can_id=0xFD,
    default_kp=80.0,              # 位置 Kp
    default_kd=4.0,               # 位置 Kd
    control_rate_hz=200.0,        # 控制循环频率
    smoothing_alpha=0.8,          # EMA 平滑系数 (0=保持, 1=直通)
    max_velocity=3.0,             # 最大关节速度 (rad/s)
    max_acceleration=15.0,        # 最大关节加速度 (rad/s²)
    velocity_limit=10.0,          # 速度前馈上限 (rad/s)
    gravity_feedforward_ratio=1.0,# 重力补偿比例 (0~1)
    limit_margin=0.15,            # 限位减速区宽度 (rad)
    limit_stop_margin=0.02,       # 限位硬停止区宽度 (rad)
    adaptive_kd_enabled=True,     # 自适应 Kd
    zero_torque_kd_min=0.02,      # 自适应 Kd 下限
    zero_torque_kd_max=1.0,       # 自适应 Kd 上限
    kd_velocity_ref=1.0,          # 自适应 Kd 速度参考 (rad/s)
    urdf_path=None,               # URDF 路径 (Pinocchio)
    inertia_config_path=None,     # 标定惯量参数
)
```

---

## API 参考

### 连接管理

| 方法 | 说明 |
|------|------|
| `ConnectPort()` | 打开 CAN socket，启动收发线程 |
| `DisconnectPort()` | 停止控制循环和线程，关闭 socket |

### 控制循环

| 方法 | 说明 |
|------|------|
| `start_control_loop(rate_hz=200.0)` | 启动后台控制循环 |
| `stop_control_loop()` | 停止控制循环 |

### 电机控制

| 方法 | 说明 |
|------|------|
| `EnableArm(motor_num=7)` | 使能电机 |
| `DisableArm(motor_num=7)` | 失能电机 |
| `EmergencyStop()` | 急停 |
| `SetZeroPosition(motor_num=7)` | 设置零位 |

### 运动控制

| 方法 | 说明 |
|------|------|
| `JointCtrl(j1..j6, kp, kd, torque_ff)` | 关节角度控制 |
| `JointCtrlList(positions)` | 列表形式关节控制 |
| `MoveJ(positions, duration, block=True)` | S-curve 关节运动 |
| `MoveL(target_pose, duration, block=True)` | 笛卡尔直线运动 |
| `EndPoseCtrl(x, y, z, rx, ry, rz, duration, block=True)` | 末端位姿控制 |
| `CartesianVelocityCtrl(vx, vy, vz, wx, wy, wz)` | 笛卡尔速度控制 |
| `GripperCtrl(gripper_angle)` | 夹爪控制 |
| `is_moving()` | 查询轨迹是否在执行 |
| `wait_for_motion(timeout)` | 等待轨迹完成 |
| `cancel_motion()` | 取消当前轨迹 |

### 零力矩模式

| 方法 | 说明 |
|------|------|
| `ZeroTorqueMode(enable, kd=1.0)` | 零力矩模式 (Kp=0) |
| `ZeroTorqueModeWithGravity(enable, kd=0.5)` | 带重力补偿的零力矩（自适应 Kd） |

### 状态反馈

| 方法 | 返回 | 说明 |
|------|------|------|
| `GetArmJointMsgs()` | `ArmJointStates` | 7 关节角度 (rad) |
| `GetArmJointVelocities()` | `ArmJointStates` | 7 关节速度 (rad/s) |
| `GetArmJointEfforts()` | `ArmJointStates` | 7 关节力矩 (Nm) |
| `GetArmEndPoseMsgs()` | `ArmEndPose` | 末端位姿 (Pinocchio FK) |
| `GetArmStatus()` | `ArmStatus` | 综合状态 |

### 动力学 (Pinocchio)

| 方法 | 说明 |
|------|------|
| `ComputeGravityTorques(positions)` | RNEA 重力补偿力矩 |
| `GetJacobian(positions)` | 末端 Jacobian (6xN) |
| `GetMassMatrix(positions)` | 质量矩阵 M(q) |
| `InverseDynamics(q, v, a)` | RNEA 逆动力学 |
| `ForwardDynamics(q, v, tau)` | ABA 正动力学 |

### 参数设置

| 方法 | 说明 |
|------|------|
| `SetPositionPD(kp, kd)` | 设置 PD 增益 |
| `SetSmoothingAlpha(alpha)` | 设置 EMA 平滑系数 |
| `SetGravityFeedforwardRatio(ratio)` | 设置重力补偿比例 |
| `SetJointLimitEnabled(enabled)` | 开关关节限位保护 |

---

## 电机通信协议

运控模式（MIT-like PD Control）：

```
τ = Kp × (θ_target - θ_actual) + Kd × (ω_target - ω_actual) + τ_ff
```

29 位扩展帧 ID：

```
Bit 28~24: 通信类型 (5 bits)
Bit 23~8:  数据区2 (16 bits)
Bit 7~0:   目标地址 (8 bits)
```

| 通信类型 | 值 | 功能 |
|----------|---|------|
| MOTION_CONTROL | 1 | 运控指令 (位置/速度/Kp/Kd/力矩) |
| FEEDBACK | 2 | 电机反馈 (位置/速度/力矩/温度) |
| ENABLE | 3 | 使能电机 |
| DISABLE | 4 | 停止电机 |
| SET_ZERO | 6 | 设置零位 |
| WRITE_PARAM | 18 | 参数写入 |

uint16 线性映射：

```
编码: uint16 = (value - min) × 65535 / (max - min)
解码: value  = uint16 × (max - min) / 65535 + min
```

---

## 故障排除

### CAN 接口

```bash
lsusb | grep -i can
sudo modprobe can && sudo modprobe can_raw && sudo modprobe gs_usb
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
candump can0
```

### 电机无响应

1. 检查 CAN 接线和终端电阻
2. 确认电机 ID 配置 (1-7)
3. 检查电源供电
4. 验证主机 CAN ID (默认 253/0xFD)

### 末端抖动

1. 调整 `default_kp` / `default_kd`
2. 增加 `smoothing_alpha`
3. 降低 `max_velocity`

---

## 目录结构

```
el_a3_sdk/
├── __init__.py          # 包入口
├── interface.py         # ELA3Interface — 主接口 + 控制循环
├── arm_manager.py       # ArmManager — 多臂管理器
├── drivers/             # SocketCAN / SLCAN 驱动包
├── protocol.py          # 协议枚举、电机参数、关节配置
├── data_types.py        # 数据结构 (SI 单位)
├── kinematics.py        # Pinocchio FK/IK/Jacobian/Gravity
├── realsense/           # RealSense RGB-D 采集、点云和选点工具
├── motion/              # S-curve、三次样条和笛卡尔轨迹辅助
├── utils.py             # 工具函数
├── pyproject.toml       # pip 安装配置
└── demo/
    ├── control_loop_demo.py   # 控制循环示例
    ├── motion_control.py      # 关节运动示例
    ├── zero_torque_mode.py    # 零力矩示教示例
    ├── cartesian_control_demo.py  # 笛卡尔控制示例
    ├── dynamics_demo.py       # 动力学示例
    ├── trajectory_demo.py     # 轨迹规划示例
    ├── waypoint_loop_real.py  # 路径点循环示例
    └── read_joints.py         # 关节读取示例
```

---

## 许可证

Apache-2.0
