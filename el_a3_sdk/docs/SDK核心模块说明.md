# EL-A3 SDK 核心模块说明

本文档面向需要阅读、维护或二次开发 `el_a3_sdk` 的开发者，重点说明 SDK 核心模块的职责边界、数据流、运动控制链路和常用扩展点。SDK 本体不依赖 ROS，主要通过 SocketCAN / SLCAN 与 Robstride 电机通信，并通过 Pinocchio 提供运动学和动力学能力。

当前版本已经将 CAN 后端和轨迹规划移入独立子包：

```text
el_a3_sdk/drivers/
el_a3_sdk/motion/
```

外部脚本和 GUI 仍然应该优先通过 `ELA3Interface` 调用机械臂能力，不建议直接绕过主接口访问底层 driver。

## 1. 总体架构

`el_a3_sdk` 可以分为六层。当前结构不是“文件越多越好”，而是保持一个清晰的对外入口，并把硬件通信、运动规划、视觉工具等职责收进各自子包。

| 层级 | 主要模块 | 职责 |
| --- | --- | --- |
| 对外入口层 | `__init__.py` | 统一导出 SDK 类型、接口类和 RealSense 工具；对重依赖模块做延迟导入 |
| 主控制接口层 | `interface.py` | 提供机械臂连接、使能、运动控制、反馈读取、控制循环、安全控制、动力学接口 |
| 硬件通信层 | `drivers/`, `protocol.py` | 封装 Robstride CAN 协议、SocketCAN / SLCAN 收发、反馈解析、参数读写 |
| 运动规划层 | `motion/` | 生成 S-curve 关节轨迹、多关节同步轨迹和笛卡尔插值辅助 |
| 运动学动力学层 | `kinematics.py` | 基于 Pinocchio 提供 FK、IK、Jacobian、重力补偿、质量矩阵等 |
| 数据与辅助层 | `data_types.py`, `utils.py`, `arm_manager.py`, `controller_profiles.py`, `realsense/` | 数据结构、单位转换、多臂管理、手柄配置、相机点云与视觉目标工具 |

核心控制链路可以概括为：

```text
上层应用 / MotorStudio / 脚本
        |
        v
ELA3Interface
        |
        +-- MoveJ / MoveL / JointCtrl / GripperCtrl
        |
        +-- ELA3Kinematics / TrajectoryPlanner
        |
        v
drivers.create_can_driver()
        |
        v
RobstrideCanDriver 或 SlcanCanDriver
        |
        v
Robstride 电机 CAN 总线
```

SDK 默认使用 SI 单位：

| 类型 | 单位 |
| --- | --- |
| 关节角 | rad |
| 关节速度 | rad/s |
| 位置 | m |
| 姿态 | rad |
| 力矩 | Nm |
| 时间 | s |

### 1.1 当前结构与后续重构边界

当前核心入口仍然是 `interface.py` 中的 `ELA3Interface`。它是 SDK 的公开门面，负责把上层调用转成控制循环、轨迹规划和 CAN 指令。

如果后续继续整理核心代码，推荐使用一个适中的 `runtime/` 分组，而不是拆出过多零碎模块：

| 建议模块 | 作用 | 可承接现有代码 |
| --- | --- | --- |
| `runtime/context.py` | 装配 driver、kinematics、关节映射、TCP 偏移和默认控制参数 | `ELA3Interface.__init__()` 中的依赖创建、`_get_kinematics()`、`_normalize_tcp_offset()` |
| `runtime/state.py` | 保存连接状态、机械臂状态、目标关节、轨迹队列、运动完成事件等运行时状态 | `_connected`, `_state`, `_target_positions`, `_target_velocities`, `_trajectory`, `_motion_done` 等成员 |
| `runtime/control.py` | 实现控制循环、JointCtrl、MoveJ、MoveL、EndPoseCtrl、CartesianVelocityCtrl 等运动行为 | `start_control_loop()`, `_control_loop_tick()`, `JointCtrl()`, `MoveJ()`, `MoveL()` |
| `runtime/safety.py` | 实现状态门禁、限位保护、急停、复位、总线健康检查等安全逻辑 | `EnableArm()`, `DisableArm()`, `EmergencyStop()`, `ResetArm()`，以及控制循环中的限位裁剪 |

这种方式能让 `ELA3Interface` 逐步变薄，同时不会把项目拆得太散。GUI、脚本和外部用户仍只依赖 `ELA3Interface`。

## 2. 对外入口：`el_a3_sdk/__init__.py`

该文件负责把 SDK 常用类型集中导出，用户可以直接从 `el_a3_sdk` 导入常用对象。

典型用法：

```python
from el_a3_sdk import ELA3Interface, ArmEndPose, ArmJointStates
```

主要特点：

| 内容 | 说明 |
| --- | --- |
| 数据类型导出 | `MotorFeedback`, `ArmStatus`, `ArmJointStates`, `ArmEndPose`, `DynamicsInfo` 等 |
| 协议枚举导出 | `MotorType`, `RunMode`, `ControlMode`, `MoveMode`, `ArmState`, `LogLevel` |
| RealSense 工具导出 | `RealSenseD435`, `PointCloud`, `pick_point`, `BookSpineMatcher` 等 |
| 延迟导入 | `ELA3Interface`, `ArmManager`, `get_kinematics()` 采用延迟导入，避免缺少 Pinocchio 时影响基础导入 |

`__init__.py` 是 SDK 的公共 API 门面。新增对外可见的数据类型或工具函数时，通常需要同步加入 `__all__`。

## 3. 主接口模块：`el_a3_sdk/interface.py`

`ELA3Interface` 是整个 SDK 的核心类，负责把上层指令转换为电机控制命令。它同时管理连接状态、控制循环、运动规划、运动学调用和硬件通信。

### 3.1 主要职责

| 功能类别 | 代表接口 | 说明 |
| --- | --- | --- |
| 连接管理 | `ConnectPort()`, `DisconnectPort()`, `get_connect_status()` | 打开 / 关闭底层 CAN 驱动，并启动接收线程 |
| 使能与安全 | `EnableArm()`, `DisableArm()`, `EmergencyStop()`, `ResetArm()` | 控制电机状态和 SDK 状态机 |
| 关节控制 | `JointCtrl()`, `JointCtrlList()` | 直接发送目标关节角，或更新控制循环目标 |
| 夹爪控制 | `GripperCtrl()` | 控制第 7 轴夹爪 |
| 轨迹运动 | `MoveJ()`, `MoveL()`, `MoveWaypoints()` | 关节空间 / 笛卡尔空间轨迹执行 |
| 笛卡尔控制 | `EndPoseCtrl()`, `CartesianVelocityCtrl()` | 通过 IK 将末端位姿或速度转为关节目标 |
| 零力矩 | `ZeroTorqueMode()`, `ZeroTorqueModeWithGravity()` | 进入可拖动状态，可叠加重力补偿 |
| 状态反馈 | `GetArmJointMsgs()`, `GetArmStatus()`, `GetMotorStates()` | 读取关节、电机和系统状态 |
| 动力学 | `ComputeGravityTorques()`, `GetJacobian()`, `GetMassMatrix()` | 通过 Pinocchio 计算动力学信息 |
| 参数配置 | `SetPositionPD()`, `SetJointLimitEnabled()`, `SetTcpOffset()` | 设置控制参数、关节限位、TCP 偏移 |

### 3.2 状态机

`ELA3Interface` 使用 `ArmState` 描述当前机械臂状态：

| 状态 | 含义 |
| --- | --- |
| `DISCONNECTED` | 未连接 CAN |
| `IDLE` | 已连接，但电机未使能 |
| `ENABLED` | 电机已使能，可以接收控制命令 |
| `RUNNING` | 正在执行运动控制 |
| `ZERO_TORQUE` | 零力矩 / 重力补偿拖动模式 |
| `ERROR` | 错误状态 |

一般控制流程：

```python
arm = ELA3Interface(can_name="can0")
arm.ConnectPort()
arm.EnableArm()
arm.start_control_loop(rate_hz=200.0)
arm.MoveJ([0.0, 1.0, -1.2, 0.0, 0.0, 0.0], duration=2.0)
arm.DisableArm()
arm.DisconnectPort()
```

### 3.3 控制循环

`start_control_loop()` 会启动后台控制线程，默认 200 Hz。控制循环主要做四件事：

| 步骤 | 说明 |
| --- | --- |
| 读取目标 | 从 `_target_positions`, `_target_velocities` 或轨迹队列取当前目标 |
| 安全处理 | 进行关节限位检查、限位附近减速、硬限位保护 |
| 发送控制帧 | 调用底层 driver 的 `send_motion_control()` 发送运控模式 Type 1 帧 |

控制循环运行时，`JointCtrl()` 不会逐帧直接发送 CAN 指令，而是更新 `_target_positions` 和 `_target_velocities`，由后台循环统一发送。`MoveJ()` / `MoveL()` 生成的轨迹也会在控制循环中按时间采样并执行。这样可以降低上层调用抖动对电机控制的影响。

控制循环未运行时，`JointCtrl()` 会在调用线程中直接给每个电机发送运动控制帧。

## 4. 运动控制数据流

### 4.1 `JointCtrl`

`JointCtrl` 是最底层的关节目标控制接口。

输入：

```text
joint_1 ... joint_6: 目标关节角，单位 rad
kp, kd: 可选 PD 参数
velocity / velocities: 速度前馈
torque_ff: 力矩前馈
```

执行逻辑：

```text
JointCtrl
  -> 检查连接状态和机械臂状态
  -> 组成 6 关节目标 positions
  -> 如果控制循环运行：
       更新 _target_positions / _target_velocities
       返回
     否则：
       对每个关节做限位、方向、offset 映射
       调用 driver.send_motion_control()
```

其中关节方向和偏移来自 `protocol.py` 中的：

```text
DEFAULT_JOINT_DIRECTIONS
DEFAULT_JOINT_OFFSETS
DEFAULT_JOINT_LIMITS
```

### 4.2 `MoveJ`

`MoveJ` 是关节空间运动。它不是简单地把目标关节角一次性发给电机，而是先规划一条多关节同步 S-curve 轨迹。

数据流：

```text
MoveJ(target_q, duration)
  -> 读取当前关节角 start_q
  -> MultiJointPlanner.plan_sync(start_q, target_q)
  -> 生成 TrajectoryPoint 列表
  -> 如果控制循环运行：
       _execute_trajectory_async(traj)
     否则：
       for pt in traj:
         compute_gravity(pt.positions)
         JointCtrl(pt.positions, velocities=pt.velocities, torque_ff=gravity)
```

也就是说，`MoveJ` 最终仍然会通过 `JointCtrl` 或控制循环把关节目标发送到电机。

### 4.3 `MoveL`

`MoveL` 是笛卡尔直线运动。它在末端空间插值，再对每个插值点做 IK。

数据流：

```text
MoveL(target_pose, duration)
  -> 读取当前关节角 current_q
  -> forward_kinematics(current_q) 得到 start_pose
  -> 在 start_pose 和 target_pose 之间做笛卡尔插值
  -> 对每个插值点：
       ik_step()
       如果失败则 inverse_kinematics() fallback
       得到 q_sol
  -> 形成关节轨迹 traj_points
  -> 补齐速度 / 加速度
  -> 如果控制循环运行：
       _execute_trajectory_async(traj_points)
     否则：
       for pt in traj_points:
         compute_gravity(pt.positions)
         JointCtrl(pt.positions, velocities=pt.velocities, torque_ff=gravity)
```

`MoveL` 的目标类型是 `ArmEndPose`：

```python
ArmEndPose(x=0.3, y=0.0, z=0.2, rx=0.0, ry=0.0, rz=0.0)
```

位置单位是 m，姿态单位是 rad。

需要注意的是，`MoveL` 的目标默认是 TCP 位姿。如果设置了 TCP 偏移，IK 会先把 TCP 目标换回 URDF 末端 frame，再求解关节角，因此 GUI 中看到的末端目标坐标应与当前 TCP 配置一致。

### 4.4 `EndPoseCtrl`

`EndPoseCtrl` 是单目标笛卡尔位姿控制，流程更接近：

```text
EndPoseCtrl(x, y, z, rx, ry, rz)
  -> inverse_kinematics()
  -> MoveJ(q_target)
```

它不是严格的直线运动；如果需要末端走直线，应使用 `MoveL`。

## 5. CAN 通信模块

### 5.1 协议定义：`protocol.py`

`protocol.py` 定义 Robstride 协议相关常量和枚举。

| 类型 | 说明 |
| --- | --- |
| `CommType` | CAN 扩展帧通信类型，如运控、反馈、使能、参数读写 |
| `MotorType` | 电机型号，如 `RS00`, `EL05`, `RS05` |
| `RunMode` | 电机运行模式，如运控、PP 位置、速度、电流、CSP |
| `ArmState` | SDK 内部机械臂状态 |
| `ControlMode` | 对外控制模式 |
| `MoveMode` | 对外运动模式 |
| `ParamIndex` | Robstride 参数索引 |
| `MotorParams` | 不同电机型号的位置、速度、力矩、PD 范围 |

CAN 29 位扩展 ID 结构：

```text
Bit28~24: 通信类型
Bit23~8 : 数据区 2
Bit7~0  : 目标电机 ID
```

默认机械臂配置：

| 配置项 | 说明 |
| --- | --- |
| `DEFAULT_MOTOR_TYPE_MAP` | 电机 ID 到电机型号映射 |
| `DEFAULT_JOINT_DIRECTIONS` | SDK 关节方向到电机方向的映射 |
| `DEFAULT_JOINT_OFFSETS` | 关节零点偏移 |
| `DEFAULT_JOINT_LIMITS` | 关节软件限位 |

### 5.2 驱动包：`drivers/`

新的驱动实现集中在 `el_a3_sdk/drivers/`：

| 文件 | 说明 |
| --- | --- |
| `drivers/base.py` | 定义 `CanDriverProtocol` 和 `create_can_driver()` 工厂函数 |
| `drivers/socketcan.py` | SocketCAN 实现，类名 `RobstrideCanDriver` |
| `drivers/slcan.py` | SLCAN 实现，类名 `SlcanCanDriver` |
| `drivers/timing.py` | 公共微秒级忙等待工具 |
| `drivers/__init__.py` | 驱动包统一入口 |

`ELA3Interface` 不再直接判断并实例化不同驱动，而是调用：

```python
from el_a3_sdk.drivers import create_can_driver

driver = create_can_driver(backend="socketcan", can_name="can0")
```

需要直接使用具体驱动类时，从对应后端模块导入：

```python
from el_a3_sdk.drivers.socketcan import RobstrideCanDriver
from el_a3_sdk.drivers.slcan import SlcanCanDriver
```

### 5.3 SocketCAN 驱动：`drivers/socketcan.py`

`RobstrideCanDriver` 负责 Linux SocketCAN 通信。

它直接使用 Linux 原生 socket：

```python
socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
```

这表示：

- `AF_CAN` 选择 CAN 协议族
- `SOCK_RAW` 使用原始帧收发
- `CAN_RAW` 通过 SocketCAN 的原始协议层直接操作 CAN 帧

实际帧使用 29 位扩展 ID 和 8 字节数据区，内部格式为：

```python
CAN_FRAME_FMT = "=IB3x8s"
```

对应 `can_id + dlc + padding + data[8]`，总计 16 字节。

主要能力：

| 接口 | 说明 |
| --- | --- |
| `connect()` / `disconnect()` | 绑定 / 关闭 CAN socket |
| `start_receive_thread()` | 启动后台接收线程 |
| `_parse_frame()` | 解析 Robstride 扩展帧 |
| `enable_motor()` / `disable_motor()` | 电机使能 / 失能 |
| `send_motion_control()` | 发送运控模式 Type 1 帧 |
| `write_parameter()` / `read_parameter()` | 参数读写 |
| `set_position_pp()` | PP 位置模式控制 |
| `get_feedback()` / `get_all_feedbacks()` | 获取缓存的电机反馈 |
| `check_bus_health()` | CAN 发送健康状态检查 |

反馈解析结果会保存为 `MotorFeedback`，上层通过 `ELA3Interface.GetArmJointMsgs()` 等接口读取。

### 5.4 SLCAN 驱动：`drivers/slcan.py`

`SlcanCanDriver` 与 `RobstrideCanDriver` 暴露近似相同的方法，但底层通过串口 SLCAN 适配器通信。

它更适合没有原生 SocketCAN 设备、但有 USB 转 CAN 适配器且适配器固件支持 SLCAN ASCII 协议的场景。

典型初始化：

```python
arm = ELA3Interface(
    backend="slcan",
    serial_port="/dev/ttyUSB0",
    serial_baudrate=2000000,
    can_bitrate=1000000,
)
```

由于 `ELA3Interface` 只依赖 driver 的统一方法名和 `create_can_driver()` 工厂函数，上层运动控制逻辑不需要区分 SocketCAN 或 SLCAN。

## 6. 数据结构模块：`data_types.py`

`data_types.py` 使用 dataclass 定义 SDK 内部和对外返回的数据结构。

| 类型 | 用途 |
| --- | --- |
| `MotorFeedback` | 单个电机的 Type 2 反馈数据 |
| `ArmJointStates` | 机械臂 6 关节 + 夹爪的角度、速度或力矩状态 |
| `ArmEndPose` | 末端位姿，包含 x/y/z/rx/ry/rz |
| `ArmStatus` | 机械臂总体状态、使能状态、故障码 |
| `MotorHighSpdInfo` | 高速反馈信息 |
| `MotorLowSpdInfo` | 参数读取得到的低速状态信息 |
| `MotorAngleLimitMaxVel` | 电机角度限制和最大速度 |
| `MotorMaxAccLimit` | 电机最大加速度 |
| `ParamReadResult` | 参数读取结果 |
| `FirmwareVersion` | 固件版本 |
| `DynamicsInfo` | 重力、Jacobian、质量矩阵、科氏力等动力学信息 |
| `TrajectoryResult` | 轨迹执行结果 |

最常用的是：

```python
joints = arm.GetArmJointMsgs()
print(joints.to_list())

pose = arm.GetArmEndPoseMsgs()
print(pose.x, pose.y, pose.z)
```

`data_types.py` 的作用不是单纯“存类型名”，而是统一 SDK 内部的数据出口。CAN 驱动先把原始反馈解析成 `MotorFeedback`，再由 `interface.py` 汇总为 `ArmJointStates`、`ArmStatus`、`ArmEndPose` 等更适合上层使用的对象。

## 7. 运动学与动力学模块：`kinematics.py`

`ELA3Kinematics` 基于 Pinocchio 和 URDF 构建机械臂模型。默认 URDF 路径为：

```text
resources/urdf/el_a3.urdf
```

### 7.1 核心能力

| 接口 | 说明 |
| --- | --- |
| `forward_kinematics(q)` | 关节角转末端位姿 |
| `forward_kinematics_se3(q)` | 关节角转 Pinocchio `SE3` |
| `inverse_kinematics(target_pose)` | 数值 IK，末端位姿转关节角 |
| `ik_step(target_pose, q_current)` | 实时 IK 单步迭代，用于 MoveL 轨迹点 |
| `compute_jacobian(q)` | 计算 6xN Jacobian |
| `compute_gravity(q)` | 计算 RNEA 重力补偿力矩 |
| `inverse_dynamics(q, v, a)` | 逆动力学 |
| `forward_dynamics(q, v, tau)` | 正动力学 |
| `mass_matrix(q)` | CRBA 质量矩阵 |
| `coriolis_matrix(q, v)` | 科氏力矩阵 |
| `set_tcp_offset()` | 设置 TCP 偏移 |

### 7.2 TCP 偏移

SDK 支持在 URDF `end_effector` 基础上叠加 TCP 偏移：

```python
arm.SetTcpOffset([x, y, z, rx, ry, rz])
```

其中：

| 字段 | 含义 |
| --- | --- |
| `x, y, z` | TCP 相对 URDF 末端 frame 的平移，单位 m |
| `rx, ry, rz` | TCP 相对 URDF 末端 frame 的 RPY 姿态，单位 rad |

FK 会返回 TCP 位姿：

```text
oMtcp = oMf * tcp_offset
```

IK 会先把目标 TCP 位姿换算回 URDF 末端 frame：

```text
target_frame = target_tcp * tcp_offset.inverse()
```

因此 `MoveL` 和 `EndPoseCtrl` 的目标点默认都是 TCP 目标，而不是裸 URDF `end_effector` frame 目标。

## 8. 运动规划模块：`motion/`

`motion/` 负责生成轨迹点和提供轨迹执行辅助，不直接与硬件通信。

| 文件 | 说明 |
| --- | --- |
| `motion/trajectory.py` | S-curve、多关节同步、三次样条规划 |
| `motion/cartesian.py` | 五次时间缩放、末端位姿插值、轨迹采样、速度/加速度补齐 |
| `motion/__init__.py` | 运动规划统一入口 |

典型导入方式：

```python
from el_a3_sdk.motion import MultiJointPlanner
```

### 8.1 主要类型

| 类型 | 说明 |
| --- | --- |
| `TrajectoryPoint` | 单个轨迹采样点，包含 time、positions、velocities、accelerations |
| `SCurveProfile` | 单关节 7 段 S-curve 参数 |
| `SCurvePlanner` | 单关节 S-curve 规划器 |
| `MultiJointPlanner` | 多关节同步规划器 |
| `CubicSplinePlanner` | 多路径点三次样条规划器 |
| `sample_trajectory()` | 控制循环按当前时间采样轨迹点 |
| `fill_trajectory_derivatives()` | 用中心差分补齐速度和加速度前馈 |
| `smooth_time_scale()` | 五次时间缩放 |
| `interpolate_pose()` | 位置线性插值 + 姿态球面插值 |

### 8.2 多关节同步

`MoveJ` 使用 `MultiJointPlanner`：

```text
每个关节单独规划 S-curve
  -> 找到最长总时长
  -> 其他关节降低速度 / 加速度以同步结束
  -> 生成统一时间轴上的 TrajectoryPoint
```

轨迹点最终由 `ELA3Interface` 执行。当前 `MoveJ()` 和 `MoveL()` 都会在需要时补齐速度和加速度前馈，再交给控制循环或同步发送逻辑。

## 9. 多臂管理：`arm_manager.py`

`ArmManager` 是一个 Singleton，用于管理多个机械臂实例。

主要接口：

| 接口 | 说明 |
| --- | --- |
| `register_can_arm(name, can_name, **kwargs)` | 注册一个 CAN 直连机械臂 |
| `get_arm(name)` | 按名称获取机械臂 |
| `disconnect_all()` | 断开所有机械臂 |
| `from_config(config_path, auto_connect=False)` | 从 YAML 配置批量创建 |

示例：

```python
from el_a3_sdk import ArmManager

mgr = ArmManager.get_instance()
left = mgr.register_can_arm("left", can_name="can0")
right = mgr.register_can_arm("right", can_name="can1")
```

适合双臂或主从臂场景。

## 10. RealSense 子模块：`el_a3_sdk/realsense`

该子包提供 D435 相机采集、点云生成、选点、图像匹配和坐标变换工具。它不是机械臂控制的必需依赖，但在点云抓取、书脊识别等视觉任务中使用。

| 模块 | 说明 |
| --- | --- |
| `camera.py` | D435 RGBD 采集、深度反投影、点云生成、图像保存 |
| `picker.py` | Open3D 点云显示、点选、选点结果保存 |
| `geometry.py` | 刚体变换 `RigidTransform`，支持 JSON 读取和点变换 |
| `book_spine_matcher.py` | 书脊模板匹配、跟踪、稳定性判断、可视化叠加 |

点云选点常见数据流：

```text
RealSenseD435.get_rgbd_frame()
  -> RGBDFrame.to_point_cloud()
  -> picker.pick_point()
  -> 得到相机坐标系下的点 p_C
  -> RigidTransform 或外部 T_B_C 变换
  -> 得到机械臂基坐标系下的目标点 p_B
```

如果在 GUI 中使用点云选点，通常由 MotorStudio 读取 `resources/config/camera_to_robot_transform.json`，把相机坐标点转换到机械臂基坐标系。

## 11. 工具模块：`utils.py`

`utils.py` 放通用数学工具。

| 函数 | 说明 |
| --- | --- |
| `float_to_uint16()` / `uint16_to_float()` | Robstride 协议浮点和 16 位整数映射 |
| `rad_to_deg()` / `deg_to_rad()` | 角度单位转换 |
| `clamp()` | 数值限幅 |
| `euler_to_quat()` / `quat_to_euler()` | 欧拉角和四元数转换 |
| `slerp_euler()` | 姿态球面插值，用于 `MoveL` 姿态插值 |

`MoveL` 的姿态插值会使用 `slerp_euler()`，避免直接线性插值欧拉角造成较明显的姿态跳变。

## 12. 手柄配置：`controller_profiles.py`

该模块用于识别和映射手柄输入，主要服务遥操作脚本。

主要内容：

| 类型 | 说明 |
| --- | --- |
| `AxisBinding` | 单个轴绑定 |
| `TriggerBinding` | 扳机轴绑定 |
| `StickMap` | 摇杆映射 |
| `ButtonMap` | 按键映射 |
| `ControllerProfile` | 手柄配置 |
| `ControllerDetection` | 手柄检测结果 |

常用函数：

| 函数 | 说明 |
| --- | --- |
| `list_profiles()` | 列出内置手柄配置 |
| `get_profile(profile_id)` | 获取指定配置 |
| `detect_controller(device, requested_profile="auto")` | 根据设备信息自动识别 |

## 13. 常见控制链路示例

### 13.1 直接关节控制

```python
from el_a3_sdk import ELA3Interface

arm = ELA3Interface(can_name="can0")
arm.ConnectPort()
arm.EnableArm()
arm.start_control_loop()

arm.JointCtrl(0.0, 1.0, -1.2, 0.0, 0.0, 0.0)
```

链路：

```text
JointCtrl
  -> 更新控制循环目标
  -> 控制循环发送 Type 1 运控帧
```

### 13.2 MoveJ 关节轨迹

```python
arm.MoveJ([0.0, 1.0, -1.2, 0.0, 0.0, 0.0], duration=2.0)
```

链路：

```text
MoveJ
  -> MultiJointPlanner
  -> TrajectoryPoint[]
  -> 控制循环轨迹队列或同步 JointCtrl
  -> CAN 运控帧
```

### 13.3 MoveL 笛卡尔直线

```python
from el_a3_sdk import ArmEndPose

target = ArmEndPose(x=0.25, y=0.0, z=0.20, rx=0.0, ry=0.0, rz=0.0)
arm.MoveL(target, duration=2.0)
```

链路：

```text
MoveL
  -> FK 得到当前末端位姿
  -> 笛卡尔空间插值
  -> IK 转每个插值点的关节角
  -> 轨迹执行
  -> JointCtrl / 控制循环
```

### 13.4 读取反馈

```python
joints = arm.GetArmJointMsgs()
pose = arm.GetArmEndPoseMsgs()
status = arm.GetArmStatus()

print(joints.to_list())
print(pose.x, pose.y, pose.z)
print(status.all_enabled, status.has_fault)
```

反馈链路：

```text
CAN 接收线程
  -> 解析 Type 2 feedback
  -> 缓存 MotorFeedback
  -> ELA3Interface 汇总为 ArmJointStates / ArmStatus
```

## 14. 修改或扩展时的建议

| 修改目标 | 推荐位置 |
| --- | --- |
| 新增对外 API | `interface.py`，必要时同步 `__init__.py` |
| 修改 CAN 协议解析 | `drivers/socketcan.py` 和 `drivers/slcan.py`，保持两者接口一致 |
| 修改电机型号或限位 | `protocol.py` |
| 修改轨迹规划 | `motion/trajectory.py` |
| 修改轨迹采样或笛卡尔插值 | `motion/cartesian.py` |
| 修改 FK / IK / TCP | `kinematics.py` |
| 新增视觉点云工具 | `el_a3_sdk/realsense/` |
| 新增数据返回结构 | `data_types.py` |

扩展时建议遵守以下原则：

| 原则 | 说明 |
| --- | --- |
| 保持单位一致 | SDK 内部统一使用 SI 单位 |
| 保持 driver 接口一致 | SocketCAN 和 SLCAN 应提供同名方法，避免上层分支 |
| 不绕过状态机 | 运动命令前应确保连接和使能状态正确 |
| 轨迹优先走规划器 | 避免上层直接以低频循环频繁调用 `JointCtrl` |
| 修改 TCP 后验证 IK | TCP 会影响 FK、IK、MoveL 和 GUI 末端显示 |
| 硬件调试先低速 | 新增控制逻辑应先降低速度、加速度和 PD 参数验证 |

## 15. 快速定位表

| 想看什么 | 文件 |
| --- | --- |
| SDK 主入口 | `el_a3_sdk/__init__.py` |
| 连接、使能、MoveJ、MoveL、JointCtrl | `el_a3_sdk/interface.py` |
| CAN 驱动工厂和抽象 | `el_a3_sdk/drivers/base.py` |
| SocketCAN 帧收发和反馈解析 | `el_a3_sdk/drivers/socketcan.py` |
| 串口 CAN 适配器 | `el_a3_sdk/drivers/slcan.py` |
| 忙等待工具 | `el_a3_sdk/drivers/timing.py` |
| 协议枚举、关节方向、限位 | `el_a3_sdk/protocol.py` |
| 数据结构 | `el_a3_sdk/data_types.py` |
| FK、IK、重力补偿、TCP | `el_a3_sdk/kinematics.py` |
| S-curve 和轨迹点 | `el_a3_sdk/motion/trajectory.py` |
| 轨迹采样和笛卡尔插值辅助 | `el_a3_sdk/motion/cartesian.py` |
| 多臂管理 | `el_a3_sdk/arm_manager.py` |
| 单位转换和姿态插值 | `el_a3_sdk/utils.py` |
| RealSense 点云与视觉 | `el_a3_sdk/realsense/` |
