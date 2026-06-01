# EL-A3 ROS2 Control System

> 7-DOF 桌面机械臂 ROS2 控制系统，基于 ros2_control、CAN 总线 Robstride 电机、Pinocchio RNEA 重力补偿、MoveIt2 运动规划。

---

## 快速开始

```bash
# 安装依赖
cd scripts && sudo ./install_deps.sh

# 编译
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 配置 CAN 接口
sudo bash scripts/setup_can.sh can0 1000000

# 启动真实硬件（基础控制）
ros2 launch el_a3_description el_a3_control.launch.py can_interface:=can0

# 启动真实硬件 + MoveIt
ros2 launch el_a3_moveit_config robot.launch.py can_interface:=can0

# 仿真模式（无硬件）
ros2 launch el_a3_moveit_config demo.launch.py
```

---

## 功能包

| 功能包 | 类型 | 说明 |
|--------|------|------|
| `el_a3_hardware` | ament_cmake | ros2_control 硬件接口 + CAN 驱动 + ZeroTorqueController 插件 |
| `el_a3_description` | ament_cmake | URDF/xacro (7 关节)、ros2_control 配置、Launch 文件 |
| `el_a3_moveit_config` | ament_cmake | MoveIt2 运动规划配置（SRDF、运动学、OMPL、Servo） |
| `el_a3_teleop` | ament_python | Xbox/手柄遥操作（Jacobian IK 笛卡尔控制） |

---

## 目录结构

```
el_a3_ros/
├── el_a3_description/          # URDF、Meshes、控制器 YAML、Launch
│   ├── urdf/
│   │   ├── el_a3.urdf.xacro        # 主 xacro（含 ros2_control）
│   │   └── el_a3_ros2_control.xacro # 硬件接口参数
│   ├── config/
│   │   ├── el_a3_controllers.yaml   # ros2_control 控制器配置
│   │   ├── multi_arm_controllers.yaml
│   │   └── inertia_params.yaml      # Pinocchio 标定惯量参数
│   └── launch/
│       ├── el_a3_control.launch.py  # 基础控制 launch
│       └── multi_arm_control.launch.py
├── el_a3_hardware/             # C++ ros2_control 硬件接口
│   ├── src/
│   │   ├── el_a3_hardware.cpp       # RsA3HardwareInterface 插件
│   │   ├── robstride_can_driver.cpp # CAN 帧收发驱动
│   │   └── zero_torque_controller.cpp # 零力矩（拖动示教）控制器
│   └── include/el_a3_hardware/
├── el_a3_moveit_config/        # MoveIt2 配置
│   ├── config/
│   │   ├── el_a3.srdf               # 规划组、碰撞矩阵
│   │   ├── kinematics.yaml          # IK 求解器（pick_ik）
│   │   ├── joint_limits.yaml        # 关节限位
│   │   ├── ompl_planning.yaml       # OMPL 规划器
│   │   ├── moveit_controllers.yaml  # MoveIt 控制器桥接
│   │   └── servo_config.yaml        # MoveIt Servo（预留）
│   └── launch/
│       ├── demo.launch.py           # 仿真 MoveIt
│       └── robot.launch.py          # 真实硬件 MoveIt
├── el_a3_teleop/               # 手柄遥操作
│   ├── config/xbox_teleop.yaml
│   └── launch/real_xbox_teleop.launch.py
├── scripts/                    # 安装脚本、CAN 配置、标定
├── fastrtps_no_shm.xml         # DDS 网络配置
└── docker-compose.yml
```

---

## Launch 文件说明

### 基础控制（不含 MoveIt）

```bash
ros2 launch el_a3_description el_a3_control.launch.py \
  can_interface:=can0 \
  wrist_motor_type:=EL05 \
  use_rviz:=true
```

启动 `ros2_control_node` + `robot_state_publisher` + `joint_state_broadcaster` + `arm_controller` + `gripper_controller`。

### MoveIt 运动规划 — 仿真

```bash
ros2 launch el_a3_moveit_config demo.launch.py
```

使用 mock hardware，在 RViz 中拖动交互标记进行运动规划测试。

### MoveIt 运动规划 — 真实硬件

```bash
ros2 launch el_a3_moveit_config robot.launch.py \
  can_interface:=can0 \
  wrist_motor_type:=EL05
```

额外启动 `move_group` 节点和 `zero_torque_controller`（inactive 状态）。

### Xbox 手柄遥操作

```bash
ros2 launch el_a3_teleop real_xbox_teleop.launch.py \
  can_interface:=can0 \
  auto_detect_controller:=true
```

自动检测手柄类型，启动 `joy_node` + `xbox_teleop_node`。详见 [XBOX_CONTROL_SETUP.md](XBOX_CONTROL_SETUP.md)。

### 多臂控制

```bash
ros2 launch el_a3_description multi_arm_control.launch.py \
  config_file:=/path/to/multi_arm_config.yaml
```

配置文件格式参考 `el_a3_description/config/multi_arm_config.yaml`。

---

## 启动测试

`scripts/tests/startup_test_demo.py` 提供端到端启动验证，自动完成系统启动、控制器检查、关节运动、夹爪控制等全流程测试。

### 运行方式

```bash
# Mock 仿真模式（默认，无需硬件）
python3 scripts/tests/startup_test_demo.py

# 真实硬件模式
python3 scripts/tests/startup_test_demo.py --mode real --can-interface can0

# 连接已运行的 ros2_control 系统（不自动启动 launch）
python3 scripts/tests/startup_test_demo.py --mode connect

# 包含零力矩控制器切换测试
python3 scripts/tests/startup_test_demo.py --test-zero-torque
```

### 测试阶段

| 阶段 | 内容 |
|------|------|
| Phase 1 | 系统健康检查：controller_manager / robot_state_publisher 节点在线、robot_description 可读、TF 存在 |
| Phase 2 | 控制器状态验证：joint_state_broadcaster / arm_controller / gripper_controller 均 active、硬件接口 claimed、Action Server 可用 |
| Phase 3 | 关节状态读取：订阅 `/joint_states` 确认 7 个关节数据完整 |
| Phase 4 | 基础运动测试：归零位、单关节运动、多关节联合运动（mock 模式下额外验证位置跟踪误差） |
| Phase 5 | 夹爪测试：L7 开合控制 |
| Phase 6 | 控制器切换（可选）：arm_controller 与 zero_torque_controller 互切验证 |

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--mode` | `mock` | 运行模式：`mock`（仿真）/ `real`（真实硬件）/ `connect`（连接已运行系统） |
| `--can-interface` | `can0` | CAN 接口名（real 模式） |
| `--wrist-motor-type` | `EL05` | 腕部电机型号（EL05 或 RS05） |
| `--wait-sec` | `25` | launch 启动后等待秒数 |
| `--test-zero-torque` | 关闭 | 启用 Phase 6 零力矩控制器切换测试 |

### 输出示例

```
============================================
 EL-A3 ROS2 启动测试 Demo
 模式: mock | 时间: 2026-03-24 16:35:26
============================================

Phase 1: 系统启动与健康检查
  [PASS] controller_manager 节点在线
  [PASS] robot_state_publisher 节点在线
  ...

Phase 4: 基础运动测试
  [PASS] 归零位 完成
  [PASS] L2 单关节 +0.3 完成
  ...

============================================
 结果: 17 passed, 0 failed
============================================
```

---

## 零力矩模式（拖动示教）

`zero_torque_controller` 是一个自定义 ros2_control 控制器插件，实现 Kp=0 + Pinocchio RNEA 重力补偿，允许手动拖动机械臂。

### 使用方法

```bash
# 启动（robot.launch.py 已自动加载为 inactive 状态）
ros2 launch el_a3_moveit_config robot.launch.py

# 切换到零力矩模式（停用 arm_controller，激活 zero_torque_controller）
ros2 control switch_controllers \
  --deactivate arm_controller \
  --activate zero_torque_controller

# 恢复轨迹控制
ros2 control switch_controllers \
  --deactivate zero_torque_controller \
  --activate arm_controller
```

### 参数

在 `el_a3_controllers.yaml` 中配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kd` | 1.0 | 阻尼增益（越小越容易拖动，越大越稳定） |
| `joints` | L1-L7 | 参与零力矩模式的关节 |

---

## CAN 接口配置

```bash
# 使用提供的脚本
sudo bash scripts/setup_can.sh can0 1000000

# 或手动配置
sudo modprobe gs_usb
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up

# 验证
ip link show can0
candump can0    # 查看 CAN 帧
```

---

## DDS / FastRTPS 配置

项目使用 `fastrtps_no_shm.xml` 禁用共享内存传输，适用于 Docker 或多机通信场景。Launch 文件会自动设置 `FASTRTPS_DEFAULT_PROFILES_FILE` 环境变量。

手动设置：

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fastrtps_no_shm.xml
```

---

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

单独编译某个包：

```bash
colcon build --symlink-install --packages-select el_a3_hardware
colcon build --symlink-install --packages-select el_a3_moveit_config
```

---

## 故障排查

### CAN 通信无响应

```bash
# 检查 CAN 接口是否 UP
ip link show can0

# 检查 USB-CAN 适配器
lsusb | grep -i can
dmesg | tail -20

# 重置 CAN
sudo ip link set can0 down
sudo modprobe -r gs_usb && sleep 1 && sudo modprobe gs_usb
sudo bash scripts/setup_can.sh can0 1000000
```

### 控制器启动超时

```bash
# 检查 controller_manager 是否运行
ros2 node list | grep controller_manager

# 查看控制器状态
ros2 control list_controllers

# 查看硬件接口
ros2 control list_hardware_interfaces
```

### MoveIt 规划失败

- 检查起始状态是否在碰撞中：RViz 中起始位姿应为绿色
- 增大 `allowed_start_tolerance` 参数
- 尝试不同的规划器（如 RRTstar、PRM）

### 电机温度过高

```bash
# 监控温度
ros2 topic echo /debug/motor_temperature
```

超过 60°C 时建议暂停操作。

---

## 相关文档

| 文档 | 说明 |
|------|------|
| [ROS_INTERFACE_REFERENCE.md](ROS_INTERFACE_REFERENCE.md) | 全部 ROS Topic / Action / Service / TF / Launch 参数 |
| [XBOX_CONTROL_SETUP.md](XBOX_CONTROL_SETUP.md) | Xbox 手柄安装与配置 |
| [XBOX_HOW_TO_USE.md](XBOX_HOW_TO_USE.md) | Xbox 手柄操作说明 |
| [scripts/GRAVITY_CALIBRATION_README.md](scripts/GRAVITY_CALIBRATION_README.md) | 重力补偿标定 |
| [hardware/electronics/README.md](hardware/electronics/README.md) | 电子硬件说明 |
| [hardware/mechanical/README.md](hardware/mechanical/README.md) | 机械结构说明 |
