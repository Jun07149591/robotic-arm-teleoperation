# Xbox 手柄控制 — 使用说明

## 系统组件

启动后会运行以下节点：

- **ros2_control_node** — 硬件控制管理（CAN 总线通信）
- **robot_state_publisher** — 机器人 URDF 模型发布
- **joint_state_broadcaster** — 关节状态广播
- **arm_controller** — L1-L6 关节轨迹控制器
- **gripper_controller** — L7 夹爪控制器
- **joy_node** — 手柄驱动（读取 `/dev/input/js*`）
- **xbox_teleop_node** — 手柄映射 + Jacobian IK 控制

## 启动

### 实机控制

```bash
# 配置 CAN 接口
sudo bash scripts/setup_can.sh can0 1000000

# 启动
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch el_a3_teleop real_xbox_teleop.launch.py can_interface:=can0
```

### 仿真模式（无硬件）

```bash
ros2 launch el_a3_teleop real_xbox_teleop.launch.py use_mock_hardware:=true
```

### 带 RViz 可视化

```bash
ros2 launch el_a3_teleop real_xbox_teleop.launch.py can_interface:=can0 use_rviz:=true
```

## 初始化流程

程序启动时自动执行：
1. 等待关节状态数据，确认所有电机在线
2. 读取并显示各关节初始位置
3. 规划并执行回 Home 位置：L1=0, L2=45, L3=-45, L4=0, L5=0, L6=0 (度)

## 操控方式

### 平移控制（末端位移）

| 输入 | 轴 | 说明 |
|------|----|------|
| 左摇杆 左右 | X | 左右平移 |
| 左摇杆 上下 | Y | 前后平移 |
| LT 扳机 | -Z | 向下 |
| RT 扳机 | +Z | 向上 |

### 旋转控制（末端姿态）

| 输入 | 轴 | 说明 |
|------|----|------|
| 右摇杆 左右 | Yaw | 绕 Z 轴偏航 |
| 右摇杆 上下 | Pitch | 绕 Y 轴俯仰 |
| LB 肩键 | -Roll | 绕 X 轴逆时针 |
| RB 肩键 | +Roll | 绕 X 轴顺时针 |

### 功能键

| 按键 | 功能 |
|------|------|
| A | 切换速度档位（5 档循环） |
| B | 回 Home 位置 |
| X | 回零位 |
| Y | 零力矩模式 |
| Back | 急停 |

### 夹爪控制

| 输入 | 功能 |
|------|------|
| D-pad 上 | 按住连续收紧夹爪 |
| D-pad 下 | 按住连续打开夹爪 |

夹爪控制现在和 PICO 遥操作一致：按住 D-pad 时按速度连续改变目标角，并通过 `/gripper_controller/torque_limit` 发送力矩上限；松开后如果夹爪仍处于闭合状态，会周期性重发当前位置和保持力矩上限。硬件层会把该值映射为 L7 电机 `LIMIT_TORQUE`，所以不是只靠位置命令硬顶。

## 当前配置参数

配置文件：`el_a3_teleop/config/xbox_teleop.yaml`

| 参数 | 值 | 说明 |
|------|----|------|
| `update_rate` | 50.0 Hz | 控制频率 |
| `max_linear_velocity` | 0.15 m/s | 最大平移速度 |
| `max_angular_velocity` | 1.5 rad/s | 最大旋转速度 |
| `deadzone` | 0.15 | 摇杆死区 |
| `input_smoothing` | 0.35 | EMA 平滑系数 |
| `trajectory_time_from_start` | 0.08 s | 轨迹时间步长 |
| `gripper_speed` | 3.0 rad/s | 夹爪开合速度 |
| `gripper_max_angle` | 2.0 rad | 夹爪最大闭合目标 |
| `gripper_deadzone` | 0.2 | D-pad 输入死区 |
| `gripper_hold_interval` | 0.25 s | 松开后夹爪保持命令重发间隔 |
| `gripper_close_effort` | 0.65 Nm | 闭合/夹取时的夹爪力矩上限 |
| `gripper_hold_effort` | 0.65 Nm | 松开 D-pad 后的保持力矩上限 |

如果夹不住物体，先把 `gripper_close_effort` 和 `gripper_hold_effort` 从 `0.65` 小幅提高到 `0.8` 或 `1.0` 测试；不要长时间大力保持。若需要长时间夹住轻物体，可以把 `gripper_hold_effort` 降到 `0.16` 左右减小发热。硬件层还有 `gripper_default_torque_limit` 和 `gripper_hold_torque_ff` 参数，可通过 `/el_a3_hw_debug` 动态调整。

## 无手柄时的替代操作

如果没有 Xbox 手柄，可以通过 MoveIt 界面控制：

```bash
# 启动 MoveIt demo（仿真）
ros2 launch el_a3_moveit_config demo.launch.py

# 或启动真实硬件 + MoveIt
ros2 launch el_a3_moveit_config robot.launch.py can_interface:=can0
```

在 RViz 的 MotionPlanning 面板中拖动交互标记，使用 "Plan & Execute" 执行运动。

## 监控

```bash
# 查看关节状态
ros2 topic echo /joint_states

# 查看手柄原始输入
ros2 topic echo /joy

# 查看所有 topic
ros2 topic list

# 查看控制器状态
ros2 control list_controllers
```

## 停止系统

在启动 launch 的终端中按 `Ctrl+C`。
