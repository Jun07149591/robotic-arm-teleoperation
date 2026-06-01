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
