# Xbox 手柄遥操作 — 安装与配置

## 功能包

`el_a3_teleop` 提供基于 Jacobian IK 的笛卡尔空间遥操作控制，通过 `joy_node` 读取手柄输入，`xbox_teleop_node` 将其映射为末端执行器增量运动。

## 控制映射

| 输入 | 功能 | 说明 |
|------|------|------|
| 左摇杆 左右 | X 轴平移 | 机器人左右移动 |
| 左摇杆 上下 | Y 轴平移 | 机器人前后移动 |
| LT (左扳机) | Z 轴向下 | 按下越深，下降越快 |
| RT (右扳机) | Z 轴向上 | 按下越深，上升越快 |
| 右摇杆 左右 | Yaw 旋转 | 绕 Z 轴旋转 |
| 右摇杆 上下 | Pitch 旋转 | 绕 Y 轴旋转 |
| LB (左肩键) | Roll 向左 | 绕 X 轴逆时针旋转 |
| RB (右肩键) | Roll 向右 | 绕 X 轴顺时针旋转 |
| A 键 | 切换速度档位 | 5 档：超慢 / 慢速 / 中速 / 快速 / 极速 |
| B 键 | 回 Home | 规划并执行回 Home 位置 |

## 安装依赖

```bash
sudo apt update
sudo apt install ros-humble-joy joystick
```

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select el_a3_teleop
source install/setup.bash
```

## 启动

### 真实硬件

```bash
# 1. 配置 CAN
sudo bash scripts/setup_can.sh can0 1000000

# 2. 启动遥操作（自动包含 el_a3_control + joy + teleop）
ros2 launch el_a3_teleop real_xbox_teleop.launch.py can_interface:=can0
```

### 仿真模式

```bash
ros2 launch el_a3_teleop real_xbox_teleop.launch.py use_mock_hardware:=true
```

### Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `auto_detect_controller` | `true` | 自动检测手柄类型并选择 profile |
| `joy_dev_override` | `""` | 手动指定手柄设备（如 `/dev/input/js1`） |
| `controller_profile_override` | `""` | 强制使用指定 profile（如 `xbox_default`） |
| `can_interface` | `can0` | CAN 接口名 |
| `use_rviz` | `false` | 启动 RViz |
| `use_mock_hardware` | `false` | 使用虚拟硬件 |

## 配置调整

编辑 `el_a3_teleop/config/xbox_teleop.yaml`：

```yaml
xbox_teleop_node:
  ros__parameters:
    update_rate: 50.0            # 控制频率 (Hz)
    max_linear_velocity: 0.15    # 最大平移速度 (m/s)
    max_angular_velocity: 1.5    # 最大旋转速度 (rad/s)
    deadzone: 0.15               # 摇杆死区 (0~1)
    input_smoothing: 0.35        # 输入平滑系数
    filter_omega: 14.0           # 低通滤波截止频率
    max_ik_jump: 0.5             # IK 跳跃限制
    trajectory_time_from_start: 0.08  # 轨迹时间步长 (s)
```

## 调试

```bash
# 检查手柄连接
ls /dev/input/js*

# 测试手柄输入
ros2 topic echo /joy

# 查看关节状态
ros2 topic echo /joint_states

# 检查手柄权限
sudo chmod 666 /dev/input/js0
# 或永久加入 input 组
sudo usermod -a -G input $USER
```

## 安全提示

1. 首次使用建议先在仿真模式下测试
2. 从最低速度开始
3. 确保工作空间内无障碍物
4. 准备好使用 `Ctrl+C` 急停
5. 避免长时间连续操作（防止电机过热）
