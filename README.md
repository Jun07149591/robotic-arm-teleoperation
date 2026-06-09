# robotic-arm-teleoperation

> AE/EL-A3 机械臂的 PICO/WebXR 遥操作使用说明。  
> 兄弟项目：https://github.com/Beiyu-kk/bookarm-control-py

## 1. 这套遥操作是怎么跑起来的

```text
PICO 头显浏览器
  -> pico3_webxr_pose_receiver.py
  -> /tmp/pico_latest_pose.json
  -> pico_teleop_node.py 或 demo/pico_control_jointctrl.py
  -> 机械臂关节控制
```

你需要先让 PICO 设备把手柄位姿发到电脑，再启动遥操作程序去读取这些数据。

## 2. 支持的两种使用方式

### ROS 2 方式

适合已经在用 `ros2_control` 的场景。

- 启动 `el_a3_teleop`
- 输出到 `JointTrajectory`
- 可接仿真，也可接真实机械臂

### SDK 方式

适合直接连 CAN 的场景。

- 运行 `el_a3_sdk/demo/pico_control_jointctrl.py`
- 不依赖 ROS
- 支持单臂和双臂

## 3. 先启动 PICO 数据接收器

先运行：

```bash
python3 pico3_webxr_pose_receiver.py
```

然后用 PICO 头显打开对应页面，确认手柄数据已经在接收。

## 4. ROS 2 使用方法

### 单臂

```bash
ros2 launch el_a3_teleop pico_teleop.launch.py can_interface:=can0
```

### 仿真

```bash
ros2 launch el_a3_teleop pico_teleop.launch.py use_mock_hardware:=true use_rviz:=true
```

### 双臂

```bash
ros2 launch el_a3_teleop pico_teleop_dual.launch.py use_mock_hardware:=true
```

## 5. SDK 使用方法

### 单臂

```bash
python3 el_a3_sdk/demo/pico_control_jointctrl.py --can can0
```

### 双臂

```bash
python3 el_a3_sdk/demo/pico_control_jointctrl.py --can-left can0 --can-right can1
```

### 仿真

```bash
python3 el_a3_sdk/demo/pico_control_jointctrl.py --sim
```

## 6. 遥操作怎么用

### 基本流程

1. 启动数据接收器
2. 启动 ROS 2 或 SDK 遥操作程序
3. 戴上 PICO 头显，打开 WebXR 页面
4. 让程序识别到手柄数据
5. 按住握持键开始跟踪
6. 移动手柄，机械臂末端跟着动
7. 松开握持键，机械臂保持当前位置

### 按键说明

- `grip`：按住约 1 秒开始跟踪，松开停止跟踪
- `trigger`：按住进入细 yaw 模式
- `thumbstick`：控制夹爪
- `A/X`：回零位
- `B/Y`：回 Home
- `左摇杆按下`：零力矩模式
- `右摇杆按下`：急停

## 7. 使用时的顺序

推荐按这个顺序操作：

1. 先检查 CAN 或仿真是否启动
2. 再启动 `pico3_webxr_pose_receiver.py`
3. 再启动遥操作程序
4. 最后在 PICO 端开始控制

如果是实机，先确认机械臂已经上电并且控制接口正常。

## 8. 常用配置

### ROS 2 参数

文件：`el_a3_ros/el_a3_teleop/config/pico_teleop.yaml`

常见参数：

- `hand`
- `position_scale`
- `deadband`
- `pos_filter_alpha`
- `target_filter_alpha`
- `filter_omega`
- `max_ik_jump`
- `workspace_radius`

### 启动文件

- `el_a3_ros/el_a3_teleop/launch/pico_teleop.launch.py`
- `el_a3_ros/el_a3_teleop/launch/pico_teleop_dual.launch.py`
- `el_a3_ros/el_a3_teleop/launch/pico_teleop_mock.launch.py`

## 9. 数据采集（训练 VLA 用）

遥操作的同时录制训练数据，输出 **LeRobot v3.0** 兼容格式（Parquet + MP4）。现在支持两种控制器采集：

- PICO：写入 `observation.pico`
- XBOX：写入 `observation.xbox`

两种 controller feature 不混在同一个数据集里，建议分别使用不同 `--repo-id`。
`--fps` 必须和采样 `--hz` 一致；例如 `--hz 15 --fps 15`，这样 parquet 每一行和 MP4 每一帧共享同一时间基。

### 9.1 PICO 采集

**做完任务 → 长按 A 失能 → 自动保存**，全程不碰键盘：

```bash
# 终端 1：遥操作（带状态导出）
python el_a3_sdk/demo/pico_control_jointctrl.py --can can0 \
  --state-export /tmp/robot_latest_state.json

# 终端 2：采集（纯观察者，不连 CAN）
python teleop_data_collection/scripts/record_sdk_episode.py \
  --camera-serial 260322277792 \
  --state-file /tmp/robot_latest_state.json \
  --repo-id my_dataset \
  --task "pick up the object" \
  --hz 15 --fps 15 --max-duration 60
```

| 终止方式 | 触发 | success |
|---------|------|---------|
| 长按 A (右手) | 机械臂失能 | ✅ 自动 |
| 按 `q` | 采集终端或预览窗口 | ✅ 自动 |
| 按 `f` | 采集终端 | ❌ 保存为失败 |
| 预览窗口按 `Esc` | 仅 `--preview` 时 | ✅ 自动 |
| `--max-duration N` | 超时兜底 | ❌ |
| Ctrl+C | 手动 | ❌ |

默认 `q` 会保存当前 episode 为成功，`f` 会保存当前 episode 为失败；可以用 `--keyboard-stop-key e` / `--keyboard-fail-key x` 改成其它单键，或用空字符串禁用。

### 9.2 XBOX 采集

XBOX 采集使用 ROS2 XBOX 遥操作控制机械臂，另开一个 exporter 把 `/joint_states` 和 `/joy` 导出成采集脚本可读的 JSON。采集脚本仍然是纯观察者，只读相机和文件，不直接控制机械臂。`--profile auto` 会按 `/dev/input/js0` 自动识别手柄映射；Zikway HID 这类手柄会使用 Start=11，而标准 Xbox 使用 Start=7。

一次完整 XBOX 采集：

```text
real_xbox_teleop.launch.py
  -> /joint_states + /joy
  -> export_ros_xbox_state.py
  -> /tmp/robot_latest_state.json + /tmp/xbox_latest_input.json
  -> record_xbox_episode.py
  -> LeRobot v3.0 数据集
```

```bash
# 终端 1：启动 ROS2 XBOX 实机遥操作
cd el_a3_ros
sudo bash scripts/setup_can.sh can0 1000000
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch el_a3_teleop real_xbox_teleop.launch.py can_interface:=can0

# 终端 2：导出 ROS2 机器人状态和 XBOX 输入
cd /home/jun/library_robot/EDULITE_A3
source /opt/ros/humble/setup.bash
/usr/bin/python3 teleop_data_collection/scripts/export_ros_xbox_state.py \
  --state-file /tmp/robot_latest_state.json \
  --gamepad-file /tmp/xbox_latest_input.json \
  --profile auto \
  --device /dev/input/js0

# 终端 3：采集 XBOX episode
python teleop_data_collection/scripts/record_xbox_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --gamepad-file /tmp/xbox_latest_input.json \
  --repo-id my_edulite_xbox_dataset \
  --task "pick up the object" \
  --hz 15 \
  --fps 15 \
  --max-duration 60 \
  --preview
```

连续采集多条 episode 时，在终端 3 加 `--continuous`。每次长按 `Start` 会保存当前 episode，松开 `Start` 后自动进入下一条；`Ctrl+C` 退出连续采集。

```bash
python teleop_data_collection/scripts/record_xbox_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --gamepad-file /tmp/xbox_latest_input.json \
  --repo-id my_edulite_xbox_dataset \
  --task "pick up the object" \
  --hz 15 \
  --fps 15 \
  --max-duration 60 \
  --preview \
  --continuous
```

如果只想连续采固定数量，可以加：

```bash
--continuous --max-episodes 20
```

XBOX 结束方式：

| 终止方式 | 触发 | success |
|---------|------|---------|
| 长按 Start 1 秒 | exporter 写 `episode_done=true` | ✅ 自动 |
| 按 `q` | 采集终端或预览窗口 | ✅ 自动，并停止 `--continuous` |
| 按 `f` | 采集终端 | ❌ 保存为失败，`--continuous` 下继续下一条 |
| 预览窗口按 `Esc` | 仅 `--preview` 时 | ✅ 自动，并停止 `--continuous` |
| `--max-duration N` | 超时兜底 | ❌ |
| Ctrl+C | 手动 | ❌ |

默认 `q` 会保存当前 episode 为成功并结束整个采集会话；`f` 会保存当前 episode 为失败，连续采集时继续进入下一条。可以用 `--keyboard-stop-key e` / `--keyboard-fail-key x` 改成其它单键，或用空字符串禁用。

XBOX/PICO 的 LeRobot episode 数量以 `meta/info.json` 的 `total_episodes`、`data/chunk-*/*.parquet` 和 `videos/*/chunk-*/*.mp4` 为准。根目录下的 `episode_000xxx/` 只是 raw 调试日志；异常中断可能留下空目录，不能用最大 raw 序号判断视频数量。若追加时提示下一条 raw 目录已存在，说明旧 repo 的 raw 目录和 LeRobot 数据已经错位，建议换新的 `--repo-id` 重新采。

XBOX LeRobot controller feature 为 `observation.xbox`，shape 为 `(20,)`：

```text
lx, ly, rx, ry, lt, rt, dpad_x, dpad_y,
btn_a, btn_b, btn_x, btn_y, btn_lb, btn_rb,
btn_back, btn_start, valid, speed_level, mode_normal, episode_done
```

XBOX 常用按键：

| 输入 | 功能 |
|------|------|
| 左摇杆 | 末端 X/Y 平移 |
| LT / RT | 末端 Z 下/上 |
| 右摇杆 | 姿态 Yaw/Pitch |
| LB / RB | Roll |
| D-pad 上/下 | 按住连续收紧/打开夹爪，松开后带夹持力保持 |
| A | 切换速度档 |
| B | 回 Home |
| X | 回零位 |
| Y | 零力矩模式 |
| Back | 急停 |
| Start 长按 | 结束当前 episode |

XBOX 夹爪已按 PICO 遥操作方式增加夹持力保持：闭合时发送目标角，并通过 `/gripper_controller/torque_limit` 发布 `gripper_close_effort`；松开后周期性重发目标角和 `gripper_hold_effort`。ROS 硬件层会把该 torque limit 映射为 L7 电机 `LIMIT_TORQUE`，用于提升夹持保持力；如果仍夹不住，优先小幅提高 `el_a3_ros/el_a3_teleop/config/xbox_teleop.yaml` 里的 `gripper_close_effort` 和 `gripper_hold_effort`。

详见 [`teleop_data_collection/README.md`](teleop_data_collection/README.md)。
