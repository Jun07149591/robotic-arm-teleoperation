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

遥操作的同时录制训练数据，输出 **LeRobot v3.0** 兼容格式（Parquet + MP4）。

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
  --hz 15 --fps 30 --max-duration 60
```

| 终止方式 | 触发 | success |
|---------|------|---------|
| 长按 A (右手) | 机械臂失能 | ✅ 自动 |
| `--max-duration N` | 超时兜底 | ❌ |
| Ctrl+C | 手动 | ❌ |

详见 [`teleop_data_collection/README.md`](teleop_data_collection/README.md)。

