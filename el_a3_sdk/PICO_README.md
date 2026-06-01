# Pico VR 手柄遥操作使用文档

## 概述

通过 Pico VR 头显的 WebXR 手柄实现 EL-A3 机械臂的实时遥操作控制。

提供两种运行方式：
- **SDK 模式**（`demo/pico_control.py`）：纯 SDK，无需 ROS，适合快速测试和实机
- **ROS 模式**（`ros2 launch`）：集成 ros2_control + RViz 可视化 + mock hardware

两种模式均支持双机械臂：左手柄控制左臂，右手柄控制右臂。

## 系统架构

```
Pico 头显浏览器 (WebXR)  →  WebSocket  →  pico3_webxr_pose_receiver.py
                                              ↓
                                     /tmp/pico_latest_pose.json
                                              ↓
                          ┌───────────────────┴───────────────────┐
                          ↓                                       ↓
                  pico_control.py (SDK)              pico_teleop_node (ROS)
                          ↓                                       ↓
                     ELA3Interface                    /arm_controller/joint_trajectory
                     (CAN 直连)                              ↓
                                                    ros2_control
                                                   ↙            ↘
                                        mock_components    RsA3HardwareInterface
                                        (仿真)              (CAN 实机)
```

## 前置条件

1. **Pico 头显**：Pico 4 / Pico 4 Ultra 等支持 WebXR 的设备
2. **网络连接**：Pico 和 PC 在同一局域网
3. **Python 环境**：conda 环境 `lingzuarm`（Python 3.12）
4. **Pinocchio**：`pip install pin`
5. **ROS 2 Humble**（仅 ROS 模式）

### Python 版本说明

系统存在两个 Python 环境：
- **conda** `lingzuarm`：Python 3.12（用于 SDK 和 pico receiver）
- **系统** `/usr/bin/python3`：Python 3.10（ROS 2 Humble 使用）

`el_a3_sdk` 通过 `pip install -e .` 安装到 conda Python 3.12。ROS 节点由系统 Python 3.10 启动，因此代码中已添加 `sys.path` 自动查找 SDK 路径（见 `pico_teleop_node.py` 顶部）。如需手动验证：

```bash
# SDK 模式（conda）
python -c "from el_a3_sdk.kinematics import ELA3Kinematics; print('OK')"

# ROS 模式（系统 Python）
/usr/bin/python3 -c "from el_a3_teleop.pico_teleop_node import PicoTeleopNode; print('OK')"
```

---

## 方式一：SDK 模式（快速测试，无 ROS）

### 仿真

```bash
# 终端 1：Pico 数据接收器
python pico3_webxr_pose_receiver.py

# 终端 2：仿真
cd el_a3_sdk
python demo/pico_control.py --sim
```

### 实机

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

cd el_a3_sdk
# 双机械臂
python demo/pico_control.py --can-left can0 --can-right can1
# 单机械臂
python demo/pico_control.py --can can0
```

### 命令行参数

```
python pico_control.py [选项]

  --sim               仿真模式（无需 CAN 硬件，纯 FK/IK 模拟）
  --can CAN           单臂模式 CAN 接口
  --can-left CAN      左臂 CAN 接口 (默认: can0)
  --can-right CAN     右臂 CAN 接口 (默认: can1)
  --rate RATE         控制频率 Hz (默认: 100)
  --pos-scale S       位置缩放 (默认: 0.1)
  --kp KP             位置增益 (默认: 80)
  --kd KD             速度增益 (默认: 4)
  --deadzone D        位移死区 m (默认: 0.02)
  --debug             调试模式
```

---

## 方式二：ROS 模式（RViz 可视化 + mock hardware）

### 编译

```bash
cd ~/library_robot/EDULITE_A3/el_a3_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install --packages-select el_a3_teleop
source install/setup.bash
```

### 仿真 + RViz 可视化

```bash
# 终端 1：Pico 数据接收器
cd ~/library_robot/EDULITE_A3
python pico3_webxr_pose_receiver.py

# 终端 2：ROS 遥操作 + RViz
cd ~/library_robot/EDULITE_A3/el_a3_ros

# conda 环境会污染 Qt 插件路径，必须先清理
unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH QML2_IMPORT_PATH
source /opt/ros/humble/setup.bash
source install/setup.bash

# 仿真 + RViz
ros2 launch el_a3_teleop pico_teleop.launch.py use_mock_hardware:=true use_rviz:=true

# 纯仿真（无 RViz）
ros2 launch el_a3_teleop pico_teleop.launch.py use_mock_hardware:=true
```

**注意**：如果终端提示符显示 `(lingzuarm)`，conda 的 Qt 库会阻止 RViz 窗口正常显示。必须先 `unset` 那三个环境变量。

### 实机

```bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch el_a3_teleop pico_teleop.launch.py can_interface:=can0
```

### Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_mock_hardware` | `false` | 仿真模式，无需 CAN 硬件 |
| `use_rviz` | `false` | 启动 RViz2 可视化 |
| `can_interface` | `can0` | CAN 接口名 |

### ROS 节点参数

配置文件：`el_a3_teleop/config/pico_teleop.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hand` | `"right"` | 跟踪哪只手 (left/right) |
| `position_scale` | `0.05` | 手柄位移到末端的缩放 |
| `deadzone` | `0.02` | 位移死区 m |
| `input_smoothing` | `0.18` | 输入 EMA 平滑系数 |
| `pos_filter_alpha` | `0.35` | 手部位置 EMA 滤波系数 |
| `filter_omega` | `14.0` | 2 阶滤波器截止频率 |
| `max_ik_jump` | `0.5` | IK 跳变保护阈值 rad |

---

## 按钮映射

| Pico 按键 | 位置 | 功能 |
|----------|------|------|
| btn[1] **背部握持键** | 中指位置 | **按住 1 秒**开启跟踪，松开关闭（手臂保持位置） |
| btn[0] **侧边扳机** | 食指位置 | 按住夹爪闭合，松开夹爪停止 |
| btn[4] **A (右) / X (左)** | 正面 | MoveJ 回零位 |
| btn[5] **B (右) / Y (左)** | 正面 | MoveJ 回 Home |
| btn[3] **摇杆按下** | — | 左手切换速度档位，右手急停 |

- **背部握持键**：跟踪开启后，手部移动 → 机械臂末端对应移动。松开立刻停住保持位置。
- **侧边扳机**：仅右手。按住夹爪持续闭合，松开停止。
- **A (右) / X (左)**：对应臂 MoveJ 回到零关节位置 `[0,0,0,0,0,0]`。
- **B (右) / Y (左)**：对应臂 MoveJ 回到 Home `[0, 45°, -45°, 0, 0, 0]`。

## 控制原理：位移对位移映射

### 参考点与标定

程序在以下时机自动记录**两个锚点**：

| 锚点 | 含义 |
|------|------|
| `ref_pos` | 标定时刻的手柄空间坐标 |
| `calib_pose` | 标定时刻的机械臂末端 FK 位姿 |

标定时机：
1. 启动后自动标定
2. 每次 A/B/X/Y 按键 MoveJ 到达目标后重新标定
3. 握持键按住 1 秒开启跟踪时重新标定

### 映射公式

```
target_pose = calib_pose + (手柄当前位置 - ref_pos) × position_scale
```

每帧 IK 求解 `target_pose` → 关节角 → 下发轨迹。等效于末端笛卡尔空间连续 MoveL。

### 举例（scale=0.05, speed=T1=0.10, 综合=0.005）

| 手柄动作 | 机械臂末端响应 |
|---------|-------------|
| 右移 20cm | 右移 1mm |
| 上移 20cm | 上移 1mm |
| 前移 20cm | 前移 1mm |
| 回到原位 | 回到标定位姿 |


## 坐标系映射

Pico WebXR `local-floor` 参考空间到机械臂基座标：

| Pico 方向 | WebXR 轴 | 机器人轴 |
|----------|---------|---------|
| 左       | -X      | -X      |
| 右       | +X      | +X      |
| 上       | +Y      | +Y      |
| 前       | -Z      | -Z      |

## 速度档位

左手摇杆按下循环切换，5 档，影响最大跟踪速度（默认 1 档起步）：

| 档位 | 缩放因子 |
|------|---------|
| 1    | 0.10    |
| 2    | 0.25    |
| 3    | 0.50    |
| 4    | 0.75    |
| 5    | 1.00    |

## 故障排查

### `ModuleNotFoundError: No module named 'el_a3_sdk'`（ROS 启动报错）

ROS 2 Humble 使用系统 Python 3.10，而 `el_a3_sdk` 通过 `pip install -e .` 装在 conda Python 3.12。代码中已通过 `sys.path` 自动查找 SDK 路径（`pico_teleop_node.py` 顶部和 `pico_teleop.launch.py` 中的 PYTHONPATH）。

如仍报错，手动验证：

```bash
# 确认 conda 下 SDK 安装状态
pip install -e /home/jun/library_robot/EDULITE_A3/el_a3_sdk

# 确认系统 Python 能找到 SDK
bash -c 'source /home/jun/library_robot/EDULITE_A3/el_a3_ros/install/setup.bash && \
  /usr/bin/python3 -c "from el_a3_teleop.pico_teleop_node import PicoTeleopNode; print(\"OK\")"'
```

### RViz 窗口不显示

```bash
unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH QML2_IMPORT_PATH
wmctrl -l | grep RViz      # 确认窗口存在
wmctrl -a RViz              # 提到最前
```

### 未检测到 Pico 数据

- 确认 `pico3_webxr_pose_receiver.py` 已在运行
- 确认 Pico 头显已进入 VR
- 检查：`cat /tmp/pico_latest_pose.json | python3 -c "import json,sys; print(len(json.load(sys.stdin).get('inputSources',[])))"`

### 机械臂不跟随手部移动

- 检查 `pico_teleop_node` 是否正常启动（终端应有 "Pinocchio kinematics initialized"、"Joint states received"、"Initialized" 日志）
- 如果终端没有 `pico_teleop_node` 输出，查看上方是否有 `ModuleNotFoundError` 红色报错
- 握持键（背部）是否已按住超过 1 秒？
- 确认终端有 "Tracking ON" 日志
- 确认标定完成（"Calibrated"）
- 如果机械臂动作幅度过大，降低 `config/pico_teleop.yaml` 中的 `position_scale`

### Pico 按键无反应

1. 先确认数据管道正常：`cat /tmp/pico_latest_pose.json | python3 -c "import json,sys; d=json.load(sys.stdin); [print(f'{s[\"handedness\"]}: {s[\"gamepad\"][\"buttons\"]}') for s in d.get('inputSources',[])]"`
2. 如果所有按键 `pressed: false` 且无变化，说明 Pico 浏览器未上报按键——重新 Enter VR 或重启 Pico 浏览器
3. 如果按键数据正常但机械臂不动，检查终端中 `pico_teleop_node` 是否已启动成功（见上一条）
4. Pico 手柄处于 `emulatedPosition: true`（3DoF）时按键仍然有效，不影响按钮功能

### 机械臂抽搐

- 真机不会出现（物理阻尼吸收）
- 仿真中按侧键时避免手指挤压背部握持键

### conda 环境下 RViz 不显示

conda 的 Qt 库与系统 ROS 2 RViz 不兼容。每次启动前执行：

```bash
unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH QML2_IMPORT_PATH
```

## 文件清单

| 文件 | 位置 | 说明 |
|------|------|------|
| Pico SDK 控制程序 | `el_a3_sdk/demo/pico_control.py` | SDK 模式（--sim + 实机） |
| Pico ROS 遥操作节点 | `el_a3_ros/el_a3_teleop/el_a3_teleop/pico_teleop_node.py` | ROS 2 节点 |
| Pico ROS 启动文件 | `el_a3_ros/el_a3_teleop/launch/pico_teleop.launch.py` | ros2 launch 入口 |
| Pico ROS 配置文件 | `el_a3_ros/el_a3_teleop/config/pico_teleop.yaml` | 节点参数 |
| Pico 数据接收器 | `pico3_webxr_pose_receiver.py` | WebXR WebSocket 服务器 |
| Xbox 手柄（参考） | `el_a3_sdk/demo/xbox_control.py` | SDK 参考实现 |
| Xbox ROS 遥操作 | `el_a3_ros/el_a3_teleop/` | ROS 参考实现 |
| 本使用文档 | `el_a3_sdk/PICO_README.md` | — |
