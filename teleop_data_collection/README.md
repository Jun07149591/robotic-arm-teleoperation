# Teleop Data Collection

面向 AE/EL-A3 + PICO SDK 遥操作的数据采集目录。输出 **LeRobot v3.0 兼容格式**。

## 采集边界

- 遥操作控制使用当前调好的 `el_a3_sdk/demo/pico_control_jointctrl.py`
- PICO 原始包仍由 `pico3_webxr_pose_receiver.py` 写入 `/tmp/pico_latest_pose.json`
- D405 腕部相机 + D455/D435 侧边相机作为 RGB 观测来源
- 默认只保存两路 RGB MP4、低维 Parquet 和 JSONL 元数据；不再逐帧保存 RGB/depth PNG
- 需要排查相机或深度图时，采集命令加 `--save-raw-images` 才会额外保存 color/depth/depth_vis PNG
- 采集脚本记录 episode 级数据

## LeRobot 输出格式

```
datasets/<repo_id>/              # 数据集根目录（多 episode 共享）
  meta/
    info.json                    # LeRobot v3.0 特征定义、fps、总帧数
    stats.json                   # 官方 writer 计算的归一化统计
    tasks.parquet                # task -> task_index
    episodes/
      chunk-000/
        file-000.parquet         # episode 元数据、数据文件索引、视频时间范围
  data/
    chunk-000/
      file-000.parquet           # 低维传感器数据，多个 episode 可追加到同一 shard
  videos/
    observation.images.wrist/
      chunk-000/
        file-000.mp4             # D405 腕部 RGB 视频 shard
    observation.images.side/
      chunk-000/
        file-000.mp4             # D455/D435 侧边 RGB 视频 shard
  episode_000001/                # 每 episode 子目录 (原始日志 + 可选调试 PNG)
    meta.json                    # 兼容旧格式的元数据
    raw/                         # 原始 JSONL 日志 (调试用)
      camera_frames.jsonl
      robot_states.jsonl
      pico_packets.jsonl
      wrist/                     # 仅 --save-raw-images 时生成
      side/                      # 仅 --save-raw-images 时生成
```

## 数据特征 (Features)

| Feature | Shape | Dtype | 说明 |
|---------|-------|-------|------|
| `observation.state` | (28,) | float32 | qpos(7) + qvel(7) + tau(7) + ee_pose(6) + gripper |
| `observation.images.wrist` | (480, 640, 3) | video | D405 腕部 RGB 帧 |
| `observation.images.side` | (480, 640, 3) | video | D455/D435 侧边 RGB 帧 |
| `observation.pico` | (15,) | float32 | PICO grip 位姿(7) + 按键(4) + axes(4) |
| `observation.xbox` | (20,) | float32 | XBOX 摇杆/扳机/D-pad(8) + 按键(8) + valid/speed/mode/done |
| `action` | (7,) | float32 | 关节/夹爪增量 |
| `timestamp` | (1,) | float32 | 官方 LeRobot 自动写入 |
| `frame_index` | (1,) | int64 | 官方 LeRobot 自动写入 |
| `episode_index` | (1,) | int64 | 官方 LeRobot 自动写入 |
| `index` | (1,) | int64 | 官方 LeRobot 自动写入的全局帧索引 |
| `task_index` | (1,) | int64 | 官方 LeRobot 自动写入 |
| `next.done` | (1,) | bool | episode 结束标志，只在最后一帧为 true |
| `next.success` | (1,) | bool | episode 成功标志，只在最后一帧按结果标记 |

底层写入使用官方 `lerobot` 包的 `LeRobotDataset.create/resume`，不是手写伪格式。采集帧会在写入前对齐到 schema 中的图像尺寸；默认两台 RealSense 均配置为 640x480。Depth 默认不保存，也不进入 LeRobot feature；只有使用 `--save-raw-images` 调试时才额外落盘 depth PNG。

> PICO 数据集写 `observation.pico`；XBOX 数据集写 `observation.xbox`。两种 controller feature 不混在同一个数据集里，建议按控制器类型使用不同 `--repo-id`。

`observation.xbox` 字段顺序：

```text
lx, ly, rx, ry, lt, rt, dpad_x, dpad_y,
btn_a, btn_b, btn_x, btn_y, btn_lb, btn_rb,
btn_back, btn_start, valid, speed_level, mode_normal, episode_done
```

## 注意事项

- 当前写入依赖官方 `lerobot` 包；没有安装时不能生成官方 v3.0 metadata。
- 建议每个新数据集使用新的 `--repo-id`，或者保证 `teleop_data_collection/datasets/<repo_id>/` 是空目录。
- `--fps` 必须和 `--hz` 一致；例如 `--hz 15 --fps 15`。LeRobot 的每一行和视频帧共用同一个时间基，不能用 `--hz 15 --fps 30`，否则 parquet timestamp 和 MP4 时间轴会压缩一半。
- 如果目标数据集目录里已经有旧版 `meta/info.json`（例如早期手写格式或 v2.1），采集脚本会拒绝继续追加。处理方式是换一个新的 `--repo-id`，或先移走旧目录。
- LeRobot v3.0 要求 episode index 连续递增，不支持手动跳号。`--episode-id` 只能指定当前“下一个” episode，例如已有 `episode_000000` 时只能指定 `episode_000001`。
- episode 数量以 `meta/info.json` 里的 `total_episodes`、`data/chunk-*/*.parquet` 和 `videos/*/chunk-*/*.mp4` 为准；根目录下的 `episode_000xxx/` 是 raw 调试日志，异常中断可能留下空目录，不能用最大 raw 序号判断视频数量。
- 如果继续追加时提示下一条 raw 目录已经存在，说明 raw 目录和 LeRobot 数据已经错位。建议换新的 `--repo-id` 重新采，或先整理旧目录后再追加。
- 真实采集链路里只有 `el_a3_sdk/demo/pico_control_jointctrl.py` 连接 CAN；采集脚本只读相机、PICO 文件和 `/tmp/robot_latest_state.json`，不直接控制机械臂。
- `/tmp/robot_latest_state.json` 超过 1 秒未更新会被视为无效状态，避免采到上一轮残留状态。
- 正常采集不要加 `--save-raw-images` 或旧参数 `--save-pngs`。这两个参数会每步、每相机写多张 PNG，几十秒 episode 很容易膨胀到几百 MB。

## 建议流程

### 1. 确认 PICO 接收器在跑
先确认 Python 环境已经安装官方 LeRobot：

```bash
python -m pip install lerobot
```

```bash
python pico3_webxr_pose_receiver.py
```

现在 `pico3_webxr_pose_receiver.py` 默认会尝试打开 `dataset_v1.yaml` 里配置的 wrist/side RealSense，并把双相机预览直接推到 Pico 头戴里的 WebXR 画面。只想接收手柄数据、不想占用相机时，加 `--no-camera-preview`。

### 2. 检查相机
```bash
python teleop_data_collection/scripts/check_realsense.py --camera-name wrist
python teleop_data_collection/scripts/check_realsense.py --camera-name side
```

### 3. 导出标定
```bash
python teleop_data_collection/scripts/export_realsense_intrinsics.py --camera-name wrist
python teleop_data_collection/scripts/export_realsense_intrinsics.py --camera-name side
python teleop_data_collection/scripts/export_realsense_extrinsic_template.py --camera-name wrist
python teleop_data_collection/scripts/export_realsense_extrinsic_template.py --camera-name side
```

### 4. 采集数据

一次完整采集 = 启动 → 做任务 → **长按 A 结束**。四种终止方式：

| 方式 | 触发 | success | 推荐度 |
|------|------|---------|--------|
| **长按 A (右手)** | 手柄 A 长按 1 秒 → 机械臂失能 | ✅ 自动标记 | ⭐ 主力 |
| `--max-duration 60` | 60 秒超时 | ❌ | 兜底安全网 |
| `--max-steps 300` | 300 步超时 | ❌ | 兜底 |
| Ctrl+C | 手动中断 | ❌ | 调试用 |

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

# 终端 1：Pico receiver（正式采集时不占用相机，预览由采集脚本推送）
python pico3_webxr_pose_receiver.py --no-camera-preview

# 终端 2：启动遥操作（带状态导出）
python el_a3_sdk/demo/pico_control_jointctrl.py --can can0 \
  --state-export /tmp/robot_latest_state.json

# 终端 3：开始采集（只读文件，不连 CAN）
python teleop_data_collection/scripts/record_sdk_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --repo-id my_edulite_dataset_two_rgb \
  --task "pick up the book and place it" \
  --hz 15 \
  --fps 15 \
  --max-duration 60 \
  --preview
```

`--preview` 会在采集时打开一个 OpenCV 窗口，横向显示 `wrist` 和 `side` 两路 RGB 当前画面。窗口只用于观察，不会额外保存 PNG，也不会让 episode 变大。采集终端按 `q`，或预览窗口里按 `q`/`Esc`，会保存当前 episode、标记 `success=true` 并结束采集；采集终端按 `f` 会保存当前 episode、标记 `success=false` 并结束采集。正常流程仍推荐做完任务后长按 A 自动结束。

键盘结束键默认是 `q`，可以改成其它单键或禁用：

```bash
--keyboard-stop-key e
--keyboard-fail-key x
--keyboard-stop-key ""
--keyboard-fail-key ""
```

如果想把双相机预览接到 Pico 头戴设备里的 WebXR 画面，正式采集时建议让 receiver 不占用相机，然后由采集脚本把同一批采集帧推回头戴：

```bash
python pico3_webxr_pose_receiver.py --no-camera-preview

python teleop_data_collection/scripts/record_sdk_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --repo-id my_edulite_dataset_two_rgb \
  --task "pick up the book and place it" \
  --max-duration 60 \
  --headset-preview
```

`--headset-preview` 会把两路相机拼成低帧率 JPEG 预览流，经 `pico3_webxr_pose_receiver.py` 推回 Pico 页面，并在头戴视野下方显示一个悬浮预览面板。默认参数为 `10fps`、`--headset-preview-scale 0.35`、`--headset-preview-quality 70`，用于避免影响手柄 pose 传输。需要同时在 PC 上看窗口时，可以同时加 `--preview`。

画面不够清晰时，提高预览分辨率和 JPEG 质量：

```bash
python teleop_data_collection/scripts/record_sdk_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --repo-id my_edulite_dataset_two_rgb \
  --task "pick up the book and place it" \
  --max-duration 60 \
  --headset-preview \
  --headset-preview-scale 0.75 \
  --headset-preview-quality 85 \
  --headset-preview-fps 8
```

如果希望头戴视频推送单独开一个进程，又不和采集抢相机：采集脚本只负责从相机帧生成最新 JPEG 文件，独立 relay 进程只读这个文件并推给 receiver。这个 relay 不打开 RealSense，不会抢占相机。

```bash
# 终端 A：Pico receiver，不占用相机
python pico3_webxr_pose_receiver.py --no-camera-preview

# 终端 B：独立头戴预览 relay，不打开相机
python teleop_data_collection/scripts/stream_headset_preview.py \
  --input /tmp/teleop_headset_preview.jpg

# 终端 C：采集脚本，唯一打开相机，并写最新预览 JPEG
python teleop_data_collection/scripts/record_sdk_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --repo-id my_edulite_dataset_two_rgb \
  --task "pick up the book and place it" \
  --max-duration 60 \
  --headset-preview-file /tmp/teleop_headset_preview.jpg \
  --headset-preview-scale 0.75 \
  --headset-preview-quality 85 \
  --headset-preview-fps 8
```

如果窗口太大，可以调小：

```bash
python teleop_data_collection/scripts/record_sdk_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --repo-id my_edulite_dataset_two_rgb \
  --task "pick up the book and place it" \
  --max-duration 60 \
  --preview \
  --preview-scale 0.35
```

如果需要临时检查相机画面或 depth 对齐，再加：

```bash
python teleop_data_collection/scripts/record_sdk_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --repo-id debug_camera_pngs \
  --task "debug camera frames" \
  --max-duration 10 \
  --save-raw-images
```

默认 `teleop_data_collection/configs/dataset_v1.yaml` 会按 serial 固定绑定：

| Camera | RealSense | Serial | LeRobot feature |
|--------|-----------|--------|-----------------|
| `wrist` | D405 | `260322277792` | `observation.images.wrist` |
| `side` | D455/D435 | `260722303162` | `observation.images.side` |

> **工作流**：启动采集 → 做任务 → **长按 A (右手) 失能** → 采集自动结束、标记 success → 长按 A 恢复使能 → 敲命令录下一个 episode。全程不碰键盘。
>
> **架构说明**：只有 `pico_control_jointctrl.py` 连接 CAN 控制机械臂。采集脚本是纯观察者，只读文件不碰 CAN。`pico_control_jointctrl.py` 在状态文件里写 `episode_done: true`（长按 A 失能时），采集脚本读到后自动终止。

### 4B. XBOX 采集数据

XBOX 采集使用 ROS2 XBOX 遥操作控制机械臂，另开一个 exporter 把 `/joint_states` 和 `/joy` 导出成采集脚本可读的 JSON。采集脚本仍然只读相机和文件，不发布控制命令。

一次完整 XBOX 采集 = 启动 XBOX 遥操作 → 启动 exporter → 启动采集 → 做任务 → **长按 Start 1 秒结束**。

```bash
# 终端 1：启动 ROS2 XBOX 实机遥操作
cd el_a3_ros
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
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
  --task "pick up the book and place it" \
  --hz 15 \
  --fps 15 \
  --max-duration 60 \
  --preview
```

`--profile auto` 会按 `/dev/input/js0` 自动识别手柄映射；Zikway HID 这类手柄会使用 Start=11，而标准 Xbox 使用 Start=7。不要把 Zikway 手柄的 exporter 固定成 `--profile xbox_default`，否则长按 Start 不会写出 `episode_done=true`。

连续采集时，在采集命令后加 `--continuous`。每次长按 `Start` 会保存当前 episode；松开 `Start` 后脚本自动进入下一条。整个连续采集会一直使用同一个 `--repo-id` 追加：

```bash
python teleop_data_collection/scripts/record_xbox_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --gamepad-file /tmp/xbox_latest_input.json \
  --repo-id my_edulite_xbox_dataset \
  --task "pick up the book and place it" \
  --hz 15 \
  --fps 15 \
  --max-duration 60 \
  --preview \
  --continuous
```

限制连续采集条数：

```bash
python teleop_data_collection/scripts/record_xbox_episode.py \
  --state-file /tmp/robot_latest_state.json \
  --gamepad-file /tmp/xbox_latest_input.json \
  --repo-id my_edulite_xbox_dataset \
  --task "pick up the book and place it" \
  --continuous \
  --max-episodes 20
```

连续采集注意事项：

- 长按 `Start` 保存一条后必须松开 `Start`，脚本才会进入下一条。
- 采集终端按 `q`，或预览窗口按 `q`/`Esc`，会保存当前 episode、标记 `success=true`，并退出整个连续采集。
- 采集终端按 `f`，会保存当前 episode、标记 `success=false`，并继续进入下一条。
- `Ctrl+C` 退出整个连续采集。
- 不要在 `--continuous` 下手动指定跳号的 `--episode-id`。
- 每条 episode 仍受 `--max-duration` 和 `--max-steps` 约束；超时结束不会自动进入下一条。

键盘结束键默认是 `q`，可以改成其它单键或禁用：

```bash
--keyboard-stop-key e
--keyboard-fail-key x
--keyboard-stop-key ""
--keyboard-fail-key ""
```

XBOX episode 结束方式：

| 方式 | 触发 | success | 推荐度 |
|------|------|---------|--------|
| **长按 Start** | Start 长按 1 秒 → exporter 写 `episode_done=true` | ✅ 自动标记 | ⭐ 主力 |
| `--max-duration 60` | 60 秒超时 | ❌ | 兜底安全网 |
| `--max-steps 300` | 300 步超时 | ❌ | 兜底 |
| Ctrl+C | 手动中断 | ❌ | 调试用 |

XBOX 控制映射仍以 `el_a3_ros/el_a3_teleop/config/xbox_teleop.yaml` 和 `xbox_teleop_node.py` 为准：

| 输入 | 功能 |
|------|------|
| 左摇杆 | 末端 X/Y 平移 |
| LT / RT | 末端 Z 下/上 |
| 右摇杆 | 姿态 Yaw/Pitch |
| LB / RB | Roll |
| D-pad 上/下 | 按住连续收紧/打开夹爪，松开后保持 |
| A | 切换速度档 |
| B | 回 Home |
| X | 回零位 |
| Y | 零力矩模式 |
| Back | 急停 |
| Start 长按 | 结束当前 episode |

### 5. 检查输出
```bash
# 查看目录结构
ls -R teleop_data_collection/datasets/my_edulite_dataset_two_rgb/

# 查看数据集信息
cat teleop_data_collection/datasets/my_edulite_dataset_two_rgb/meta/info.json | python -c "import json,sys; d=json.load(sys.stdin); print(f'episodes: {d[\"total_episodes\"]}, frames: {d[\"total_frames\"]}')"

# 查看 episode 数据 (需要 pip install pyarrow)
python -c "
import pyarrow.parquet as pq
t = pq.read_table('teleop_data_collection/datasets/my_edulite_dataset_two_rgb/data/chunk-000/file-000.parquet')
print(f'rows: {t.num_rows}, columns: {t.column_names}')
"

# 查看单集双相机摘要
python teleop_data_collection/scripts/inspect_episode.py \
  teleop_data_collection/datasets/my_edulite_dataset_two_rgb/episode_000000

# 生成 wrist/side 首帧并排预览图
python teleop_data_collection/scripts/preview_episode.py \
  teleop_data_collection/datasets/my_edulite_dataset_two_rgb/episode_000000

# 用官方 LeRobotDataset 直接加载验证
python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('local/my_edulite_dataset_two_rgb', root='teleop_data_collection/datasets/my_edulite_dataset_two_rgb', video_backend='pyav')
print(len(ds), ds[0].keys())
"
```

### 6. 迁移旧数据 (可选)
```bash
python teleop_data_collection/scripts/convert_to_lerobot.py \
  --input-root teleop_data_collection/datasets/teleop_vla \
  --repo-id edulite_a3_converted \
  --output-root teleop_data_collection/datasets
```

## 文件说明

| 文件 | 功能 |
|------|------|
| `lib/lerobot_writer.py` | LeRobot 格式写入器 (Parquet + 多路 RGB MP4) |
| `lib/episode.py` | EpisodeWriter 统一接口 |
| `lib/camera.py` | RealSense 相机接口封装 |
| `lib/robot.py` | EL-A3 SDK 机器人状态读取 |
| `lib/pico.py` | PICO 手柄位姿读取 |
| `lib/dataset.py` | Episode 目录管理 |
| `lib/utils.py` | JSONL/Parquet/视频工具函数 |
| `scripts/record_sdk_episode.py` | 采集入口脚本 |
| `scripts/check_realsense.py` | 按 `camera-name` 检查 wrist/side RealSense 连接 |
| `scripts/export_realsense_intrinsics.py` | 按 `camera-name` 导出 wrist/side 内参 |
| `scripts/export_realsense_extrinsic_template.py` | 按 `camera-name` 生成 wrist/side 外参模板 |
| `scripts/inspect_episode.py` | 查看 LeRobot episode 的双相机摘要 |
| `scripts/preview_episode.py` | 生成 wrist/side 首帧并排预览图 |
| `scripts/convert_to_lerobot.py` | 旧格式 → LeRobot 转换，支持 `camera_paths` 双相机输入 |
| `configs/` | 配置文件 (YAML) |
| `calibrations/` | wrist/side RealSense 内参/外参 |

兼容说明：旧脚本 `check_d405.py`、`export_d405_intrinsics.py`、`export_d405_extrinsic_template.py` 仍然保留，但只是转发到新的 `realsense` 通用脚本。

## 与 DROID / LeRobot 的对比

| 方面 | 旧格式 | DROID | LeRobot | 新格式 |
|------|--------|-------|---------|--------|
| 图像存储 | 单帧 PNG | HDF5 内嵌 MP4 | MP4 视频文件 | **MP4 视频文件** |
| 传感器数据 | JSONL | HDF5 dataset | Parquet | **Parquet** |
| 元数据 | meta.json | HDF5 attrs | info.json + tasks/stats/episodes parquet | **官方 LeRobot v3.0 meta** |
| 文件数量 | 极多 (每帧 3 PNG) | 1 HDF5 | 少量 Parquet + MP4 | **少量 Parquet + MP4** |
| 生态兼容 | 无 | DROID 训练栈 | LeRobot / HuggingFace | **LeRobot / HuggingFace** |
