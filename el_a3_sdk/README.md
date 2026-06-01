# EL-A3 Python SDK

> 7-DOF 桌面机械臂纯 Python SDK，基于 Direct CAN 通信，支持多臂管理、Pinocchio 动力学、S-curve 轨迹规划。

---

## 文档

| 语言 | 文件 |
|------|------|
| 中文 | [README_zh.md](README_zh.md) |
| English | [README_en.md](README_en.md) |

---

## 快速开始

```bash
# 创建 conda 环境
conda create -n lingzuarm python=3.12

# 安装 SDK（开发模式）
cd el_a3_sdk
pip install matplotlib pillow tqdm python-dateutil
pip install -e .

# 如需 RealSense 点云可视化/选点
pip install -e ".[camera]"

# 如需运动学/动力学功能
pip install -e ".[dynamics]"

# ubuntu22：如需 Debugger 上位机（PyQt6 GUI + PyVista 3D 可视化）
sudo apt install -y libgl1-mesa-glx libgl1-mesa-dev libxrender1 libxcb-xinerama0  # Ubuntu 系统依赖
pip install -e ".[debugger]"

# ubuntu24：如需 Debugger 上位机（PyQt6 GUI + PyVista 3D 可视化）
sudo apt install -y libgl1 libglx-mesa0 libgl1-mesa-dri libgl1-mesa-dev libxrender1 libxcb-xinerama0  # Ubuntu 系统依赖
pip install "wrapt~=1.10" importlib-metadata "decorator>=5.1.0" "pexpect>4.6"
pip install -e ".[debugger]"

# 配置 CAN 接口
sudo bash scripts/setup_can.sh can0 1000000

# 运行示例
python3 demo/control_loop_demo.py
python3 demo/zero_torque_mode.py --gravity

# 启动 Debugger 上位机
el-a3-debugger
```

---

## 项目结构

| 目录/文件 | 说明 |
|-----------|------|
| `el_a3_sdk/` | Python 包核心代码 |
| `demo/` | 示例脚本 |
| `docs/` | SDK API 协议文档 |
| `resources/` | URDF、Meshes、惯性参数配置 |
| `scripts/` | CAN 配置、测试脚本 |
| `scripts/camera_test/` | RealSense 点云可视化与选点脚本 |
| `pyproject.toml` | pip 安装配置 |

---

## 依赖

- **必需**: `numpy`, `pyyaml`
- **可选**: `pin` (Pinocchio) - 运动学/动力学
- **Debugger 上位机**: `pyqt6`, `pyqtgraph`, `pyvista`, `pyvistaqt` — GUI + 3D URDF 可视化

---

## 示例

| 示例 | 说明 |
|------|------|
| `control_loop_demo.py` | 200Hz 控制循环、MoveJ、JointCtrl |
| `xbox_control.py` | Xbox 手柄笛卡尔控制 |
| `zero_torque_mode.py` | 零力矩拖动模式 |
| `dynamics_demo.py` | 重力补偿、雅可比、质量矩阵 |
| `trajectory_demo.py` | S-curve 和样条轨迹规划 |
| `cartesian_control_demo.py` | 笛卡尔空间控制 |
| `motion_control.py` | 路径点运动控制 |
| `waypoint_loop_real.py` | 路径点循环测试 |
| `read_joints.py` | 关节状态读取 |
