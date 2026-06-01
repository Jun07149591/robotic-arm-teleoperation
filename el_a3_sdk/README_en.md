# EL-A3 Robotic Arm Python SDK

> **EL-A3** is a 7-DOF (6 arm joints + L7 gripper) desktop robotic arm driven by Robstride motors over CAN bus. This SDK provides a pure Python control interface with a built-in 200Hz background control loop (EMA smoothing, velocity feedforward, Pinocchio RNEA gravity compensation, adaptive-damping zero-torque mode), S-curve trajectory planning, Cartesian control, and multi-arm management. **No ROS dependency required.**

---

## Table of Contents

- [Architecture](#architecture)
- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
  - [Debugger GUI](#3-install-debugger-gui-optional)
- [Quick Start](#quick-start)
- [Control Loop](#control-loop)
- [API Reference](#api-reference)
- [Motor Communication Protocol](#motor-communication-protocol)
- [Troubleshooting](#troubleshooting)
- [Directory Structure](#directory-structure)

---

## Architecture

```
┌──────────────────────────────────────────────────────┐
│                   User Application                     │
│   JointCtrl / MoveJ / MoveL / EndPoseCtrl / ...     │
└──────────────────────┬───────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────┐
│              ELA3Interface                             │
│         200Hz Background Control Loop                  │
│  ┌─────────┐ ┌──────────┐ ┌────────────┐            │
│  │ EMA     │ │ Velocity │ │ Pinocchio  │            │
│  │ Position│ │ Feedfwd  │ │ RNEA       │            │
│  │ Smooth  │ │ 4-MA+EMA │ │ Gravity    │            │
│  └─────────┘ └──────────┘ └────────────┘            │
│  ┌──────────────┐ ┌──────────────────────┐          │
│  │ Joint Limit  │ │ Adaptive Kd          │          │
│  │ Protection   │ │ Zero-Torque Mode     │          │
│  └──────────────┘ └──────────────────────┘          │
│  ┌──────────────┐ ┌──────────────────────┐          │
│  │ S-curve Traj │ │ Cartesian IK         │          │
│  │ Planning     │ │ (Pinocchio)          │          │
│  └──────────────┘ └──────────────────────┘          │
└──────────────────────┬───────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────┐
│        drivers.create_can_driver()                    │
│        SocketCAN / SLCAN · CAN 2.0 Extended Frames   │
└──────────────────────┬───────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────┐
│               Robstride Motors (7x)                    │
│     τ = Kp(θ_t - θ) + Kd(ω_t - ω) + τ_ff            │
│     RS00 (L1-L3) + EL05/RS05 (L4-L7)                 │
└──────────────────────────────────────────────────────┘
```

### State Machine

```
DISCONNECTED ──ConnectPort()──▶ IDLE ──EnableArm()──▶ ENABLED
                                 ▲                      │  ▲
                          DisableArm()          JointCtrl()/MoveJ()
                                 │                      ▼  │
                                 │                   RUNNING
                                 │              ZeroTorqueMode(True)
                                 │                      ▼
                                 ├───────────── ZERO_TORQUE
                                 │         ZeroTorqueMode(False) → ENABLED
                          EmergencyStop()
                                 ▼
                               ERROR
```

---

## Hardware Requirements

- **EL-A3 Robotic Arm** (7 Robstride motors)
- **CAN Adapter**: CANdle / gs_usb compatible
- **Power Supply**: 24V/48V DC
- **PC**: Ubuntu 22.04+ x86_64

### Motor Configuration

| Joint | Motor ID | Type | Torque Limit | Velocity Limit | Position Limit | Direction |
|-------|----------|------|-------------|----------------|----------------|-----------|
| L1 | 1 | RS00 | ±14 Nm | ±33 rad/s | ±2.79 rad (±160°) | -1 |
| L2 | 2 | RS00 | ±14 Nm | ±33 rad/s | 0~3.67 rad (0°~210°) | +1 |
| L3 | 3 | RS00 | ±14 Nm | ±33 rad/s | -4.01~0 rad (-230°~0°) | -1 |
| L4 | 4 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |
| L5 | 5 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | -1 |
| L6 | 6 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |
| L7 (gripper) | 7 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |

---

## Installation

### 1. Setup CAN Interface

```bash
sudo ip link set can0 up type can bitrate 1000000
# or use the script
sudo ./scripts/setup_can.sh can0 1000000
```

### 2. Install SDK

```bash
cd el_a3_sdk
pip install -e .              # basic install
pip install -e ".[dynamics]"  # with Pinocchio dynamics support
pip install -e ".[camera]"    # with RealSense point cloud visualization/picking support
```

Dependencies: `numpy`, `pyyaml`. Optional: `pinocchio` (`pip install pin`) for FK/IK/gravity compensation; `pyrealsense2`, `open3d`, and `opencv-python` for RealSense point cloud visualization/picking.

### 3. Install Debugger GUI (Optional)

The Debugger provides a PyQt6-based GUI featuring 3D URDF visualization powered by **PyVista** (VTK), interactive joint drag control, and real-time monitoring.

```bash
# Ubuntu system dependencies (OpenGL / Mesa, required for 3D rendering)
sudo apt install -y libgl1-mesa-glx libgl1-mesa-dev libxrender1 libxcb-xinerama0

# Install all Debugger Python dependencies
pip install -e ".[debugger]"
```

This installs the following packages (and their dependencies):

| Package | Purpose |
|---------|---------|
| `pyqt6` | GUI framework |
| `pyqtgraph` | Real-time data plotting |
| `pyvista` | 3D mesh rendering / URDF visualization (VTK-based) |
| `pyvistaqt` | Embeds PyVista renderer in PyQt6 windows |

Once installed, launch with:

```bash
el-a3-debugger
```

---

## Quick Start

### Basic Joint Control

```python
from el_a3_sdk import ELA3Interface

arm = ELA3Interface(can_name="can0")
arm.ConnectPort()
arm.EnableArm()

arm.JointCtrl(0.0, 0.5, -0.3, 0.0, 0.0, 0.0)

print(arm.GetArmJointMsgs())
print(arm.GetArmEndPoseMsgs())

arm.DisableArm()
arm.DisconnectPort()
```

### Using the Background Control Loop (Recommended)

```python
from el_a3_sdk import ELA3Interface
import math, time

arm = ELA3Interface(can_name="can0")
arm.ConnectPort()
arm.EnableArm()

# Start 200Hz background control loop (EMA smoothing + velocity ff + gravity comp + limit protection)
arm.start_control_loop(rate_hz=200.0)

# S-curve trajectory motion (blocking)
arm.MoveJ([0.0, math.radians(30), math.radians(-30), 0.0, 0.0, 0.0], duration=2.0)

# S-curve trajectory motion (non-blocking)
arm.MoveJ([0.0] * 6, duration=2.0, block=False)
arm.wait_for_motion()

# Real-time joint control (smoothed by control loop)
arm.JointCtrl(0.0, 0.5, -0.3, 0.0, 0.0, 0.0)

# Cartesian control
arm.EndPoseCtrl(0.3, 0.0, 0.3, 0.0, 0.0, 0.0, duration=2.0)

# Zero-torque teach mode (adaptive Kd + Pinocchio gravity compensation)
arm.ZeroTorqueModeWithGravity(True, kd=0.5)
time.sleep(10)
arm.ZeroTorqueModeWithGravity(False)

arm.stop_control_loop()
arm.DisableArm()
arm.DisconnectPort()
```

### Multi-Arm Management

```python
from el_a3_sdk import ArmManager

mgr = ArmManager()
master = mgr.register_can_arm("master", can_name="can0")
slave = mgr.register_can_arm("slave", can_name="can1")

# Or batch create from config
mgr = ArmManager.from_config("config/multi_arm_config.yaml", auto_connect=True)

mgr.disconnect_all()
```

---

## Control Loop

`start_control_loop()` launches a 200Hz background thread ported from the C++ `el_a3_hardware` control logic:

| Feature | Description |
|---------|-------------|
| **EMA Position Smoothing** | `smoothed = alpha * target + (1-alpha) * smoothed`, prevents step jumps |
| **Velocity Feedforward** | 4-sample moving average + 2nd-order EMA + acceleration limiting + smooth deadzone + tanh soft clamp |
| **Gravity Compensation** | Pinocchio RNEA feedforward torque, `gravity_feedforward_ratio` controls compensation level |
| **Joint Limit Protection** | Hard-stop clamping near joint limits |
| **Adaptive Kd Zero-Torque** | Lorentzian decay: `kd = kd_min + (kd_max - kd_min) / (1 + (v/v_ref)^2)`, EMA smoothed |
| **Trajectory Queue** | MoveJ/MoveL plans are pushed into the control loop for async execution, supports blocking/non-blocking |

### Constructor Parameters

```python
ELA3Interface(
    can_name="can0",
    host_can_id=0xFD,
    default_kp=80.0,              # Position Kp
    default_kd=4.0,               # Position Kd
    control_rate_hz=200.0,        # Control loop frequency
    smoothing_alpha=0.8,          # EMA smoothing (0=hold, 1=passthrough)
    max_velocity=3.0,             # Max joint velocity (rad/s)
    max_acceleration=15.0,        # Max joint acceleration (rad/s²)
    velocity_limit=10.0,          # Velocity feedforward limit (rad/s)
    gravity_feedforward_ratio=1.0,# Gravity compensation ratio (0~1)
    limit_margin=0.15,            # Limit deceleration zone width (rad)
    limit_stop_margin=0.02,       # Limit hard-stop zone width (rad)
    adaptive_kd_enabled=True,     # Adaptive Kd
    zero_torque_kd_min=0.02,      # Adaptive Kd lower bound
    zero_torque_kd_max=1.0,       # Adaptive Kd upper bound
    kd_velocity_ref=1.0,          # Adaptive Kd velocity reference (rad/s)
    urdf_path=None,               # URDF path (Pinocchio)
    inertia_config_path=None,     # Calibrated inertia parameters
)
```

---

## API Reference

### Connection

| Method | Description |
|--------|-------------|
| `ConnectPort()` | Open CAN socket, start I/O threads |
| `DisconnectPort()` | Stop control loop and threads, close socket |

### Control Loop

| Method | Description |
|--------|-------------|
| `start_control_loop(rate_hz=200.0)` | Start background control loop |
| `stop_control_loop()` | Stop control loop |

### Motor Control

| Method | Description |
|--------|-------------|
| `EnableArm(motor_num=7)` | Enable motors |
| `DisableArm(motor_num=7)` | Disable motors |
| `EmergencyStop()` | Emergency stop |
| `SetZeroPosition(motor_num=7)` | Set zero position |

### Motion Control

| Method | Description |
|--------|-------------|
| `JointCtrl(j1..j6, kp, kd, torque_ff)` | Joint angle control |
| `JointCtrlList(positions)` | List-form joint control |
| `MoveJ(positions, duration, block=True)` | S-curve joint motion |
| `MoveL(target_pose, duration, block=True)` | Cartesian linear motion |
| `EndPoseCtrl(x, y, z, rx, ry, rz, duration, block=True)` | End-effector pose control |
| `CartesianVelocityCtrl(vx, vy, vz, wx, wy, wz)` | Cartesian velocity control |
| `GripperCtrl(gripper_angle)` | Gripper control |
| `is_moving()` | Check if trajectory is executing |
| `wait_for_motion(timeout)` | Wait for trajectory completion |
| `cancel_motion()` | Cancel current trajectory |

### Zero-Torque Mode

| Method | Description |
|--------|-------------|
| `ZeroTorqueMode(enable, kd=1.0)` | Zero-torque mode (Kp=0) |
| `ZeroTorqueModeWithGravity(enable, kd=0.5)` | Gravity-compensated zero-torque (adaptive Kd) |

### Status Feedback

| Method | Returns | Description |
|--------|---------|-------------|
| `GetArmJointMsgs()` | `ArmJointStates` | 7 joint angles (rad) |
| `GetArmJointVelocities()` | `ArmJointStates` | 7 joint velocities (rad/s) |
| `GetArmJointEfforts()` | `ArmJointStates` | 7 joint torques (Nm) |
| `GetArmEndPoseMsgs()` | `ArmEndPose` | End-effector pose (Pinocchio FK) |
| `GetArmStatus()` | `ArmStatus` | Composite status |

### Dynamics (Pinocchio)

| Method | Description |
|--------|-------------|
| `ComputeGravityTorques(positions)` | RNEA gravity compensation torques |
| `GetJacobian(positions)` | End-effector Jacobian (6xN) |
| `GetMassMatrix(positions)` | Mass matrix M(q) |
| `InverseDynamics(q, v, a)` | RNEA inverse dynamics |
| `ForwardDynamics(q, v, tau)` | ABA forward dynamics |

### Parameter Settings

| Method | Description |
|--------|-------------|
| `SetPositionPD(kp, kd)` | Set PD gains |
| `SetSmoothingAlpha(alpha)` | Set EMA smoothing coefficient |
| `SetGravityFeedforwardRatio(ratio)` | Set gravity compensation ratio |
| `SetJointLimitEnabled(enabled)` | Toggle joint limit protection |

---

## Motor Communication Protocol

Motion control mode (MIT-like PD Control):

```
τ = Kp × (θ_target - θ_actual) + Kd × (ω_target - ω_actual) + τ_ff
```

29-bit extended frame ID:

```
Bit 28~24: CommType (5 bits)
Bit 23~8:  DataArea2 (16 bits)
Bit 7~0:   Target Address (8 bits)
```

| CommType | Value | Function |
|----------|-------|----------|
| MOTION_CONTROL | 1 | Motion command (position/velocity/Kp/Kd/torque) |
| FEEDBACK | 2 | Motor feedback (position/velocity/torque/temperature) |
| ENABLE | 3 | Enable motor |
| DISABLE | 4 | Stop motor |
| SET_ZERO | 6 | Set zero position |
| WRITE_PARAM | 18 | Parameter write |

uint16 linear mapping:

```
Encode: uint16 = (value - min) × 65535 / (max - min)
Decode: value  = uint16 × (max - min) / 65535 + min
```

---

## Troubleshooting

### CAN Interface

```bash
lsusb | grep -i can
sudo modprobe can && sudo modprobe can_raw && sudo modprobe gs_usb
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
candump can0
```

### Motor Not Responding

1. Check CAN wiring and termination resistors
2. Confirm motor ID configuration (1-7)
3. Check power supply
4. Verify host CAN ID (default 253/0xFD)

### End-Effector Jitter

1. Adjust `default_kp` / `default_kd`
2. Increase `smoothing_alpha`
3. Reduce `max_velocity`

---

## Directory Structure

```
el_a3_sdk/
├── __init__.py          # Package entry
├── interface.py         # ELA3Interface — main interface + control loop
├── arm_manager.py       # ArmManager — multi-arm manager
├── drivers/             # SocketCAN / SLCAN driver package
├── protocol.py          # Protocol enums, motor params, joint config
├── data_types.py        # Data structures (SI units)
├── kinematics.py        # Pinocchio FK/IK/Jacobian/Gravity
├── realsense/           # RealSense RGB-D capture, point cloud, and picking tools
├── motion/              # S-curve, cubic spline, and Cartesian trajectory helpers
├── utils.py             # Utility functions
├── pyproject.toml       # pip install configuration
└── demo/
    ├── control_loop_demo.py   # Control loop demo
    ├── motion_control.py      # Joint motion demo
    ├── zero_torque_mode.py    # Zero-torque teach demo
    ├── cartesian_control_demo.py  # Cartesian control demo
    ├── dynamics_demo.py       # Dynamics demo
    ├── trajectory_demo.py     # Trajectory planning demo
    ├── waypoint_loop_real.py  # Waypoint loop demo
    └── read_joints.py         # Joint reading demo
```

---

## License

Apache-2.0
