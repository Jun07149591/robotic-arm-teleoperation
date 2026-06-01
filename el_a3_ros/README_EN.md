# EL-A3 ROS2 Control System

> ROS2 control system for a 7-DOF desktop robot arm, built on ros2_control, CAN bus Robstride motors, Pinocchio RNEA gravity compensation, and MoveIt2 motion planning.

---

## Quick Start

```bash
# Install dependencies
cd scripts && sudo ./install_deps.sh

# Build
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Configure CAN interface
sudo bash scripts/setup_can.sh can0 1000000

# Launch with real hardware (basic control)
ros2 launch el_a3_description el_a3_control.launch.py can_interface:=can0

# Launch with real hardware + MoveIt
ros2 launch el_a3_moveit_config robot.launch.py can_interface:=can0

# Simulation mode (no hardware required)
ros2 launch el_a3_moveit_config demo.launch.py
```

---

## Packages

| Package | Type | Description |
|---------|------|-------------|
| `el_a3_hardware` | ament_cmake | ros2_control hardware interface + CAN driver + ZeroTorqueController plugin |
| `el_a3_description` | ament_cmake | URDF/xacro (7 joints), ros2_control config, launch files |
| `el_a3_moveit_config` | ament_cmake | MoveIt2 motion planning config (SRDF, kinematics, OMPL, Servo) |
| `el_a3_teleop` | ament_python | Xbox/gamepad teleoperation (Jacobian IK Cartesian control) |

---

## Directory Structure

```
el_a3_ros/
├── el_a3_description/          # URDF, Meshes, Controller YAML, Launch
│   ├── urdf/
│   │   ├── el_a3.urdf.xacro        # Main xacro (with ros2_control)
│   │   └── el_a3_ros2_control.xacro # Hardware interface parameters
│   ├── config/
│   │   ├── el_a3_controllers.yaml   # ros2_control controller config
│   │   ├── multi_arm_controllers.yaml
│   │   └── inertia_params.yaml      # Pinocchio calibrated inertia parameters
│   └── launch/
│       ├── el_a3_control.launch.py  # Basic control launch
│       └── multi_arm_control.launch.py
├── el_a3_hardware/             # C++ ros2_control hardware interface
│   ├── src/
│   │   ├── el_a3_hardware.cpp       # RsA3HardwareInterface plugin
│   │   ├── robstride_can_driver.cpp # CAN frame transceiver driver
│   │   └── zero_torque_controller.cpp # Zero-torque (teach-by-drag) controller
│   └── include/el_a3_hardware/
├── el_a3_moveit_config/        # MoveIt2 configuration
│   ├── config/
│   │   ├── el_a3.srdf               # Planning groups, collision matrix
│   │   ├── kinematics.yaml          # IK solver (pick_ik)
│   │   ├── joint_limits.yaml        # Joint limits
│   │   ├── ompl_planning.yaml       # OMPL planner
│   │   ├── moveit_controllers.yaml  # MoveIt controller bridge
│   │   └── servo_config.yaml        # MoveIt Servo (reserved)
│   └── launch/
│       ├── demo.launch.py           # Simulation MoveIt
│       └── robot.launch.py          # Real hardware MoveIt
├── el_a3_teleop/               # Gamepad teleoperation
│   ├── config/xbox_teleop.yaml
│   └── launch/real_xbox_teleop.launch.py
├── scripts/                    # Install scripts, CAN config, calibration
├── fastrtps_no_shm.xml         # DDS network configuration
└── docker-compose.yml
```

---

## Launch Files

### Basic Control (without MoveIt)

```bash
ros2 launch el_a3_description el_a3_control.launch.py \
  can_interface:=can0 \
  wrist_motor_type:=EL05 \
  use_rviz:=true
```

Starts `ros2_control_node` + `robot_state_publisher` + `joint_state_broadcaster` + `arm_controller` + `gripper_controller`.

### MoveIt Motion Planning -- Simulation

```bash
ros2 launch el_a3_moveit_config demo.launch.py
```

Uses mock hardware. Drag interactive markers in RViz to test motion planning.

### MoveIt Motion Planning -- Real Hardware

```bash
ros2 launch el_a3_moveit_config robot.launch.py \
  can_interface:=can0 \
  wrist_motor_type:=EL05
```

Additionally starts the `move_group` node and `zero_torque_controller` (inactive state).

### Xbox Gamepad Teleoperation

```bash
ros2 launch el_a3_teleop real_xbox_teleop.launch.py \
  can_interface:=can0 \
  auto_detect_controller:=true
```

Auto-detects gamepad type and starts `joy_node` + `xbox_teleop_node`. See [XBOX_CONTROL_SETUP.md](XBOX_CONTROL_SETUP.md) for details.

### Multi-Arm Control

```bash
ros2 launch el_a3_description multi_arm_control.launch.py \
  config_file:=/path/to/multi_arm_config.yaml
```

See `el_a3_description/config/multi_arm_config.yaml` for the configuration file format.

---

## Startup Test

`scripts/tests/startup_test_demo.py` provides end-to-end startup verification, automatically running system launch, controller checks, joint motion, gripper control, and more.

### Usage

```bash
# Mock simulation mode (default, no hardware required)
python3 scripts/tests/startup_test_demo.py

# Real hardware mode
python3 scripts/tests/startup_test_demo.py --mode real --can-interface can0

# Connect to a running ros2_control system (does not launch automatically)
python3 scripts/tests/startup_test_demo.py --mode connect

# Include zero-torque controller switching test
python3 scripts/tests/startup_test_demo.py --test-zero-torque
```

### Test Phases

| Phase | Description |
|-------|-------------|
| Phase 1 | System health check: controller_manager / robot_state_publisher online, robot_description readable, TF available |
| Phase 2 | Controller status: joint_state_broadcaster / arm_controller / gripper_controller all active, hardware interfaces claimed, Action Servers available |
| Phase 3 | Joint state reading: subscribe to `/joint_states` and verify all 7 joints present |
| Phase 4 | Basic motion test: home position, single-joint motion, multi-joint coordinated motion (mock mode additionally verifies position tracking error) |
| Phase 5 | Gripper test: L7 open/close control |
| Phase 6 | Controller switching (optional): arm_controller and zero_torque_controller switch verification |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--mode` | `mock` | Run mode: `mock` (simulation) / `real` (real hardware) / `connect` (attach to running system) |
| `--can-interface` | `can0` | CAN interface name (real mode) |
| `--wrist-motor-type` | `EL05` | Wrist motor type (EL05 or RS05) |
| `--wait-sec` | `25` | Seconds to wait after launch startup |
| `--test-zero-torque` | off | Enable Phase 6 zero-torque controller switching test |

### Example Output

```
============================================
 EL-A3 ROS2 Startup Test Demo
 Mode: mock | Time: 2026-03-24 16:35:26
============================================

Phase 1: System Health Check
  [PASS] controller_manager node online
  [PASS] robot_state_publisher node online
  ...

Phase 4: Basic Motion Test
  [PASS] Home position done
  [PASS] L2 single joint +0.3 done
  ...

============================================
 Result: 17 passed, 0 failed
============================================
```

---

## Zero-Torque Mode (Teach by Dragging)

`zero_torque_controller` is a custom ros2_control controller plugin implementing Kp=0 + Pinocchio RNEA gravity compensation, allowing manual dragging of the robot arm.

### Usage

```bash
# Launch (robot.launch.py automatically loads it as inactive)
ros2 launch el_a3_moveit_config robot.launch.py

# Switch to zero-torque mode (deactivate arm_controller, activate zero_torque_controller)
ros2 control switch_controllers \
  --deactivate arm_controller \
  --activate zero_torque_controller

# Restore trajectory control
ros2 control switch_controllers \
  --deactivate zero_torque_controller \
  --activate arm_controller
```

### Parameters

Configured in `el_a3_controllers.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kd` | 1.0 | Damping gain (lower = easier to drag, higher = more stable) |
| `joints` | L1-L7 | Joints participating in zero-torque mode |

---

## CAN Interface Configuration

```bash
# Using the provided script
sudo bash scripts/setup_can.sh can0 1000000

# Or manual configuration
sudo modprobe gs_usb
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up

# Verify
ip link show can0
candump can0    # View CAN frames
```

---

## DDS / FastRTPS Configuration

The project uses `fastrtps_no_shm.xml` to disable shared memory transport, suitable for Docker or multi-machine communication scenarios. Launch files automatically set the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable.

Manual setup:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fastrtps_no_shm.xml
```

---

## Building

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build a specific package:

```bash
colcon build --symlink-install --packages-select el_a3_hardware
colcon build --symlink-install --packages-select el_a3_moveit_config
```

---

## Troubleshooting

### CAN Communication Not Responding

```bash
# Check if CAN interface is UP
ip link show can0

# Check USB-CAN adapter
lsusb | grep -i can
dmesg | tail -20

# Reset CAN
sudo ip link set can0 down
sudo modprobe -r gs_usb && sleep 1 && sudo modprobe gs_usb
sudo bash scripts/setup_can.sh can0 1000000
```

### Controller Startup Timeout

```bash
# Check if controller_manager is running
ros2 node list | grep controller_manager

# View controller status
ros2 control list_controllers

# View hardware interfaces
ros2 control list_hardware_interfaces
```

### MoveIt Planning Failure

- Check if the start state is in collision: the start pose should appear green in RViz
- Increase the `allowed_start_tolerance` parameter
- Try a different planner (e.g., RRTstar, PRM)

### Motor Overheating

```bash
# Monitor temperature
ros2 topic echo /debug/motor_temperature
```

Pause operation when temperature exceeds 60 degrees C.

---

## Related Documentation

| Document | Description |
|----------|-------------|
| [ROS_INTERFACE_REFERENCE.md](ROS_INTERFACE_REFERENCE.md) | All ROS Topics / Actions / Services / TF / Launch parameters |
| [XBOX_CONTROL_SETUP.md](XBOX_CONTROL_SETUP.md) | Xbox gamepad installation and configuration |
| [XBOX_HOW_TO_USE.md](XBOX_HOW_TO_USE.md) | Xbox gamepad operation guide |
| [scripts/GRAVITY_CALIBRATION_README.md](scripts/GRAVITY_CALIBRATION_README.md) | Gravity compensation calibration |
| [hardware/electronics/README.md](hardware/electronics/README.md) | Electronics hardware documentation |
| [hardware/mechanical/README.md](hardware/mechanical/README.md) | Mechanical structure documentation |
