# EL-A3 本机 ROS 仿真说明

这个仿真基于项目已有的 `el_a3_description`、`ros2_control` 配置和 `mock_components/GenericSystem`。它不会连接真实 CAN 设备，也不会驱动真实电机；它是在 RViz 中显示机械臂模型，并通过 `joint_trajectory_controller` 自动发送关节轨迹，让机械臂在虚拟环境中运动。

## 新增内容

| 路径 | 作用 |
|---|---|
| `el_a3_sim/` | 新增 ROS 2 Python 包，提供本机 mock 仿真 launch 和自动运动节点。 |
| `el_a3_sim/launch/sim.launch.py` | 一键启动仿真：mock hardware、robot_state_publisher、ros2_control、RViz、自动轨迹。 |
| `el_a3_sim/el_a3_sim/auto_motion_node.py` | 自动向 `/arm_controller/follow_joint_trajectory` 和 `/gripper_controller/follow_joint_trajectory` 发送循环动作。 |
| `scripts/run_simulation.sh` | 构建并启动仿真的便捷脚本。 |

## 环境要求

推荐在 Ubuntu 22.04 或 Windows WSL2 Ubuntu 22.04 中运行：

- ROS 2 Humble
- `ros-humble-ros2-control`
- `ros-humble-ros2-controllers`
- `ros-humble-xacro`
- `ros-humble-robot-state-publisher`
- `ros-humble-rviz2`
- `python3-colcon-common-extensions`

项目原本提供了依赖安装脚本，可在 Ubuntu/WSL2 中运行：

```bash
cd /path/to/EDULITE_A3/el_a3_ros
sudo bash scripts/install_deps.sh
sudo apt install -y python3-colcon-common-extensions
```

## 一键运行

```bash
cd /path/to/EDULITE_A3/el_a3_ros
bash scripts/run_simulation.sh
```

启动后会打开 RViz，机械臂会自动循环执行几个预设动作。

## 常用参数

只打开仿真，不自动运动：

```bash
bash scripts/run_simulation.sh auto_motion:=false
```

动作只跑一遍：

```bash
bash scripts/run_simulation.sh loop:=false
```

加快动作：

```bash
bash scripts/run_simulation.sh speed_scale:=2.0
```

不打开 RViz，仅跑控制系统和自动轨迹：

```bash
bash scripts/run_simulation.sh use_rviz:=false
```

指定腕部电机模型限制：

```bash
bash scripts/run_simulation.sh wrist_motor_type:=RS05
```

## 手动运行方式

```bash
cd /path/to/EDULITE_A3/el_a3_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select el_a3_description el_a3_sim
source install/setup.bash
ros2 launch el_a3_sim sim.launch.py
```

## 手动发一个轨迹目标

如果启动时使用 `auto_motion:=false`，可以另开一个终端手动发送轨迹：

```bash
source /opt/ros/humble/setup.bash
source /path/to/EDULITE_A3/el_a3_ros/install/setup.bash

ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [L1_joint, L2_joint, L3_joint, L4_joint, L5_joint, L6_joint],
    points: [
      {
        positions: [0.3, 1.0, -1.2, 0.2, 0.5, -0.2],
        velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 3, nanosec: 0}
      }
    ]
  }
}"
```

## 注意

这不是 Gazebo/物理仿真，没有碰撞接触、重力动力学或传感器物理反馈；它是一个轻量级 ROS/RViz/ros2_control mock 仿真，适合验证 URDF、关节限位、trajectory controller、RViz 可视化和上层运动命令。
