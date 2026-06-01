# EL-A3 ROS2 接口参考

> 本文档列出 EL-A3 ROS2 控制系统运行时所有的 Topics、Actions、TF、Launch 文件参数及控制器切换操作。

---

## 1. ROS Topics

### 1.1 标准 ros2_control Topics

| Topic | 类型 | 发布者 | 说明 |
|-------|------|--------|------|
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` | L1-L7 关节位置、速度、力矩 |
| `/robot_description` | `std_msgs/msg/String` | `robot_state_publisher` | URDF 描述（latched） |
| `/tf` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` | 动态 TF 变换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `static_transform_publisher` | world→base_link 静态变换 |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | `controller_manager` | 所有关节状态接口值 |

### 1.2 el_a3_hardware 调试 Topics

硬件接口层发布的调试数据，全部使用 `sensor_msgs/msg/JointState` 类型。

| Topic | 说明 | 常用场景 |
|-------|------|----------|
| `/debug/hw_command` | 硬件层接收到的原始位置指令 | 对比 MoveIt/Teleop 输出 |
| `/debug/smoothed_command` | 平滑滤波后的位置指令 | 检查 EMA 滤波效果 |
| `/debug/gravity_torque` | Pinocchio RNEA 计算的重力补偿力矩 | 验证重力补偿精度 |
| `/debug/velocity_feedforward` | 速度前馈量 | 调试速度控制 |
| `/debug/motor_temperature` | 各电机绕组温度 (°C) | 过热监控 |
| `/debug/torque_feedback` | 各电机实际力矩反馈 | 力矩分析 |
| `/debug/adaptive_kd` | 自适应 Kd 阻尼值 | 调试零力矩模式 |

### 1.3 ZeroTorqueController Topics

| Topic | 类型 | 说明 |
|-------|------|------|
| `/zero_torque_controller/gravity_torque` | `sensor_msgs/msg/JointState` | 零力矩模式下的重力补偿力矩（每 10 个控制周期发布一次） |

### 1.4 MoveIt Topics

| Topic | 类型 | 说明 |
|-------|------|------|
| `/planning_scene` | `moveit_msgs/msg/PlanningScene` | 当前规划场景 |
| `/planning_scene_world` | `moveit_msgs/msg/PlanningSceneWorld` | 规划场景世界信息 |
| `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | RViz 中显示规划路径 |
| `/monitored_planning_scene` | `moveit_msgs/msg/PlanningScene` | 监控的规划场景 |

### 1.5 Teleop Topics

| Topic | 类型 | 发布/订阅 | 说明 |
|-------|------|-----------|------|
| `/joy` | `sensor_msgs/msg/Joy` | joy_node 发布 | Xbox/手柄原始输入 |

---

## 2. ROS Actions

| Action | 类型 | 服务端 | 说明 |
|--------|------|--------|------|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `arm_controller` | L1-L6 关节轨迹执行 |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `gripper_controller` | L7 夹爪轨迹执行 |
| `/move_group` | `moveit_msgs/action/MoveGroup` | `move_group` | MoveIt 运动规划（仅 MoveIt launch 下可用） |

---

## 3. ROS Services

| Service | 类型 | 节点 | 说明 |
|---------|------|------|------|
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | `controller_manager` | 列出所有控制器及状态 |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | `controller_manager` | 激活/停用控制器 |
| `/controller_manager/load_controller` | `controller_manager_msgs/srv/LoadController` | `controller_manager` | 加载控制器插件 |

---

## 4. TF 树

```
world (固定帧)
  └── base_link (static_transform_publisher)
        └── l1_urdf_urdf_asm (L1_joint, revolute)
              └── l1_link_urdf_asm
                    └── l2_l3_urdf_asm (L2_joint, revolute)
                          └── l3_lnik_urdf_asm (L3_joint, revolute)
                                └── l4_l5_urdf_asm (L4_joint, revolute)
                                      └── part_9 (L5_joint, revolute)
                                            └── l5_l6_urdf_asm (L6_joint, revolute)
                                                  ├── end_effector (end_effector_joint, fixed)
                                                  └── gripper_link (L7_joint, revolute)
```

---

## 5. 控制器切换操作

`arm_controller` 和 `zero_torque_controller` 共享 L1-L6 关节的 command_interface，它们是**互斥**的，不能同时激活。

### 5.1 切换到零力矩模式（拖动示教）

```bash
ros2 control switch_controllers \
  --deactivate arm_controller \
  --activate zero_torque_controller
```

### 5.2 从零力矩恢复到轨迹控制

```bash
ros2 control switch_controllers \
  --deactivate zero_torque_controller \
  --activate arm_controller
```

### 5.3 查看当前控制器状态

```bash
ros2 control list_controllers
```

---

## 6. Launch 文件参数一览

### 6.1 `el_a3_description / el_a3_control.launch.py`

基础控制 launch（不含 MoveIt）。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_mock_hardware` | `false` | 使用虚拟硬件（无需实际 CAN） |
| `can_interface` | `can0` | CAN 接口名 |
| `host_can_id` | `253` | 主机 CAN ID (0xFD) |
| `wrist_motor_type` | `EL05` | 腕部电机型号：EL05 或 RS05 |
| `use_rviz` | `true` | 启动 RViz |
| `use_rt_sched` | `false` | 使用 SCHED_FIFO 实时调度（需 RT 权限） |

### 6.2 `el_a3_moveit_config / robot.launch.py`

真实硬件 + MoveIt 运动规划。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | `can0` | CAN 接口名 |
| `host_can_id` | `253` | 主机 CAN ID |
| `wrist_motor_type` | `EL05` | 腕部电机型号 |
| `use_rviz` | `true` | 启动 RViz + MoveIt 插件 |

### 6.3 `el_a3_moveit_config / demo.launch.py`

仿真模式（mock hardware + MoveIt），无需硬件。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_rviz` | `true` | 启动 RViz + MoveIt 插件 |

### 6.4 `el_a3_teleop / real_xbox_teleop.launch.py`

Xbox/手柄遥操作（包含 el_a3_control launch）。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `auto_detect_controller` | `true` | 自动检测手柄类型 |
| `joy_dev_override` | `""` | 手动指定手柄设备路径（如 `/dev/input/js1`） |
| `controller_profile_override` | `""` | 强制指定手柄配置文件 |
| `can_interface` | `can0` | CAN 接口名 |
| `use_rviz` | `false` | 启动 RViz |
| `use_mock_hardware` | `false` | 使用虚拟硬件 |

### 6.5 `el_a3_description / multi_arm_control.launch.py`

多臂控制。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `config_file` | `""` | 多臂配置 YAML 文件路径（空则使用默认配置） |
| `use_rviz` | `true` | 启动 RViz |

---

## 7. 常用调试命令

```bash
# 查看所有 topic
ros2 topic list

# 监听关节状态
ros2 topic echo /joint_states

# 监听电机温度
ros2 topic echo /debug/motor_temperature

# 监听重力补偿力矩
ros2 topic echo /debug/gravity_torque

# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看控制器列表
ros2 control list_controllers

# 查看硬件接口
ros2 control list_hardware_interfaces
```
