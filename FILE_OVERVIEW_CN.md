# EDULITE_A3 文件说明总览

本仓库已完整下载到 `D:\codex\EA3\EDULITE_A3`。当前检出的 Git 提交是 `ea7231f fix gravity_torques`，并且 Git LFS 大文件已经拉取为真实文件。

## 项目整体

这是 EL-A3 7 自由度桌面机械臂项目，主要分三块：

- `el_a3_sdk/`：纯 Python SDK、演示程序、测试、上位机 MotorStudio。
- `el_a3_ros/`：ROS 2 Humble 控制系统，包含 robot description、ros2_control 硬件接口、MoveIt 配置、手柄遥操作和测试脚本。
- `hardware/`：机械、3D 打印、PCB、线束、装配 SOP 等硬件制造资料。

## 根目录

| 文件 | 作用 |
|---|---|
| `.clangd` | clangd/C++ IDE 配置，用于补充 ROS 2 C++ 包的 include、编译参数提示。 |
| `.gitattributes` | Git LFS 规则，指定 `.stl`、`.step`、`.3mf`、`.pdf`、Gerber 等大文件由 LFS 管理。 |
| `.gitignore` | 根仓库忽略规则，排除构建输出、缓存、临时文件等。 |
| `LICENSE` | Apache-2.0 开源许可证文本。 |
| `README.md` | 仓库总说明，介绍 EL-A3 项目、硬件要求、软件环境、SDK/ROS/hardware 三个子项目。 |
| `iface_diff.txt` | 一份接口差异补丁/对比文本，记录 SDK 接口新增 SLCAN 后端、重力关节缩放等改动。 |

## el_a3_ros 根目录

| 文件 | 作用 |
|---|---|
| `el_a3_ros/.dockerignore` | Docker 构建上下文忽略规则，避免把 build/log/install 等文件打进镜像。 |
| `el_a3_ros/.gitignore` | ROS 工作区内部忽略规则。 |
| `el_a3_ros/ACTIVATE_XBOX_CONTROLLER.md` | Xbox 手柄激活、识别、权限与连接步骤说明。 |
| `el_a3_ros/BLUETOOTH_XBOX_SETUP.md` | 蓝牙连接 Xbox 手柄的系统配置说明。 |
| `el_a3_ros/Dockerfile` | ROS 2 环境 Docker 镜像定义，安装依赖并构建运行环境。 |
| `el_a3_ros/docker-compose.yml` | Docker Compose 启动配置，用于容器化运行 ROS 环境。 |
| `el_a3_ros/docker/entrypoint.sh` | Docker 容器入口脚本，负责 source ROS 环境并执行命令。 |
| `el_a3_ros/fastrtps_no_shm.xml` | FastDDS/FastRTPS 配置，禁用共享内存传输，常用于 Docker 或跨主机场景。 |
| `el_a3_ros/README.md` | ROS 2 控制系统中文说明，包含构建、launch、MoveIt、测试、零力矩模式等。 |
| `el_a3_ros/README_EN.md` | ROS 2 控制系统英文说明。 |
| `el_a3_ros/ROS_INTERFACE_REFERENCE.md` | ROS 接口参考，列出 topic、action、service、TF、launch 参数等。 |
| `el_a3_ros/XBOX_CONTROL_SETUP.md` | Xbox 遥操作安装与配置指南。 |
| `el_a3_ros/XBOX_HOW_TO_USE.md` | Xbox 手柄操作映射与使用说明。 |
| `el_a3_ros/XBOX_QUICK_FIX.md` | Xbox 手柄常见问题快速修复文档。 |

## el_a3_ros/EDULITE-A3

这个目录像是由 CAD/URDF 导出工具生成的原始 ROS 描述包，和后面的 `el_a3_description` 有重复资源。

| 文件 | 作用 |
|---|---|
| `el_a3_ros/EDULITE-A3/CMakeLists.txt` | 原始机器人描述包的 ament_cmake 构建与安装配置。 |
| `el_a3_ros/EDULITE-A3/package.xml` | 原始机器人描述包的 ROS 2 包元数据。 |
| `el_a3_ros/EDULITE-A3/README.md` | 原始描述包说明。 |
| `el_a3_ros/EDULITE-A3/config.json` | 原始导出工具/描述包配置文件。 |
| `el_a3_ros/EDULITE-A3/EDULITE-A3.urdf` | EL-A3 机械臂 URDF 模型，定义 link、joint、mesh、惯量和视觉几何。 |
| `el_a3_ros/EDULITE-A3/launch/display.launch.py` | 启动 robot_state_publisher/RViz 展示 URDF 的 launch 文件。 |
| `el_a3_ros/EDULITE-A3/meshes/el05_l6.part` | CAD/建模源部件文件，表示 EL05/L6 相关部件。 |
| `el_a3_ros/EDULITE-A3/meshes/el05_l6.stl` | EL05/L6 相关 3D 网格，URDF 可视化/碰撞使用。 |
| `el_a3_ros/EDULITE-A3/meshes/jaw.part` | 夹爪 CAD/建模源部件文件。 |
| `el_a3_ros/EDULITE-A3/meshes/jaw.stl` | 夹爪 3D 网格。 |
| `el_a3_ros/EDULITE-A3/meshes/l1.part` | 第 1 连杆 CAD/建模源部件文件。 |
| `el_a3_ros/EDULITE-A3/meshes/l1.stl` | 第 1 连杆 3D 网格。 |
| `el_a3_ros/EDULITE-A3/meshes/l2.part` | 第 2 连杆 CAD/建模源部件文件。 |
| `el_a3_ros/EDULITE-A3/meshes/l2.stl` | 第 2 连杆 3D 网格。 |
| `el_a3_ros/EDULITE-A3/meshes/l3.part` | 第 3 连杆 CAD/建模源部件文件。 |
| `el_a3_ros/EDULITE-A3/meshes/l3.stl` | 第 3 连杆 3D 网格。 |
| `el_a3_ros/EDULITE-A3/meshes/l4.part` | 第 4 连杆 CAD/建模源部件文件。 |
| `el_a3_ros/EDULITE-A3/meshes/l4.stl` | 第 4 连杆 3D 网格。 |
| `el_a3_ros/EDULITE-A3/meshes/l5.part` | 第 5 连杆 CAD/建模源部件文件。 |
| `el_a3_ros/EDULITE-A3/meshes/l5.stl` | 第 5 连杆 3D 网格。 |
| `el_a3_ros/EDULITE-A3/meshes/l6.part` | 第 6 连杆 CAD/建模源部件文件。 |
| `el_a3_ros/EDULITE-A3/meshes/l6.stl` | 第 6 连杆 3D 网格。 |

## el_a3_ros/el_a3_description

这是主要 ROS 2 机器人描述包，提供 URDF/xacro、mesh、控制器配置和 launch。

| 文件 | 作用 |
|---|---|
| `el_a3_ros/el_a3_description/CMakeLists.txt` | 安装 `urdf`、`meshes`、`launch`、`config` 目录的 ament_cmake 配置。 |
| `el_a3_ros/el_a3_description/package.xml` | ROS 2 description 包元数据。 |
| `el_a3_ros/el_a3_description/urdf/el_a3.urdf` | 纯 URDF 版本机器人模型。 |
| `el_a3_ros/el_a3_description/urdf/el_a3.urdf.xacro` | xacro 主模型，可参数化生成机器人描述并包含 ros2_control 配置。 |
| `el_a3_ros/el_a3_description/urdf/el_a3_ros2_control.xacro` | ros2_control hardware plugin、joint command/state interfaces、CAN 参数等 xacro 片段。 |
| `el_a3_ros/el_a3_description/config/arm2_only_config.yaml` | 多机械臂场景中仅启用 arm2 的配置样例。 |
| `el_a3_ros/el_a3_description/config/el_a3_controllers.yaml` | ros2_control 控制器配置，包括 joint_state_broadcaster、arm trajectory、gripper、zero_torque 等。 |
| `el_a3_ros/el_a3_description/config/el_a3_view.rviz` | RViz 显示配置。 |
| `el_a3_ros/el_a3_description/config/inertia_params.yaml` | 标定后的惯性/动力学参数，供 Pinocchio 重力补偿使用。 |
| `el_a3_ros/el_a3_description/config/inertia_params_arm2.yaml` | 第二台机械臂的惯性参数配置。 |
| `el_a3_ros/el_a3_description/config/master_slave_config.yaml` | 主从遥操作/双臂跟随配置。 |
| `el_a3_ros/el_a3_description/config/multi_arm_config.yaml` | 多机械臂 CAN 接口、命名空间、关节前缀等配置。 |
| `el_a3_ros/el_a3_description/config/multi_arm_controllers.yaml` | 多机械臂 ros2_control 控制器配置。 |
| `el_a3_ros/el_a3_description/launch/comm_test.launch.py` | 通信测试 launch，用于快速启动并验证硬件/控制链路。 |
| `el_a3_ros/el_a3_description/launch/el_a3_control.launch.py` | 基础控制 launch，启动 robot_state_publisher、ros2_control_node、控制器和可选 RViz。 |
| `el_a3_ros/el_a3_description/launch/multi_arm_control.launch.py` | 多机械臂控制 launch，从 YAML 配置生成多套控制节点。 |
| `el_a3_ros/el_a3_description/meshes/el05_l6.stl` | EL05/L6 相关 3D 网格，ROS description 正式使用。 |
| `el_a3_ros/el_a3_description/meshes/jaw.stl` | 夹爪 3D 网格。 |
| `el_a3_ros/el_a3_description/meshes/l1.stl` | 第 1 连杆 3D 网格。 |
| `el_a3_ros/el_a3_description/meshes/l2.stl` | 第 2 连杆 3D 网格。 |
| `el_a3_ros/el_a3_description/meshes/l3.stl` | 第 3 连杆 3D 网格。 |
| `el_a3_ros/el_a3_description/meshes/l4.stl` | 第 4 连杆 3D 网格。 |
| `el_a3_ros/el_a3_description/meshes/l5.stl` | 第 5 连杆 3D 网格。 |
| `el_a3_ros/el_a3_description/meshes/l6.stl` | 第 6 连杆 3D 网格。 |

## el_a3_ros/el_a3_hardware

这是 C++ ros2_control 硬件接口包，负责和 Robstride 电机通过 SocketCAN 通信。

| 文件 | 作用 |
|---|---|
| `el_a3_ros/el_a3_hardware/.clangd` | 该 C++ 包单独的 clangd 配置。 |
| `el_a3_ros/el_a3_hardware/CMakeLists.txt` | 构建 `el_a3_hardware` 共享库，链接 ROS 2、hardware_interface、controller_interface、Pinocchio。 |
| `el_a3_ros/el_a3_hardware/package.xml` | ROS 2 C++ 硬件接口包元数据与依赖声明。 |
| `el_a3_ros/el_a3_hardware/HARDWARE_INTERFACE.md` | 硬件接口设计/使用说明。 |
| `el_a3_ros/el_a3_hardware/el_a3_hardware_plugin.xml` | pluginlib 硬件接口插件描述，把 `RsA3HardwareInterface` 暴露给 ros2_control。 |
| `el_a3_ros/el_a3_hardware/el_a3_controller_plugin.xml` | pluginlib 控制器插件描述，把 `ZeroTorqueController` 暴露给 controller_manager。 |
| `el_a3_ros/el_a3_hardware/include/el_a3_hardware/el_a3_hardware.hpp` | `RsA3HardwareInterface` 头文件，定义关节配置、状态、命令、限位、重力补偿成员。 |
| `el_a3_ros/el_a3_hardware/include/el_a3_hardware/robstride_can_driver.hpp` | Robstride CAN 驱动头文件，定义电机类型、参数、反馈和收发接口。 |
| `el_a3_ros/el_a3_hardware/include/el_a3_hardware/s_curve_generator.hpp` | S 曲线轨迹生成器头文件，定义 jerk/速度/加速度受限轨迹结构。 |
| `el_a3_ros/el_a3_hardware/include/el_a3_hardware/zero_torque_controller.hpp` | 零力矩/拖动示教控制器头文件。 |
| `el_a3_ros/el_a3_hardware/src/el_a3_hardware.cpp` | ros2_control 硬件接口实现，处理 lifecycle、read/write、命令模式切换、限位保护、Pinocchio 重力补偿。 |
| `el_a3_ros/el_a3_hardware/src/robstride_can_driver.cpp` | SocketCAN 收发、Robstride 协议编码解码、电机使能/失能/参数写入/反馈解析实现。 |
| `el_a3_ros/el_a3_hardware/src/s_curve_generator.cpp` | S 曲线轨迹生成算法实现。 |
| `el_a3_ros/el_a3_hardware/src/zero_torque_controller.cpp` | 零力矩控制器实现，使用 Kp=0、阻尼和重力补偿支持拖动示教。 |

## el_a3_ros/el_a3_moveit_config

这是 MoveIt 2 配置包，负责运动规划、IK、控制器桥接和 RViz 规划界面。

| 文件 | 作用 |
|---|---|
| `el_a3_ros/el_a3_moveit_config/CMakeLists.txt` | 安装 MoveIt 配置与 launch 文件。 |
| `el_a3_ros/el_a3_moveit_config/package.xml` | MoveIt 配置包元数据。 |
| `el_a3_ros/el_a3_moveit_config/config/el_a3.srdf` | MoveIt 语义描述，定义规划组、末端、禁用碰撞对等。 |
| `el_a3_ros/el_a3_moveit_config/config/joint_limits.yaml` | MoveIt 关节速度/加速度/位置限制。 |
| `el_a3_ros/el_a3_moveit_config/config/kinematics.yaml` | IK 求解器配置，项目中用于 pick_ik 等求解器。 |
| `el_a3_ros/el_a3_moveit_config/config/moveit.rviz` | MoveIt RViz 插件界面布局。 |
| `el_a3_ros/el_a3_moveit_config/config/moveit_controllers.yaml` | MoveIt 到 ros2_control trajectory controller 的映射配置。 |
| `el_a3_ros/el_a3_moveit_config/config/ompl_planning.yaml` | OMPL 规划器参数配置。 |
| `el_a3_ros/el_a3_moveit_config/config/servo_config.yaml` | MoveIt Servo 配置，预留/支持实时笛卡尔伺服控制。 |
| `el_a3_ros/el_a3_moveit_config/launch/demo.launch.py` | 无硬件 mock 模式 MoveIt 演示 launch。 |
| `el_a3_ros/el_a3_moveit_config/launch/robot.launch.py` | 真实硬件 MoveIt launch，启动 move_group、控制器、RViz 等。 |

## el_a3_ros/el_a3_teleop

这是 ROS 2 Python 手柄遥操作包。

| 文件 | 作用 |
|---|---|
| `el_a3_ros/el_a3_teleop/package.xml` | ROS 2 Python 包元数据。 |
| `el_a3_ros/el_a3_teleop/setup.py` | Python 包安装配置，注册可执行入口。 |
| `el_a3_ros/el_a3_teleop/setup.cfg` | setuptools/ament_python 安装路径配置。 |
| `el_a3_ros/el_a3_teleop/resource/el_a3_teleop` | ament resource marker，标记这是一个 ROS 2 Python 包。 |
| `el_a3_ros/el_a3_teleop/config/xbox_teleop.yaml` | 手柄遥操作参数，如轴映射、速度比例、死区、控制模式等。 |
| `el_a3_ros/el_a3_teleop/el_a3_teleop/__init__.py` | Python 包初始化文件。 |
| `el_a3_ros/el_a3_teleop/el_a3_teleop/xbox_teleop_node.py` | Xbox 遥操作 ROS 节点，读取 `sensor_msgs/Joy` 并发送机械臂/夹爪控制命令。 |
| `el_a3_ros/el_a3_teleop/launch/real_xbox_teleop.launch.py` | 真实硬件手柄控制 launch，组合 joy_node、teleop 节点、控制系统。 |

## el_a3_ros/scripts

| 文件 | 作用 |
|---|---|
| `el_a3_ros/scripts/GRAVITY_CALIBRATION_README.md` | 重力补偿/动力学标定流程说明。 |
| `el_a3_ros/scripts/batch_kp_test.sh` | 批量测试不同 Kp 参数的脚本。 |
| `el_a3_ros/scripts/dynamics_calibration.py` | ROS 节点形式的动力学/惯性标定采集与拟合脚本。 |
| `el_a3_ros/scripts/foxglove_bridge.service` | systemd service 文件，用于启动 Foxglove Bridge。 |
| `el_a3_ros/scripts/gravity_calibration.py` | 交互式重力标定 ROS 节点，采集不同姿态下的数据。 |
| `el_a3_ros/scripts/gravity_calibration_20260122_192858.json` | 一份历史重力标定结果数据。 |
| `el_a3_ros/scripts/gravity_calibration_20260122_193651.json` | 另一份历史重力标定结果数据。 |
| `el_a3_ros/scripts/gravity_calibration_analyzer.py` | 分析重力标定 JSON，输出报告和图表。 |
| `el_a3_ros/scripts/inertia_calibration.py` | 惯性参数标定脚本。 |
| `el_a3_ros/scripts/install_deps.sh` | 安装 ROS 2 项目依赖，如 ros2_control、MoveIt、joy 等。 |
| `el_a3_ros/scripts/install_joycon_driver.sh` | 安装 Joy-Con 手柄相关驱动/依赖。 |
| `el_a3_ros/scripts/install_xpadneo.sh` | 安装 xpadneo Xbox 蓝牙驱动。 |
| `el_a3_ros/scripts/joycon_visualizer.py` | Joy-Con IMU 读取与 3D 姿态可视化脚本，内含 Madgwick 滤波。 |
| `el_a3_ros/scripts/move_to_zero.py` | ROS 节点脚本，让机械臂移动到零位。 |
| `el_a3_ros/scripts/pinocchio_gravity_calibration.py` | 基于 Pinocchio 模型的重力补偿参数标定脚本。 |
| `el_a3_ros/scripts/setup_bluetooth_xbox.sh` | 蓝牙 Xbox 手柄系统配置脚本。 |
| `el_a3_ros/scripts/setup_can.sh` | 配置单个 SocketCAN 接口，如 `can0`、1 Mbps。 |
| `el_a3_ros/scripts/setup_multi_can.sh` | 配置多个 CAN 接口的脚本。 |
| `el_a3_ros/scripts/simple_motion_test.py` | ROS 方式的简单关节运动测试节点。 |
| `el_a3_ros/scripts/start_real_xbox_control.sh` | 启动真实硬件 Xbox 遥操作的便捷脚本。 |
| `el_a3_ros/scripts/start_teleop.sh` | 启动遥操作功能的通用脚本。 |
| `el_a3_ros/scripts/start_web_ui.sh` | 启动 Web UI/桥接相关服务的脚本。 |
| `el_a3_ros/scripts/start_xbox_control.sh` | 启动 Xbox 控制的便捷脚本。 |
| `el_a3_ros/scripts/teleop_master_slave.py` | 直接 CAN 层主从/跟随遥操作脚本，包含 Robstride CAN 协议处理。 |
| `el_a3_ros/scripts/test_kp_values.py` | 测试不同 Kp 参数对运动表现影响的 ROS 节点。 |
| `el_a3_ros/scripts/test_master_slave.py` | 主从控制测试节点。 |
| `el_a3_ros/scripts/test_moveit_waypoints.py` | MoveIt 路径点规划/执行测试。 |
| `el_a3_ros/scripts/test_single_move.py` | 单次关节运动测试。 |
| `el_a3_ros/scripts/test_teleop_can0_can1.py` | can0/can1 双 CAN 遥操作测试。 |
| `el_a3_ros/scripts/test_teleop_can1_master.py` | can1 作为主端的遥操作测试。 |
| `el_a3_ros/scripts/test_xbox_control.sh` | Xbox 控制功能测试脚本。 |
| `el_a3_ros/scripts/wait_and_start_xbox.sh` | 等待设备/系统就绪后启动 Xbox 控制。 |
| `el_a3_ros/scripts/zero_test.py` | 零位/归零相关测试节点。 |
| `el_a3_ros/scripts/calibration_results/calibration_data.jsonl` | 标定过程采集的 JSON Lines 原始数据。 |
| `el_a3_ros/scripts/calibration_results/inertia_params.yaml` | 标定生成的惯性参数 YAML。 |

## el_a3_ros/scripts/tests

| 文件 | 作用 |
|---|---|
| `el_a3_ros/scripts/tests/run_sim_tests.sh` | 批量运行仿真/mock 测试的 shell 脚本。 |
| `el_a3_ros/scripts/tests/startup_test_demo.py` | 端到端启动测试，检查节点、控制器、joint_states、运动、夹爪、可选零力矩切换。 |
| `el_a3_ros/scripts/tests/test_build.sh` | ROS 工作区构建测试脚本。 |
| `el_a3_ros/scripts/tests/test_hw_gripper.py` | 真实硬件夹爪控制测试。 |
| `el_a3_ros/scripts/tests/test_hw_motion.py` | 真实硬件关节轨迹运动测试。 |
| `el_a3_ros/scripts/tests/test_hw_zero_torque.py` | 真实硬件零力矩控制器切换与状态测试。 |
| `el_a3_ros/scripts/tests/test_mock_launch.py` | mock 模式 launch、控制器、robot_description、trajectory 的测试。 |
| `el_a3_ros/scripts/tests/test_xacro_gen.sh` | 验证 xacro 能正确生成 URDF 的脚本。 |

## el_a3_sdk 根目录

| 文件 | 作用 |
|---|---|
| `el_a3_sdk/.gitignore` | SDK 子目录忽略规则。 |
| `el_a3_sdk/README.md` | SDK 说明文档。 |
| `el_a3_sdk/README_en.md` | SDK 英文说明。 |
| `el_a3_sdk/README_zh.md` | SDK 中文说明。 |
| `el_a3_sdk/setup.py` | Python 包安装配置，依赖 `numpy`、`pyyaml`，可选 `pin`、`pyserial`、PyQt/PyVista debugger 依赖。 |
| `el_a3_sdk/el_a3_debugger.spec` | PyInstaller 打包 MotorStudio/Debugger 上位机的 spec 文件。 |

## el_a3_sdk/el_a3_sdk

这是核心 Python SDK 包。

| 文件 | 作用 |
|---|---|
| `el_a3_sdk/el_a3_sdk/__init__.py` | SDK 包入口，导出 `ELA3Interface`、数据结构、协议、轨迹等，并按需加载运动学/SLCAN。 |
| `el_a3_sdk/el_a3_sdk/arm_manager.py` | 多机械臂管理器，可注册多个 CAN/SLCAN 机械臂实例并统一连接/断开。 |
| `el_a3_sdk/el_a3_sdk/can_driver.py` | Linux SocketCAN 后端，实现 Robstride CAN 通信线程、收发和反馈缓存。 |
| `el_a3_sdk/el_a3_sdk/controller_profiles.py` | 手柄 profile、轴/按钮映射、设备自动识别和命令行检测工具。 |
| `el_a3_sdk/el_a3_sdk/data_types.py` | SDK 数据结构，包含电机反馈、关节状态、末端位姿、状态机、动力学信息等 dataclass。 |
| `el_a3_sdk/el_a3_sdk/interface.py` | SDK 主接口 `ELA3Interface`，封装连接、使能、关节控制、轨迹、控制循环、重力补偿、零力矩等。 |
| `el_a3_sdk/el_a3_sdk/joystick.py` | Linux joystick 设备读取封装，用于 demo/xbox_control。 |
| `el_a3_sdk/el_a3_sdk/kinematics.py` | 基于 Pinocchio 的 FK、IK、Jacobian、重力力矩、动力学计算。 |
| `el_a3_sdk/el_a3_sdk/protocol.py` | Robstride 协议常量、枚举、参数索引、电机型号参数、默认关节配置。 |
| `el_a3_sdk/el_a3_sdk/slcan_can_driver.py` | 串口 SLCAN 后端，适合 Windows/串口 CAN 适配器。 |
| `el_a3_sdk/el_a3_sdk/trajectory.py` | S 曲线、多关节同步、三次样条等轨迹规划工具。 |
| `el_a3_sdk/el_a3_sdk/utils.py` | 浮点/uint16 映射、角度转换、限幅、欧拉角/四元数、插值等工具函数。 |

## el_a3_sdk/demo

| 文件 | 作用 |
|---|---|
| `el_a3_sdk/demo/__init__.py` | demo 包初始化文件，当前为空。 |
| `el_a3_sdk/demo/cartesian_control_demo.py` | 笛卡尔空间控制示例，演示末端位姿/直线运动控制。 |
| `el_a3_sdk/demo/control_loop_demo.py` | SDK 后台 200Hz 控制循环示例。 |
| `el_a3_sdk/demo/dynamics_demo.py` | Pinocchio 动力学/重力补偿相关示例。 |
| `el_a3_sdk/demo/motion_control.py` | 基础关节运动控制示例。 |
| `el_a3_sdk/demo/read_joints.py` | 读取关节状态、速度、力矩等反馈的示例。 |
| `el_a3_sdk/demo/slcan_test.py` | SLCAN 后端连接与通信测试示例。 |
| `el_a3_sdk/demo/trajectory_demo.py` | S 曲线、多关节、三次样条轨迹规划示例。 |
| `el_a3_sdk/demo/waypoint_loop_real.py` | 真实机械臂循环执行路径点的示例。 |
| `el_a3_sdk/demo/waypoints_config.py` | 路径点数据定义和摘要工具。 |
| `el_a3_sdk/demo/xbox_control.py` | 纯 SDK 版 Xbox 手柄控制示例，不依赖 ROS。 |
| `el_a3_sdk/demo/zero_torque_mode.py` | 零力矩/拖动示教模式示例。 |

## el_a3_sdk/docs

| 文件 | 作用 |
|---|---|
| `el_a3_sdk/docs/SDK_API_Protocol.md` | SDK API 与通信协议文档。 |
| `el_a3_sdk/docs/电机通信协议汇总.md` | 电机通信协议中文汇总，记录 CAN 帧、参数、控制模式等。 |

## el_a3_sdk/resources

| 文件 | 作用 |
|---|---|
| `el_a3_sdk/resources/config/inertia_params.yaml` | SDK 默认惯性/重力补偿参数。 |
| `el_a3_sdk/resources/urdf/el_a3.urdf` | SDK 使用的当前 EL-A3 URDF 模型。 |
| `el_a3_sdk/resources/urdf/el_a3_legacy.urdf` | 旧版/兼容版 URDF 模型。 |
| `el_a3_sdk/resources/rs05/urdf/el_a3.urdf` | RS05 腕部电机版本的 URDF。 |
| `el_a3_sdk/resources/rs05/config/inertia_params.yaml` | RS05 版本惯性参数。 |
| `el_a3_sdk/resources/rs05/config/inertia_params_arm2.yaml` | RS05 第二机械臂惯性参数。 |
| `el_a3_sdk/resources/meshes/el05_l6.stl` | SDK 3D 视图/URDF 使用的 EL05/L6 网格。 |
| `el_a3_sdk/resources/meshes/jaw.stl` | SDK 3D 视图/URDF 使用的夹爪网格。 |
| `el_a3_sdk/resources/meshes/l1.stl` | SDK 3D 视图/URDF 使用的第 1 连杆网格。 |
| `el_a3_sdk/resources/meshes/l2.stl` | SDK 3D 视图/URDF 使用的第 2 连杆网格。 |
| `el_a3_sdk/resources/meshes/l3.stl` | SDK 3D 视图/URDF 使用的第 3 连杆网格。 |
| `el_a3_sdk/resources/meshes/l4.stl` | SDK 3D 视图/URDF 使用的第 4 连杆网格。 |
| `el_a3_sdk/resources/meshes/l5.stl` | SDK 3D 视图/URDF 使用的第 5 连杆网格。 |
| `el_a3_sdk/resources/meshes/l6.stl` | SDK 3D 视图/URDF 使用的第 6 连杆网格。 |

## el_a3_sdk/scripts

| 文件 | 作用 |
|---|---|
| `el_a3_sdk/scripts/detect_controller.py` | 检测手柄设备与 profile 的命令行脚本。 |
| `el_a3_sdk/scripts/setup_can.sh` | SDK 侧单 CAN 接口配置脚本。 |
| `el_a3_sdk/scripts/setup_multi_can.sh` | SDK 侧多 CAN 接口配置脚本。 |

## el_a3_sdk/scripts/tests

| 文件 | 作用 |
|---|---|
| `el_a3_sdk/scripts/tests/test_basic_cartesian.py` | 基础笛卡尔控制实机/SDK 测试。 |
| `el_a3_sdk/scripts/tests/test_basic_connection.py` | SDK 连接、使能、读取反馈等基础连接测试。 |
| `el_a3_sdk/scripts/tests/test_basic_motion.py` | 基础关节运动测试。 |
| `el_a3_sdk/scripts/tests/test_basic_multi_arm.py` | 多机械臂管理器基础测试。 |
| `el_a3_sdk/scripts/tests/test_basic_params.py` | 参数读写/配置相关基础测试。 |
| `el_a3_sdk/scripts/tests/test_basic_zero_torque.py` | SDK 零力矩模式基础测试。 |
| `el_a3_sdk/scripts/tests/test_can2_bus_analyzer.py` | CAN2 总线分析脚本，统计电机反馈和总线状态。 |
| `el_a3_sdk/scripts/tests/test_can_codec.py` | CAN 协议浮点/uint16 编解码单元测试。 |
| `el_a3_sdk/scripts/tests/test_hw_can_comm.py` | 真实硬件 CAN 通信测试。 |
| `el_a3_sdk/scripts/tests/test_hw_safety.py` | 硬件安全逻辑测试，如力矩映射一致性、关节限位保护。 |
| `el_a3_sdk/scripts/tests/test_joint_config.py` | 关节限位、方向、offset、电机类型配置测试。 |
| `el_a3_sdk/scripts/tests/test_protocol.py` | 协议枚举、电机参数、默认映射等单元测试。 |
| `el_a3_sdk/scripts/tests/test_state_machine.py` | SDK 状态机转换测试。 |

## el_a3_sdk/MotorStudio

MotorStudio 是 PyQt6 上位机/调试器。

| 文件 | 作用 |
|---|---|
| `el_a3_sdk/MotorStudio/__init__.py` | MotorStudio 包初始化文件。 |
| `el_a3_sdk/MotorStudio/main.py` | 上位机启动入口，创建 Qt 应用和主窗口。 |
| `el_a3_sdk/MotorStudio/main_window.py` | 主窗口，组织工具栏、3D 视图、控制、监控、示教、标定等面板。 |
| `el_a3_sdk/MotorStudio/backend/__init__.py` | backend 包初始化文件。 |
| `el_a3_sdk/MotorStudio/backend/arm_worker.py` | 后台线程封装 SDK 机械臂连接、控制和状态刷新，避免 GUI 阻塞。 |
| `el_a3_sdk/MotorStudio/backend/calibration_worker.py` | GUI 标定后台线程和标定数据读写/拟合逻辑。 |
| `el_a3_sdk/MotorStudio/backend/data_buffer.py` | 实时数据环形缓冲，用于监控曲线。 |
| `el_a3_sdk/MotorStudio/backend/trajectory_recorder.py` | 示教轨迹点记录、保存、加载、回放数据结构。 |
| `el_a3_sdk/MotorStudio/utils/__init__.py` | utils 包初始化文件。 |
| `el_a3_sdk/MotorStudio/utils/can_utils.py` | 检测/配置 CAN 接口和串口设备的工具函数。 |
| `el_a3_sdk/MotorStudio/utils/i18n.py` | GUI 文案翻译/国际化工具。 |
| `el_a3_sdk/MotorStudio/utils/joint_drag_controls.py` | 3D 视图中拖动关节的几何计算与控制逻辑。 |
| `el_a3_sdk/MotorStudio/utils/style.py` | GUI 样式表和界面视觉风格配置。 |
| `el_a3_sdk/MotorStudio/utils/theme_manager.py` | 主题管理器，处理明暗主题/样式切换。 |
| `el_a3_sdk/MotorStudio/utils/urdf_loader.py` | 轻量 URDF 解析器，读取 link、joint、visual、mesh 并计算姿态。 |
| `el_a3_sdk/MotorStudio/widgets/__init__.py` | widgets 包初始化文件。 |
| `el_a3_sdk/MotorStudio/widgets/calibration_panel.py` | 标定面板，启动/显示动力学或重力标定流程。 |
| `el_a3_sdk/MotorStudio/widgets/diagnostics_panel.py` | 诊断面板，显示连接、错误、设备状态等。 |
| `el_a3_sdk/MotorStudio/widgets/gamepad_panel.py` | 手柄输入状态面板，显示轴、按钮、profile。 |
| `el_a3_sdk/MotorStudio/widgets/gripper_panel.py` | 夹爪控制面板。 |
| `el_a3_sdk/MotorStudio/widgets/joint_control_panel.py` | 关节控制面板，提供每个关节的目标、使能、运动控制。 |
| `el_a3_sdk/MotorStudio/widgets/monitoring_panel.py` | 实时监控面板，展示关节/电机状态曲线。 |
| `el_a3_sdk/MotorStudio/widgets/monitoring_window.py` | 独立监控窗口。 |
| `el_a3_sdk/MotorStudio/widgets/teaching_panel.py` | 示教/录制/回放面板。 |
| `el_a3_sdk/MotorStudio/widgets/toolbar_panel.py` | 顶部工具栏面板，包含连接、使能、急停等常用操作。 |
| `el_a3_sdk/MotorStudio/widgets/trajectory_panel.py` | 轨迹规划/执行面板。 |
| `el_a3_sdk/MotorStudio/widgets/viewer_3d.py` | PyVista/Qt 3D 机器人显示面板，加载 URDF 和 STL 并支持交互。 |

## hardware

| 文件 | 作用 |
|---|---|
| `hardware/README.md` | 硬件资料目录说明，介绍 step、3mf、wiring、pcb、assembly_sop 等。 |
| `hardware/3mf/.gitkeep` | 保留 3MF 目录的空占位文件。 |
| `hardware/3mf/Edulite_A3.3mf` | 3D 打印制造文件，通常可由切片软件打开。 |
| `hardware/assembly_sop/.gitkeep` | 保留装配 SOP 目录的占位文件。 |
| `hardware/assembly_sop/A3_Assembly_SOP.pdf` | 英文/默认版装配标准作业流程 PDF。 |
| `hardware/assembly_sop/A3_Assembly_SOP_CN .pdf` | 中文装配标准作业流程 PDF，文件名中 `CN` 后有一个空格。 |
| `hardware/pcb/.gitkeep` | 保留 PCB 根目录的占位文件。 |
| `hardware/pcb/gerber/.gitkeep` | 保留 Gerber 目录的占位文件。 |
| `hardware/pcb/gerber/GERBER_EDU_A3_PCB_1_V1.zip` | PCB 1 的 Gerber 制造压缩包，可提交给 PCB 厂。 |
| `hardware/pcb/gerber/GERBER_EDU_A3_PCB_2_V1.zip` | PCB 2 的 Gerber 制造压缩包，可提交给 PCB 厂。 |
| `hardware/pcb/pcb/.gitkeep` | 保留 PCB 源文件目录的占位文件。 |
| `hardware/pcb/pcb/EDU_A3_PCB_1_v1.epro` | PCB 1 工程源文件，适合用对应 EDA 工具打开。 |
| `hardware/pcb/pcb/EDU_LITE_A3_PCB_2_V1.epro2` | PCB 2 工程源文件，适合用对应 EDA 工具打开。 |
| `hardware/step/.gitkeep` | 保留 STEP 目录的占位文件。 |
| `hardware/step/Edulite_A3.step` | 整机 STEP 3D CAD 模型，可用于机械查看、改型、装配检查。 |
| `hardware/wiring/.gitkeep` | 保留线束目录的占位文件。 |
| `hardware/wiring/Model 1.pdf` | 线束/接线图纸第 1 份。 |
| `hardware/wiring/Model 2.pdf` | 线束/接线图纸第 2 份。 |
| `hardware/wiring/Model 3.pdf` | 线束/接线图纸第 3 份。 |
| `hardware/wiring/Model 4.pdf` | 线束/接线图纸第 4 份。 |
| `hardware/wiring/Model 5.pdf` | 线束/接线图纸第 5 份。 |

