"""
EL-A3 Pico VR dual-arm teleoperation launch file.

Launches multi-arm ros2_control stack + two pico_teleop_node instances
(left hand -> arm1, right hand -> arm2). Supports mock hardware and RViz.

Usage:
  ros2 launch el_a3_teleop pico_teleop_dual.launch.py use_mock_hardware:=true
  ros2 launch el_a3_teleop pico_teleop_dual.launch.py use_mock_hardware:=true use_rviz:=true
"""

import os
import yaml
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _find_fastrtps_xml() -> str:
    here = os.path.realpath(__file__)
    ws = os.path.dirname(os.path.dirname(os.path.dirname(here)))
    candidate = os.path.join(ws, "fastrtps_no_shm.xml")
    if os.path.isfile(candidate):
        return candidate
    here_abs = os.path.abspath(__file__)
    for levels in range(3, 7):
        d = here_abs
        for _ in range(levels):
            d = os.path.dirname(d)
        c = os.path.join(d, "fastrtps_no_shm.xml")
        if os.path.isfile(c):
            return c
    return candidate


def _configure_launch(context, *args, **kwargs):
    """Build dual-arm nodes."""
    use_mock = LaunchConfiguration("use_mock_hardware").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    can0 = LaunchConfiguration("can_interface_left").perform(context)
    can1 = LaunchConfiguration("can_interface_right").perform(context)

    # Derive desc_share from teleop_share (same workspace)
    teleop_share = LaunchConfiguration("teleop_share").perform(context)
    ws_root = os.path.dirname(os.path.dirname(teleop_share))  # up from share/el_a3_teleop
    # el_a3_description is in install/el_a3_description/share/el_a3_description
    desc_share = os.path.join(ws_root, "el_a3_description", "share", "el_a3_description")
    rviz_config = os.path.join(desc_share, "config", "el_a3_view.rviz")

    nodes = []

    # ---- Arm1 (left arm, can0) ----
    controller_yaml_left = _write_controller_config("arm1", "arm1_")
    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="arm1_control_node",
        parameters=[
            _robot_desc_xacro(desc_share, "arm1_", can0, use_mock),
            controller_yaml_left,
        ],
        output="screen",
        remappings=[("~/robot_description", "/arm1/robot_description")],
    ))
    for spawner_name, spawner_args in [
        ("arm1_jsb", ["arm1_joint_state_broadcaster", "-c", "/arm1/controller_manager", "-t", "60"]),
        ("arm1_arm", ["arm1_arm_controller", "-c", "/arm1/controller_manager", "-t", "60"]),
        ("arm1_grip", ["arm1_gripper_controller", "-c", "/arm1/controller_manager", "-t", "60"]),
    ]:
        nodes.append(Node(
            package="controller_manager", executable="spawner",
            name=spawner_name, arguments=spawner_args, output="screen",
        ))

    # ---- Arm2 (right arm, can1) ----
    controller_yaml_right = _write_controller_config("arm2", "arm2_")
    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="arm2_control_node",
        parameters=[
            _robot_desc_xacro(desc_share, "arm2_", can1, use_mock),
            controller_yaml_right,
        ],
        output="screen",
        remappings=[("~/robot_description", "/arm2/robot_description")],
    ))
    for spawner_name, spawner_args in [
        ("arm2_jsb", ["arm2_joint_state_broadcaster", "-c", "/arm2/controller_manager", "-t", "60"]),
        ("arm2_arm", ["arm2_arm_controller", "-c", "/arm2/controller_manager", "-t", "60"]),
        ("arm2_grip", ["arm2_gripper_controller", "-c", "/arm2/controller_manager", "-t", "60"]),
    ]:
        nodes.append(Node(
            package="controller_manager", executable="spawner",
            name=spawner_name, arguments=spawner_args, output="screen",
        ))

    # ---- Robot State Publishers ----
    for arm_name, prefix in [("arm1", "arm1_"), ("arm2", "arm2_")]:
        nodes.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name=f"{arm_name}_rsp", output="screen",
            parameters=[_robot_desc_xacro(desc_share, prefix, can0 if arm_name == "arm1" else can1, use_mock)],
        ))

    # ---- Pico teleop nodes (left hand -> arm1, right hand -> arm2) ----
    teleop_config = os.path.join(teleop_share, "config", "pico_teleop.yaml")

    nodes.append(Node(
        package="el_a3_teleop", executable="pico_teleop_node",
        name="pico_teleop_left",
        parameters=[teleop_config, {
            "hand": "left",
            "arm_controller_topic": "/arm1/arm1_arm_controller/joint_trajectory",
            "gripper_controller_topic": "/arm1/arm1_gripper_controller/joint_trajectory",
        }],
        output="screen",
    ))

    nodes.append(Node(
        package="el_a3_teleop", executable="pico_teleop_node",
        name="pico_teleop_right",
        parameters=[teleop_config, {
            "hand": "right",
            "arm_controller_topic": "/arm2/arm2_arm_controller/joint_trajectory",
            "gripper_controller_topic": "/arm2/arm2_gripper_controller/joint_trajectory",
        }],
        output="screen",
    ))

    # ---- RViz ----
    if use_rviz.lower() == "true":
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2",
            output="screen", arguments=["-d", rviz_config],
        ))

    return nodes


def _robot_desc_xacro(pkg_share: str, prefix: str, can_iface: str, use_mock: str):
    """Generate robot_description param dict via xacro command."""
    import subprocess
    from launch_ros.parameter_descriptions import ParameterValue

    mock_flag = "true" if use_mock.lower() == "true" else "false"
    xacro_file = os.path.join(pkg_share, "urdf", "el_a3.urdf.xacro")
    result = subprocess.run(
        ["xacro", xacro_file,
         f"prefix:={prefix}",
         f"use_mock_hardware:={mock_flag}",
         f"can_interface:={can_iface}",
         "host_can_id:=253"],
        capture_output=True, text=True,
    )
    urdf_text = result.stdout
    return {"robot_description": ParameterValue(urdf_text, value_type=str)}


def _write_controller_config(arm_ns: str, prefix: str) -> str:
    """Write temporary controller YAML for one arm."""
    joints = [f"L{i}_joint" for i in range(1, 7)]
    params = {
        f"/{arm_ns}/controller_manager": {
            "ros__parameters": {
                "update_rate": 200,
                f"{prefix}joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                f"{prefix}arm_controller": {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                },
                f"{prefix}gripper_controller": {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                },
            }
        },
        f"/{arm_ns}/{prefix}arm_controller": {
            "ros__parameters": {
                "joints": joints,
                "command_interfaces": ["position", "velocity"],
                "state_interfaces": ["position", "velocity"],
                "open_loop_control": True,
                "allow_nonzero_velocity_at_trajectory_end": True,
                "interpolation_method": "splines",
                "state_publish_rate": 200.0,
                "action_monitor_rate": 50.0,
            }
        },
        f"/{arm_ns}/{prefix}gripper_controller": {
            "ros__parameters": {
                "joints": ["L7_joint"],
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "open_loop_control": True,
                "allow_nonzero_velocity_at_trajectory_end": True,
                "interpolation_method": "splines",
                "state_publish_rate": 50.0,
                "action_monitor_rate": 20.0,
            }
        },
    }
    fd, path = tempfile.mkstemp(prefix=f"{arm_ns}_ctrls_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.dump(params, f, default_flow_style=False)
    return path


def generate_launch_description():
    teleop_share = FindPackageShare("el_a3_teleop")

    declared_arguments = [
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="Use mock hardware"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="Start RViz2"),
        DeclareLaunchArgument("can_interface_left", default_value="can0",
                              description="Left arm CAN interface"),
        DeclareLaunchArgument("can_interface_right", default_value="can1",
                              description="Right arm CAN interface"),
        DeclareLaunchArgument("teleop_share", default_value=teleop_share,
                              description="el_a3_teleop share path"),
    ]

    return LaunchDescription(
        declared_arguments
        + [
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", _find_fastrtps_xml()),
            OpaqueFunction(function=_configure_launch),
        ]
    )
