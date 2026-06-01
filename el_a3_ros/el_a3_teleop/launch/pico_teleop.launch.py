"""
EL-A3 Pico VR teleoperation launch file.

Launches ros2_control stack + pico_teleop_node.
Reads Pico WebXR poses from /tmp/pico_latest_pose.json (written by
pico3_webxr_pose_receiver.py). Supports mock hardware simulation and RViz.

Usage:
  # Simulation (no hardware)
  ros2 launch el_a3_teleop pico_teleop.launch.py use_mock_hardware:=true

  # Simulation + RViz
  ros2 launch el_a3_teleop pico_teleop.launch.py use_mock_hardware:=true use_rviz:=true

  # Real hardware
  ros2 launch el_a3_teleop pico_teleop.launch.py can_interface:=can0
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
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


def generate_launch_description():
    el_a3_desc_share = FindPackageShare("el_a3_description")
    el_a3_teleop_share = FindPackageShare("el_a3_teleop")

    declared_arguments = [
        DeclareLaunchArgument("can_interface", default_value="can0",
                              description="CAN interface name"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="Start RViz2"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="Use mock hardware for simulation"),
    ]

    # Include the standard el_a3_control launch (controller_manager + hardware)
    el_a3_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([el_a3_desc_share, "launch", "el_a3_control.launch.py"])
        ),
        launch_arguments={
            "can_interface": LaunchConfiguration("can_interface"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
        }.items(),
    )

    # Pico teleop node
    pico_teleop_node = Node(
        package="el_a3_teleop",
        executable="pico_teleop_node",
        name="pico_teleop_node",
        parameters=[
            PathJoinSubstitution([el_a3_teleop_share, "config", "pico_teleop.yaml"]),
        ],
        output="screen",
    )

    _sdk_path = os.path.realpath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "el_a3_sdk"))
    _pythonpath = os.environ.get("PYTHONPATH", "")
    _pythonpath = _sdk_path if not _pythonpath else _pythonpath + os.pathsep + _sdk_path

    return LaunchDescription(
        declared_arguments
        + [
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", _find_fastrtps_xml()),
            SetEnvironmentVariable("PYTHONPATH", _pythonpath),
            el_a3_control_launch,
            pico_teleop_node,
        ]
    )
