"""
EL-A3 Xbox/gamepad teleoperation launch file.

Launches the full ros2_control stack + joy_node + xbox_teleop_node.
Performs automatic controller detection using el_a3_sdk.controller_profiles
and injects the resolved profile into the teleop node.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _find_fastrtps_xml() -> str:
    """Locate fastrtps_no_shm.xml in the workspace root (el_a3_ros/)."""
    # With --symlink-install, realpath resolves to source tree
    here = os.path.realpath(__file__)
    # .../el_a3_teleop/launch/real_xbox_teleop.launch.py → go up 3 levels to el_a3_ros/
    ws = os.path.dirname(os.path.dirname(os.path.dirname(here)))
    candidate = os.path.join(ws, "fastrtps_no_shm.xml")
    if os.path.isfile(candidate):
        return candidate
    # Fallback: try from install path (without symlink resolution)
    here_abs = os.path.abspath(__file__)
    for levels in range(3, 7):
        d = here_abs
        for _ in range(levels):
            d = os.path.dirname(d)
        c = os.path.join(d, "fastrtps_no_shm.xml")
        if os.path.isfile(c):
            return c
    return candidate  # return best guess even if not found


def _create_input_nodes(context, *_args, **_kwargs):
    """OpaqueFunction: auto-detect controller and construct joy + teleop nodes."""
    from el_a3_sdk.controller_profiles import detect_controller

    auto_detect = context.launch_configurations.get("auto_detect_controller", "true")
    joy_dev_override = context.launch_configurations.get("joy_dev_override", "")
    profile_override = context.launch_configurations.get("controller_profile_override", "")

    device = joy_dev_override if joy_dev_override else "/dev/input/js0"
    requested_profile = profile_override if profile_override else "auto"

    if auto_detect.lower() == "true":
        detection = detect_controller(device, requested_profile=requested_profile)
        resolved_device = detection.resolved_device
        profile_id = detection.profile.profile_id
        controller_name = detection.name or "unknown"
        summary = (
            f"device={resolved_device} name={controller_name} "
            f"profile={profile_id} ({detection.profile.display_name}) "
            f"source={detection.source}"
        )
    else:
        resolved_device = device
        profile_id = profile_override if profile_override else "xbox_default"
        controller_name = "manual"
        summary = f"手动模式: device={resolved_device} profile={profile_id}"

    teleop_config = PathJoinSubstitution(
        [FindPackageShare("el_a3_teleop"), "config", "xbox_teleop.yaml"]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"dev": resolved_device, "deadzone": 0.05, "autorepeat_rate": 20.0}],
        output="screen",
    )

    teleop_node = Node(
        package="el_a3_teleop",
        executable="xbox_teleop_node",
        name="xbox_teleop_node",
        parameters=[
            teleop_config,
            {
                "controller_profile": profile_id,
                "controller_name": controller_name,
                "joy_device": resolved_device,
            },
        ],
        output="screen",
    )

    return [
        LogInfo(msg=f"[el_a3_teleop] 控制器检测: {summary}"),
        joy_node,
        teleop_node,
    ]


def generate_launch_description():
    el_a3_desc_share = FindPackageShare("el_a3_description")

    declared_arguments = [
        DeclareLaunchArgument(
            "auto_detect_controller", default_value="true",
            description="Automatically detect joystick type and select profile",
        ),
        DeclareLaunchArgument(
            "joy_dev_override", default_value="",
            description="Override joystick device path (e.g. /dev/input/js1)",
        ),
        DeclareLaunchArgument(
            "controller_profile_override", default_value="",
            description="Force a controller profile (e.g. xbox_default, zikway_3537_1041, generic_hid)",
        ),
        DeclareLaunchArgument(
            "can_interface", default_value="can0",
            description="CAN interface name",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false",
            description="Start RViz2",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
            description="Use mock hardware for simulation",
        ),
    ]

    el_a3_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [el_a3_desc_share, "launch", "el_a3_control.launch.py"]
            )
        ),
        launch_arguments={
            "can_interface": LaunchConfiguration("can_interface"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                _find_fastrtps_xml(),
            ),
            el_a3_control_launch,
            OpaqueFunction(function=_create_input_nodes),
        ]
    )
