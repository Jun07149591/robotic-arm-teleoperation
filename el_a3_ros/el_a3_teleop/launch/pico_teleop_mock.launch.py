"""
EL-A3 PICO/WebXR teleoperation launch file for MoveIt mock hardware.

This launch starts:
  1. el_a3_moveit_config demo.launch.py  (mock hardware + RViz + arm_controller)
  2. pico3_webxr_pose_receiver.py        (HTTPS/WSS WebXR receiver, optional)
  3. vr_teleop.py --sim --ros-sim        (PICO pose -> IK -> JointTrajectory)

Recommended placement:
  el_a3_teleop/launch/pico_teleop_mock.launch.py

Recommended script placement:
  el_a3_teleop/scripts/pico3_webxr_pose_receiver.py
  el_a3_teleop/scripts/vr_teleop.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "start_moveit_demo",
            default_value="true",
            description="Start MoveIt demo.launch.py with mock hardware and RViz.",
        ),
        DeclareLaunchArgument(
            "start_pico_receiver",
            default_value="true",
            description="Start the PICO WebXR pose receiver.",
        ),
        DeclareLaunchArgument(
            "pico_host",
            default_value="0.0.0.0",
            description="Bind address for PICO WebXR pose receiver.",
        ),
        DeclareLaunchArgument(
            "pico_port",
            default_value="8765",
            description="Port for PICO WebXR pose receiver.",
        ),
        DeclareLaunchArgument(
            "scale_pos",
            default_value="0.3",
            description="Position scale from PICO motion to robot motion.",
        ),
        DeclareLaunchArgument(
            "scale_rot",
            default_value="0.0",
            description="Rotation scale. Keep 0.0 first to fix end-effector orientation.",
        ),
        DeclareLaunchArgument(
            "dead_zone",
            default_value="0.02",
            description="Position dead zone in meters.",
        ),
        DeclareLaunchArgument(
            "hand",
            default_value="right",
            description="Controller hand: right or left.",
        ),
        DeclareLaunchArgument(
            "pose_type",
            default_value="grip",
            description="WebXR pose type: grip or targetRay.",
        ),
        DeclareLaunchArgument(
            "receiver_script",
            default_value=PathJoinSubstitution([
                FindPackageShare("el_a3_teleop"), "scripts", "pico3_webxr_pose_receiver.py"
            ]),
            description="Path to pico3_webxr_pose_receiver.py.",
        ),
        DeclareLaunchArgument(
            "teleop_script",
            default_value=PathJoinSubstitution([
                FindPackageShare("el_a3_teleop"), "scripts", "vr_teleop.py"
            ]),
            description="Path to vr_teleop.py.",
        ),
    ]

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("el_a3_moveit_config"), "launch", "demo.launch.py"
            ])
        ),
        launch_arguments={"use_rviz": "true"}.items(),
        condition=IfCondition(LaunchConfiguration("start_moveit_demo")),
    )

    pico_receiver = ExecuteProcess(
        cmd=[
            "python3",
            LaunchConfiguration("receiver_script"),
            "--host", LaunchConfiguration("pico_host"),
            "--port", LaunchConfiguration("pico_port"),
            "--regen-cert",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_pico_receiver")),
    )

    # Delay teleop a few seconds so controller_manager and arm_controller can become active.
    pico_teleop = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    LaunchConfiguration("teleop_script"),
                    "--sim",
                    "--ros-sim",
                    "--scale-pos", LaunchConfiguration("scale_pos"),
                    "--scale-rot", LaunchConfiguration("scale_rot"),
                    "--dead-zone", LaunchConfiguration("dead_zone"),
                    "--hand", LaunchConfiguration("hand"),
                    "--pose-type", LaunchConfiguration("pose_type"),
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            moveit_demo,
            pico_receiver,
            pico_teleop,
        ]
    )
