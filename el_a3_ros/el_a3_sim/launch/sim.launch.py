"""Launch the EL-A3 arm in local mock simulation with optional auto motion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    auto_motion = LaunchConfiguration("auto_motion")
    loop = LaunchConfiguration("loop")
    speed_scale = LaunchConfiguration("speed_scale")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    dwell_scale = LaunchConfiguration("dwell_scale")
    wrist_motor_type = LaunchConfiguration("wrist_motor_type")

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("el_a3_description"),
                    "launch",
                    "el_a3_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_mock_hardware": "true",
            "use_rviz": use_rviz,
            "wrist_motor_type": wrist_motor_type,
        }.items(),
    )

    auto_motion_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="el_a3_sim",
                executable="el_a3_auto_motion",
                name="el_a3_auto_motion",
                output="screen",
                parameters=[
                    {
                        "loop": ParameterValue(loop, value_type=bool),
                        "speed_scale": ParameterValue(speed_scale, value_type=float),
                        "start_delay_sec": ParameterValue(start_delay_sec, value_type=float),
                        "dwell_scale": ParameterValue(dwell_scale, value_type=float),
                    }
                ],
                condition=IfCondition(auto_motion),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Open RViz2 with the EL-A3 robot model.",
            ),
            DeclareLaunchArgument(
                "auto_motion",
                default_value="true",
                description="Automatically send a looping demo trajectory.",
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="true",
                description="Loop the demo trajectory sequence.",
            ),
            DeclareLaunchArgument(
                "speed_scale",
                default_value="1.0",
                description="Motion speed multiplier. 2.0 runs twice as fast.",
            ),
            DeclareLaunchArgument(
                "start_delay_sec",
                default_value="6.0",
                description="Delay before the auto motion node sends the first goal.",
            ),
            DeclareLaunchArgument(
                "dwell_scale",
                default_value="1.0",
                description="Scale pause time between demo poses.",
            ),
            DeclareLaunchArgument(
                "wrist_motor_type",
                default_value="EL05",
                description="Wrist motor type used in xacro limits: EL05 or RS05.",
            ),
            control_launch,
            auto_motion_node,
        ]
    )
