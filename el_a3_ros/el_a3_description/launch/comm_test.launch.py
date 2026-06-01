"""
EL-A3 通信测试 Launch File

仅启动 robot_state_publisher + RViz，不启动 ros2_control。
配合 el_a3_sdk/demo/comm_test_rviz.py 使用，该脚本通过 CAN 发送 Type 1
并接收 Type 2 反馈，发布 /joint_states 驱动 RViz 模型。

用法:
  # 终端 1
  ros2 launch el_a3_description comm_test.launch.py

  # 终端 2
  python3 el_a3_sdk/demo/comm_test_rviz.py --can can0
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("el_a3_description"), "urdf", "el_a3.urdf.xacro"]
            ),
            " use_mock_hardware:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("el_a3_description"), "config", "el_a3_view.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        robot_state_pub_node,
        rviz_node,
    ])
