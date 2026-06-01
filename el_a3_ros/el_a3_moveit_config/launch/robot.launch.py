"""
EL-A3 Robot Arm MoveIt Real Hardware Launch File

Launches MoveIt motion planning with real hardware control
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface name",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "host_can_id",
            default_value="253",
            description="Host CAN ID (0xFD = 253)",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_motor_type",
            default_value="EL05",
            description="Wrist motor type: EL05 or RS05",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 with MoveIt plugin",
        )
    )

    can_interface = LaunchConfiguration("can_interface")
    host_can_id = LaunchConfiguration("host_can_id")
    wrist_motor_type = LaunchConfiguration("wrist_motor_type")
    use_rviz = LaunchConfiguration("use_rviz")

    # Get URDF via xacro (real hardware)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("el_a3_description"), "urdf", "el_a3.urdf.xacro"]
            ),
            " use_mock_hardware:=false",
            " can_interface:=",
            can_interface,
            " host_can_id:=",
            host_can_id,
            " wrist_motor_type:=",
            wrist_motor_type,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # SRDF
    robot_description_semantic_content = Command(
        [
            "cat ",
            PathJoinSubstitution(
                [FindPackageShare("el_a3_moveit_config"), "config", "el_a3.srdf"]
            ),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # Kinematics
    kinematics_yaml = load_yaml("el_a3_moveit_config", "config/kinematics.yaml")

    # Joint limits
    joint_limits_yaml = load_yaml("el_a3_moveit_config", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # Planning
    ompl_planning_yaml = load_yaml("el_a3_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_yaml}

    # Controllers
    moveit_controllers_yaml = load_yaml("el_a3_moveit_config", "config/moveit_controllers.yaml")

    # Trajectory execution - increased timeout tolerance to support fast teleoperation
    trajectory_execution = {
        "moveit_manage_controllers": False,  # Let ros2_control manage controllers
        "trajectory_execution.allowed_execution_duration_scaling": 4.0,  # Increased to 4x for more lenient execution time
        "trajectory_execution.allowed_goal_duration_margin": 2.0,  # Increased goal duration margin
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ros2_control controllers config
    ros2_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("el_a3_description"), "config", "el_a3_controllers.yaml"]
    )

    # Resolve zero_torque_controller paths dynamically (avoid hardcoded absolute paths)
    desc_share = get_package_share_directory("el_a3_description")
    zero_torque_path_overrides = {
        "zero_torque_controller.ros__parameters.urdf_path":
            os.path.join(desc_share, "urdf", "el_a3.urdf"),
        "zero_torque_controller.ros__parameters.inertia_config_path":
            os.path.join(desc_share, "config", "inertia_params.yaml"),
    }

    # RViz config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("el_a3_moveit_config"), "config", "moveit.rviz"]
    )

    # Nodes
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_yaml, zero_torque_path_overrides],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Static transform: world -> base_link
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="log",
    )

    # Spawner nodes - add wait time for controller_manager to be ready
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", 
                   "--controller-manager-timeout", "120"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager", 
                   "--controller-manager-timeout", "120"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )

    zero_torque_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zero_torque_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "120", "--inactive"],
    )

    # Move group - delay start until controllers are ready
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
        ],
    )

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            robot_description_kinematics,
        ],
        condition=IfCondition(use_rviz),
    )

    # Event handlers for sequential startup
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner, zero_torque_controller_spawner],
        )
    )

    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    nodes = [
        ros2_control_node,
        robot_state_publisher_node,
        static_tf_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller,
        delay_move_group,
        rviz_node,
    ]

    _project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    no_shm_xml = os.path.join(_project_root, 'ros2_ws', 'fastrtps_no_shm.xml')
    if not os.path.isfile(no_shm_xml):
        no_shm_xml = os.path.join(_project_root, 'fastrtps_no_shm.xml')
    set_fastrtps_env = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE', no_shm_xml)

    return LaunchDescription([set_fastrtps_env] + declared_arguments + nodes)

