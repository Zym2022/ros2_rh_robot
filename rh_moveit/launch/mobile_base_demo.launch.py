import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="mobile_base.rviz",
            description="RViz configuration file"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joint_state_gui",
            default_value="true",
            description="Launch joint state publisher GUI for testing"
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")

    # MoveIt configuration using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("mobile_base", package_name="rh_moveit")
        .robot_description(file_path="config/mobile_base/robot.urdf.xacro")
        .robot_description_semantic(file_path="config/mobile_base/robot.srdf")
        .robot_description_kinematics(file_path="config/mobile_base/kinematics.yaml")
        .joint_limits(file_path="config/mobile_base/joint_limits.yaml")
        .trajectory_execution(file_path="config/mobile_base/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            PathJoinSubstitution([
                FindPackageShare("rh_moveit"),
                "config", "mobile_base", "ompl_planning.yaml"
            ]),
            {
                "use_sim_time": use_sim_time,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution([
                FindPackageShare("rh_moveit"),
                "config", "mobile_base", "ros_controllers.yaml"
            ]),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Base controller spawner
    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_controller"],
        output="screen",
    )

    # RViz
    rh_moveit_path = FindPackageShare("rh_moveit")
    rviz_config_file = PathJoinSubstitution(
        [rh_moveit_path, "config", "mobile_base", rviz_config]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Static transform publisher (for odom -> world if needed)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "odom"],
    )

    # Joint state publisher GUI (conditional)
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_joint_state_gui)
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            base_controller_spawner,
            move_group_node,
            static_tf,
            joint_state_publisher,
            rviz_node,
        ]
    ) 