import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Set the log level for nodes (debug, info, warn, error, fatal)'
    )
    log_level = LaunchConfiguration('log_level')

    # Define the path to the xacro file
    urdf_file = os.path.join(
        get_package_share_directory('rh_moveit'), 'config', 'realhex.urdf.xacro'
    )

    # Use xacro to process the URDF file
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])

    # Node for Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Node for Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Node for RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(get_package_share_directory('rh_moveit'), 'config', 'moveit.rviz'),
            '--ros-args', '--log-level', log_level
        ]
    )

    return LaunchDescription([
        log_level_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]) 