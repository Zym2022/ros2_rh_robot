import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    # 启动RealHex MPC状态发布节点
    state_publisher_node = Node(
        package='realhex_mpc',
        executable='realhex_mpc_state_publisher',
        name='realhex_mpc_state_publisher',
        parameters=[
            {'mpc_freq': 100.0}
        ],
        output='screen',
    )
    
    # 启动RealHex MPC接口节点
    mpc_interface_node = Node(
        package='realhex_mpc',
        executable='realhex_mpc_interface',
        name='realhex_mpc_interface',
        parameters=[
            {'control_freq': 100.0}
        ],
        output='screen',
    )
    
    # 创建启动描述
    return LaunchDescription([
        state_publisher_node,
        mpc_interface_node
    ]) 