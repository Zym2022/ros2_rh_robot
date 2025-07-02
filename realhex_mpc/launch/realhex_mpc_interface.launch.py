import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    # 启动RealHex MPC接口节点 (整合版本 - 包含状态发布和控制接口功能)
    mpc_interface_node = Node(
        package='realhex_mpc',
        executable='realhex_mpc_interface',
        name='realhex_mpc_interface',
        parameters=[
            {'realman_control_freq': 200.0,
             'hexmove_control_freq': 20.0,
             'mpc_freq': 20.0,
             'is_sim': True}
        ],
        output='screen',
    )
    
    # 创建启动描述
    return LaunchDescription([
        mpc_interface_node
    ]) 