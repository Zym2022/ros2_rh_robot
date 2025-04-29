import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    # 包含Gazebo仿真启动文件，并传入参数禁用其内部的RViz
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rh_gazebo'),
                'launch',
                'gazebo_realhex.launch.py'
            )
        ]),
        launch_arguments={
            'rviz': 'false'  # 禁用Gazebo启动文件中的RViz
        }.items()
    )
    
        
    # 创建启动描述
    return LaunchDescription([
        # 启动节点
        gazebo_launch,
    ]) 