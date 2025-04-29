import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    # 声明参数
    use_mpc = LaunchConfiguration('use_mpc')
    declare_use_mpc = DeclareLaunchArgument(
        'use_mpc',
        default_value='true',
        description='启用MPC控制器'
    )
    
    mpc_task_file = LaunchConfiguration('mpc_task_file')
    declare_mpc_task_file = DeclareLaunchArgument(
        'mpc_task_file',
        default_value=os.path.join(
            get_package_share_directory('ocs2_mobile_manipulator'),
            'config/realhex/task.info'
        ),
        description='MPC任务配置文件路径'
    )
    
    mpc_urdf_file = LaunchConfiguration('mpc_urdf_file')
    declare_mpc_urdf_file = DeclareLaunchArgument(
        'mpc_urdf_file',
        default_value=os.path.join(
            get_package_share_directory('ocs2_robotic_assets'),
            'resources/mobile_manipulator/realhex/urdf/realhex.urdf'
        ),
        description='机器人URDF文件路径'
    )
    
    mpc_lib_folder = LaunchConfiguration('mpc_lib_folder')
    declare_mpc_lib_folder = DeclareLaunchArgument(
        'mpc_lib_folder',
        default_value=os.path.join(
            get_package_share_directory('ocs2_mobile_manipulator'),
            'auto_generated/realhex'
        ),
        description='自动生成的MPC库文件夹'
    )
    
    # 声明启用可视化
    rviz = LaunchConfiguration('rviz')
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='启用RViz可视化'
    )
    
    # 仅启动OCS2 MPC控制器节点，不启动dummy_mrt节点
    mpc_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_mpc_node',
        name='mobile_manipulator_mpc',
        parameters=[
            {'taskFile': mpc_task_file},
            {'urdfFile': mpc_urdf_file},
            {'libFolder': mpc_lib_folder}
        ],
        output='screen',
        condition=IfCondition(use_mpc)
    )
    
    # 启动RViz（可选，由rviz参数控制）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('ocs2_mobile_manipulator_ros'),
            'rviz/mobile_manipulator.rviz'
        )],
        output='screen',
        condition=IfCondition(rviz)
    )
    
    # 启动目标发布节点
    target_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_target',
        name='mobile_manipulator_target',
        parameters=[
            {'taskFile': mpc_task_file},
            {'urdfFile': mpc_urdf_file},
            {'libFolder': mpc_lib_folder}
        ],
        output='screen',
        condition=IfCondition(use_mpc)
    )
    
    # 创建启动描述
    return LaunchDescription([
        # 声明参数
        declare_use_mpc,
        declare_mpc_task_file,
        declare_mpc_urdf_file,
        declare_mpc_lib_folder,
        declare_rviz,
        
        rviz_node,
        mpc_node,
        target_node,
    ]) 