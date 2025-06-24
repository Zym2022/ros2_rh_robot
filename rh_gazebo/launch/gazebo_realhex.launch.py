import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

import xacro
import yaml

def generate_launch_description():
    package_name = 'rh_gazebo'
    robot_name_in_model = 'realhex_description'


    # DECLARE Gazebo WORLD:
    # world_name = LaunchConfiguration('world_name')
    # declare_world_cmd = DeclareLaunchArgument(
    #     'world_name',
    #     default_value='momanip.world',
    #     description='Gazebo world name')
    # # DECLARE Gazebo WORLD file:
    # gazebo_world_path = (
    #     get_package_share_directory('rh_gazebo'),
    #     '/config/',
    #     world_name
    # )
    # world_name = LaunchConfiguration('world_name')
    # declare_world_cmd = DeclareLaunchArgument(
    #     'world_name',
    #     default_value='empty.world',
    #     description='Gazebo world name')
    # # DECLARE Gazebo WORLD file:
    # gazebo_world_path = (
    #     get_package_share_directory('rh_gazebo'),
    #     '/world/',
    #     world_name
    # )

    # DECLARE URDF file:
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(
        pkg_share, f'config/gazebo_realhex_description.urdf.xacro')
    

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}


    # 启动gazebo
    gazebo =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True},
                    params, {"publish_frequency": 15.0}],
        output='screen'
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', f'{robot_name_in_model}'],
                        output='screen')

    # gazebo在加载urdf时，根据urdf的设定，会启动一个joint_states节点
    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 路径执行控制器，也就是那个action
    # 这个rm_group_controller需要根据urdf文件里面引用的ros2_controllers.yaml里面的名字确定
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_velocity_controller'],
        output='screen'
    )

    # 用下面这两个估计是想控制好各个节点的启动顺序
    # 监听 spawn_entity_cmd，当其退出（完全启动）时，启动load_joint_state_controller
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
    )
    # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )

    # 添加静态TF发布器，发布base_footprint到base_link的变换
    static_tf_publisher_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 添加自定义TF广播器，从odom到base_footprint的变换中获取数据
    # 并以系统时间发布world到base_link的变换
    custom_tf_broadcaster = Node(
        package='rh_gazebo',
        executable='custom_tf_broadcaster',
        name='custom_tf_broadcaster',
        output='screen',
        parameters=[
            {'source_frame': 'odom'},
            {'target_frame': 'base_footprint'},
            {'new_source_frame': 'world'},
            {'new_target_frame': 'mobile_base'},
            {'publish_frequency': 100.0},
            {'wait_timeout': 30.0},  # 等待30秒，足够Gazebo启动
            {'use_sim_time': True}
        ]
    )

    # 添加 RViz 配置
    rviz_config_path = '/home/zhuyiming/robot_ws/src/ros2_mobile/ros2_rh_robot/rh_gazebo/config/test.rviz'
    
    # 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    ld = LaunchDescription([
        # declare_world_cmd,
        gazebo,
        close_evt1,
        close_evt2,
        node_robot_state_publisher,
        spawn_entity,
        # static_tf_publisher_footprint,  # 添加静态TF发布器: base_footprint -> base_link
        custom_tf_broadcaster,  # 添加自定义TF广播器: odom -> base_link
        # rviz_node,  # 添加 RViz 节点
    ]
    )

    return ld
