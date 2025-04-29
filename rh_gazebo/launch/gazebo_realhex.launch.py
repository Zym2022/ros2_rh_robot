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
    world_name = LaunchConfiguration('world_name')
    declare_world_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='momanip.world',
        description='Gazebo world name')
    # DECLARE Gazebo WORLD file:
    gazebo_world_path = (
        get_package_share_directory('rh_gazebo'),
        '/config/',
        world_name
    )

    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': gazebo_world_path}.items(),
    )

    # DECLARE URDF file:
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(
        pkg_share, f'config/gazebo_realhex_description.urdf.xacro')
    
    print("path---", urdf_model_path)

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    print("urdf---", doc.toxml())

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': False},
                    params, {"publish_frequency": 15.0}],
        output='screen'
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', f'{robot_name_in_model}'],
                        output='screen')

    # Load controllers
    delay_seconds = 10
    load_controllers = []
    for controller in [
            "joint_state_broadcaster", 
            "rm_gripper_controller", 
            "rm_group_controller", 
            "joint_velocity_controller", 
            "joint_position_controller"]:
        cmd = ["ros2", "run", "controller_manager", "spawner.py", controller]
        delayed_cmd = ["sleep {}; {}".format(delay_seconds, " ".join(cmd))]
        load_controllers += [
            ExecuteProcess(
                cmd=delayed_cmd,
                shell=True,
                output="screen",
            )
        ]

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
    # 并以系统时间发布odom到base_link的变换
    custom_tf_broadcaster = Node(
        package='rh_gazebo',
        executable='custom_tf_broadcaster',
        name='custom_tf_broadcaster',
        output='screen',
        parameters=[
            {'source_frame': 'odom'},
            {'target_frame': 'base_footprint'},
            {'broadcast_frame': 'base_link'},
            {'publish_frequency': 100.0},
            {'wait_timeout': 30.0},  # 等待30秒，足够Gazebo启动
            {'use_sim_time': False}
        ]
    )

    # 添加 RViz 配置
    rviz_config_path = '/home/zhuyiming/robot_ws/src/ros2_rh_robot/rh_gazebo/config/test.rviz'
    
    # 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    ld = LaunchDescription([
        declare_world_cmd,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        # static_tf_publisher_footprint,  # 添加静态TF发布器: base_footprint -> base_link
        custom_tf_broadcaster,  # 添加自定义TF广播器: odom -> base_link
        # rviz_node,  # 添加 RViz 节点
    ]
        + load_controllers
    )

    return ld
