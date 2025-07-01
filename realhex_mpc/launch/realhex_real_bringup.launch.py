import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ================================================
# 启动 realman 和 hexman 的实物节点以及rviz
# ================================================

def generate_launch_description():
    # =============hexman============================
    vehicle_ini_path = PathJoinSubstitution(
        [FindPackageShare("xpkg_vehicle"), "ini", "device_id_list.ini"])

    xnode_comm = Node(name="xnode_comm",
                      package="xpkg_comm",
                      executable="xnode_comm",
                      output="screen",
                      parameters=[{
                          "dev_list": False,
                          "com_enable": True,
                          "com_channel_common": False,
                          "com_channel_xstd": True,
                      }])

    xnode_vehicle = Node(name="xnode_vehicle",
                         package="xpkg_vehicle",
                         executable="xnode_vehicle",
                         output="screen",
                         parameters=[{
                             "ini_path": vehicle_ini_path,
                             "show_path": True,
                             "show_loc": False,
                             "calc_speed": False,
                             "mode_can_lock": False,
                             "rate_x": 1.0,
                             "rate_y": 1.0,
                             "rate_z": 1.0,
                             "rate_az": 1.0,
                         }])
    # ================================================


    # =============realman============================
    rm_75_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_driver')), 'launch', 'rm_75_driver.launch.py'))
    )
    # ================================================


    realman_xacro_file = os.path.join(get_package_share_directory('ocs2_robotic_assets'), 'resources/mobile_manipulator/realhex/urdf/realhex.urdf')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', realman_xacro_file])
    realhex_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        respawn=True,
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    # ================================================

    # =============rviz===============================
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("realhex_mpc"), "config",
        "real.rviz"
    ])

    rviz_node = Node(name="rviz2",
                     package="rviz2",
                     executable="rviz2",
                     arguments=["-d", rviz_config_path])
    # ================================================

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
            {'wait_timeout': 30.0},  # 等待30秒
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        xnode_comm,
        xnode_vehicle,
        rm_75_driver,
        realhex_description,
        rviz_node,
        custom_tf_broadcaster
    ])




