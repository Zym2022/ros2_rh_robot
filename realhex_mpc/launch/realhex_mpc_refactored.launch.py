import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    启动重构后的MPC接口节点：
    - state_observer_node: 状态观测节点
    - policy_executor_node: 策略执行节点
    - goal_manager_node: 任务管理节点
    """

    # 可以通过参数文件来配置节点参数
    # config = os.path.join(
    #     get_package_share_directory('realhex_mpc'),
    #     'config',
    #     'mpc_params.yaml'
    # )

    state_observer_node = Node(
        package='realhex_mpc',
        executable='state_observer_node',
        name='state_observer_node',
        output='screen',
        # parameters=[config]
    )

    policy_executor_node = Node(
        package='realhex_mpc',
        executable='policy_executor_node',
        name='policy_executor_node',
        output='screen',
        # parameters=[config]
    )

    goal_manager_node = Node(
        package='realhex_mpc',
        executable='goal_manager_node',
        name='goal_manager_node',
        output='screen',
        # parameters=[config]
    )

    return LaunchDescription([
        state_observer_node,
        policy_executor_node,
        goal_manager_node
    ]) 