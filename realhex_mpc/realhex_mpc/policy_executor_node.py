#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ocs2_msgs.msg import MpcFlattenedController
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rm_ros_interfaces.msg import Jointpos
from sensor_msgs.msg import JointState
import threading
from .utils import interpolate_trajectory

class PolicyExecutorNode(Node):
    """
    策略执行节点
    - 订阅MPC策略 (/mobile_manipulator_mpc_policy)
    - 根据策略插值计算控制指令
    - 发布底盘速度 (/cmd_vel) 和关节控制指令
    - 发布当前应用的控制输入 (/mpc_applied_input) 以供状态观测器使用
    """
    def __init__(self):
        super().__init__('policy_executor_node')

        # ============= 参数配置 =============
        self.declare_parameter('is_sim', True)
        self.is_sim = self.get_parameter('is_sim').value
        self.declare_parameter('realman_control_freq', 200.0)
        self.realman_control_freq = self.get_parameter('realman_control_freq').get_parameter_value().double_value
        self.declare_parameter('hexmove_control_freq', 20.0)
        self.hexmove_control_freq = self.get_parameter('hexmove_control_freq').get_parameter_value().double_value

        # ============= 订阅器 =============
        self.mpc_policy_sub = self.create_subscription(
            MpcFlattenedController,
            '/mobile_manipulator_mpc_policy',
            self.mpc_policy_callback,
            10
        )
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # ============= 发布器 =============
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_vel_pub = self.create_publisher(
            Float64MultiArray, '/joint_velocity_controller/commands', 10)
        self.joint_pos_pub = self.create_publisher(
            Jointpos, '/rm_driver/movej_canfd_cmd', 1)
        self.applied_input_pub = self.create_publisher(
            Float64MultiArray, '/mpc_applied_input', 10)

        # ============= 状态变量 =============
        self.mpc_policy = MpcFlattenedController()
        self.mpc_policy_lock = threading.Lock()
        self.current_joint = [0.0] * 7
        self.start_time = self.get_clock().now()

        # ============= 定时器 =============
        self.realman_control_timer = self.create_timer(
            1.0 / self.realman_control_freq, self.realman_control_timer_callback)
        self.hexmove_control_timer = self.create_timer(
            1.0 / self.hexmove_control_freq, self.hexmove_control_timer_callback)

        self.get_logger().info('策略执行节点已启动')

    def get_elapsed_time(self):
        current_time = self.get_clock().now()
        return (current_time - self.start_time).nanoseconds * 1e-9

    def mpc_policy_callback(self, msg):
        with self.mpc_policy_lock:
            self.mpc_policy = msg
            self.get_logger().debug(f'收到新的MPC策略, 时间轨迹起点: {msg.time_trajectory[0]}')
            
    def joint_states_callback(self, msg):
        # 实物控制需要当前的关节角度
        if not self.is_sim:
            joint_names = [f'joint{i+1}' for i in range(7)]
            for joint_name in joint_names:
                if joint_name in msg.name:
                    index = msg.name.index(joint_name)
                    joint_index = int(joint_name[5:]) - 1
                    if 0 <= joint_index < len(self.current_joint):
                        self.current_joint[joint_index] = msg.position[index]

    def realman_control_timer_callback(self):
        current_time = self.get_elapsed_time()
        with self.mpc_policy_lock:
            if not self.mpc_policy.time_trajectory:
                return

            control_input = interpolate_trajectory(
                self.mpc_policy.time_trajectory,
                [input_msg.value for input_msg in self.mpc_policy.input_trajectory],
                current_time
            )
            
            if not control_input:
                return

            # 发布应用的控制输入供观测节点使用
            applied_input_msg = Float64MultiArray(data=[float(v) for v in control_input])
            self.applied_input_pub.publish(applied_input_msg)

            if self.is_sim:
                joint_vel = Float64MultiArray()
                joint_vel.data = [float(val) for val in control_input[2:9]]
                self.joint_vel_pub.publish(joint_vel)
            else:
                # 在实物控制中, realman_control_timer_callback 里的关节位置控制被注释掉了
                # 原始代码使用了插值的 state_trajectory, 这里也保持一致
                desired_joint_pos_list = interpolate_trajectory(
                    self.mpc_policy.time_trajectory,
                    [state_msg.value for state_msg in self.mpc_policy.state_trajectory],
                    current_time
                )
                if desired_joint_pos_list:
                    joint_pos = Jointpos()
                    joint_pos.dof = 7
                    joint_pos.expand = 0.0
                    joint_pos.follow = True
                    # 原始状态从 3 到 10 是关节位置
                    joint_pos.joint = [float(val) for val in desired_joint_pos_list[3:10]]
                    self.joint_pos_pub.publish(joint_pos)
    
    def hexmove_control_timer_callback(self):
        current_time = self.get_elapsed_time()
        with self.mpc_policy_lock:
            if not self.mpc_policy.time_trajectory:
                return

            control_input = interpolate_trajectory(
                self.mpc_policy.time_trajectory,
                [input_msg.value for input_msg in self.mpc_policy.input_trajectory],
                current_time
            )
            
            if not control_input:
                return
            
            # 对小速度进行死区处理
            if abs(control_input[0]) < 0.05: control_input[0] = 0.0
            if abs(control_input[1]) < 0.05: control_input[1] = 0.0
            
            cmd_vel = Twist()
            cmd_vel.linear.x = float(control_input[0])
            cmd_vel.angular.z = float(control_input[1])
            self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 