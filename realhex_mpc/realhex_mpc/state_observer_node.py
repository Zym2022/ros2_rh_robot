#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from ocs2_msgs.msg import MpcObservation, MpcState, MpcInput
from std_msgs.msg import Float64MultiArray
import threading
import math
from transforms3d.euler import quat2euler
from .utils import transform_odom_to_state

class StateObserverNode(Node):
    """
    状态观测节点
    - 订阅底盘里程计 (/odom) 和关节状态 (/joint_states)
    - 订阅策略执行器发布的当前控制输入 (/mpc_applied_input)
    - 组合成MPC观测消息并发布 (/mobile_manipulator_mpc_observation)
    """
    def __init__(self):
        super().__init__('state_observer_node')

        # ============= 参数配置 =============
        self.declare_parameter('mpc_freq', 20.0)
        self.mpc_freq = self.get_parameter('mpc_freq').get_parameter_value().double_value

        # ============= 订阅器 =============
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.applied_input_sub = self.create_subscription(
            Float64MultiArray, '/mpc_applied_input', self.applied_input_callback, 10)

        # ============= 发布器 =============
        self.mpc_observation_pub = self.create_publisher(
            MpcObservation, '/mobile_manipulator_mpc_observation', 10)

        # ============= 状态变量 =============
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.prev_theta = 0.0
        self.cumulative_theta = 0.0
        self.first_odom = True
        self.joint_positions = [0.0] * 7
        self.current_input = [0.0] * 9  # 2 base + 7 arm

        self.state_lock = threading.Lock()
        self.start_time = self.get_clock().now()

        # ============= 定时器 =============
        self.observation_timer = self.create_timer(
            1.0 / self.mpc_freq, self.publish_observation_callback)

        self.get_logger().info('状态观测节点已启动')

    def get_elapsed_time(self):
        current_time = self.get_clock().now()
        return (current_time - self.start_time).nanoseconds * 1e-9

    def odom_callback(self, msg):
        with self.state_lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            quat = [
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ]
            _, _, raw_theta = quat2euler(quat)
            if self.first_odom:
                self.cumulative_theta = raw_theta
                self.prev_theta = raw_theta
                self.first_odom = False
            else:
                delta_theta = raw_theta - self.prev_theta
                if delta_theta > math.pi:
                    delta_theta -= 2 * math.pi
                elif delta_theta < -math.pi:
                    delta_theta += 2 * math.pi
                self.cumulative_theta += delta_theta
                self.prev_theta = raw_theta
            self.odom_theta = self.cumulative_theta

    def joint_states_callback(self, msg):
        with self.state_lock:
            joint_names = [f'joint{i+1}' for i in range(7)]
            for joint_name in joint_names:
                if joint_name in msg.name:
                    index = msg.name.index(joint_name)
                    joint_index = int(joint_name[5:]) - 1
                    if 0 <= joint_index < len(self.joint_positions):
                        self.joint_positions[joint_index] = msg.position[index]
    
    def applied_input_callback(self, msg):
        with self.state_lock:
            if len(msg.data) == 9:
                self.current_input = list(msg.data)

    def publish_observation_callback(self):
        with self.state_lock:
            state_vector = transform_odom_to_state(
                self.odom_x,
                self.odom_y,
                self.odom_theta,
                self.joint_positions
            )
            observation = MpcObservation()
            observation.time = self.get_elapsed_time()
            observation.state = MpcState(value=state_vector)
            observation.input = MpcInput(value=self.current_input)
            self.mpc_observation_pub.publish(observation)
            self.get_logger().debug(f'发布MPC观测: 时间={observation.time:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = StateObserverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 