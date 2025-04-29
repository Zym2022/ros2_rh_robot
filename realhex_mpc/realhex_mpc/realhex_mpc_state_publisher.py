#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ocs2_msgs.msg import MpcObservation, MpcState, MpcInput
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from transforms3d.euler import quat2euler
import threading
from .utils import transform_odom_to_state
import numpy as np

class RealHexMpcStatePublisher(Node):
    """
    RealHex MPC状态发布节点
    
    订阅：
    - /odom: 里程计数据，获取底盘位置和姿态
    - /joint_states: 关节状态，获取机械臂关节角度
    
    发布：
    - /mobile_manipulator_mpc_observation: MPC观测消息
    """
    
    def __init__(self):
        super().__init__('realhex_mpc_state_publisher')
        
        # 参数配置
        self.declare_parameter('mpc_freq', 100.0)
        self.mpc_freq = self.get_parameter('mpc_freq').value
        
        # 创建MPC观测消息发布器
        self.mpc_observation_pub = self.create_publisher(
            MpcObservation, 
            '/mobile_manipulator_mpc_observation', 
            10
        )
        
        # 创建底盘里程计订阅器
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 创建关节状态订阅器
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # 初始化状态变量
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.joint_positions = [0.0] * 7  # 假设有7个关节
        self.current_input = [0.0] * 9    # 2个底盘 + 7个关节的输入
        
        # 互斥锁保护共享数据
        self.state_lock = threading.Lock()
        
        # 启动定时器，定期发布MPC观测消息
        self.timer = self.create_timer(1.0/self.mpc_freq, self.timer_callback)
        
        # 记录开始时间
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('RealHex MPC状态发布节点已启动')
    
    def get_elapsed_time(self):
        """获取从节点启动到现在的时间，单位为秒"""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds * 1e-9
        return float(elapsed_time)
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        with self.state_lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            
            # 从四元数转换为欧拉角
            quat = [
                msg.pose.pose.orientation.w,  # 注意：transforms3d使用w,x,y,z顺序
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ]
            _, _, self.odom_theta = quat2euler(quat)
    
    def joint_states_callback(self, msg):
        """处理关节状态消息"""
        with self.state_lock:
            # 过滤出机械臂的7个关节角度
            # 这里假设机械臂关节名称为'joint1' ~ 'joint7'
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
            
            # self.get_logger().info(f'关节状态消息: {msg}')

            # 确保关节名称存在于消息中
            for joint_name in joint_names:
                if joint_name in msg.name:
                    index = msg.name.index(joint_name)
                    joint_index = int(joint_name[5:]) - 1  # 从'joint1'提取索引1并减1得到0
                    if 0 <= joint_index < len(self.joint_positions):
                        self.joint_positions[joint_index] = msg.position[index]
                    # self.get_logger().info(f'关节 {joint_name} 状态: {self.joint_positions[joint_index]}')
    
    def timer_callback(self):
        """定时发布MPC观测消息"""
        with self.state_lock:
            # 创建状态向量
            state_vector = transform_odom_to_state(
                self.odom_x, 
                self.odom_y, 
                self.odom_theta, 
                self.joint_positions
            )
            
            # 创建MPC观测消息
            observation = MpcObservation()
            observation.time = self.get_elapsed_time()
            
            # 设置状态
            state = MpcState()
            state.value = state_vector
            observation.state = state
            
            # 设置输入（当前控制信号）
            input_msg = MpcInput()
            input_msg.value = self.current_input
            observation.input = input_msg
            
            # 发布观测消息
            self.mpc_observation_pub.publish(observation)
            
            # 日志记录
            self.get_logger().debug(f'发布MPC观测: 时间={observation.time}, 状态={state_vector}')


def main(args=None):
    rclpy.init(args=args)
    node = RealHexMpcStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 