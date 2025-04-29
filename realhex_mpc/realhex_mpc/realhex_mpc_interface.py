#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ocs2_msgs.msg import MpcFlattenedController, MpcObservation, MpcTargetTrajectories, MpcState, MpcInput
from ocs2_msgs.srv import Reset
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import threading
import numpy as np
from .utils import interpolate_trajectory, find_nearest_timestamp_index


class RealHexMpcInterface(Node):
    """
    RealHex MPC接口节点
    
    订阅：
    - /mobile_manipulator_mpc_policy: MPC优化控制策略
    
    发布：
    - /cmd_vel: 底盘速度控制指令
    - /joint_velocity_controller/commands: 关节速度控制指令
    """
    
    def __init__(self):
        super().__init__('realhex_mpc_interface')
        
        # 参数配置
        self.declare_parameter('control_freq', 100.0)
        self.control_freq = self.get_parameter('control_freq').value
        
        # 订阅MPC策略
        self.mpc_policy_sub = self.create_subscription(
            MpcFlattenedController,
            '/mobile_manipulator_mpc_policy',
            self.mpc_policy_callback,
            10
        )
        
        # 创建底盘速度和关节速度发布器
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.joint_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocity_controller/commands',
            10
        )
        
        # 初始化MPC策略
        self.mpc_policy = MpcFlattenedController()
        self.mpc_policy_lock = threading.Lock()
        
        # 启动控制定时器
        self.control_timer = self.create_timer(1.0/self.control_freq, self.control_timer_callback)
        
        # 记录开始时间
        self.start_time = self.get_clock().now()
        
        # 创建MPC重置服务客户端
        self.reset_client = self.create_client(Reset, '/mobile_manipulator_mpc_reset')
        
        # 等待MPC启动并发送初始重置
        self.get_logger().info('等待MPC服务启动...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MPC重置服务不可用，继续等待...')
        
        self.send_reset_request()
        
        self.get_logger().info('RealHex MPC接口节点已启动')
    
    def get_elapsed_time(self):
        """获取从节点启动到现在的时间，单位为秒"""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds * 1e-9
        return float(elapsed_time)
    
    def send_reset_request(self):
        """发送MPC重置请求"""
        req = Reset.Request()
        req.reset = True
        
        # 创建一个空的目标轨迹（使用当前状态）
        mpc_state = MpcState(value=[0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 1.0])  # 3 position + 4 quaternion
        mpc_input = MpcInput(value=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 2个底盘 + 7个关节速度
        target_traj = MpcTargetTrajectories(
            time_trajectory=[0.0],
            state_trajectory=[mpc_state],
            input_trajectory=[mpc_input]
        )
        req.target_trajectories = target_traj
        
        # 发送请求
        future = self.reset_client.call_async(req)
        future.add_done_callback(self.reset_response_callback)
    
    def reset_response_callback(self, future):
        """处理重置响应"""
        try:
            response = future.result()
            if response.done:
                self.get_logger().info('MPC重置成功')
            else:
                self.get_logger().error('MPC重置失败')
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {e}')
    
    def mpc_policy_callback(self, msg):
        """处理MPC策略消息"""
        with self.mpc_policy_lock:
            self.mpc_policy = msg
            self.get_logger().debug(f'收到新的MPC策略, 时间轨迹起点: {msg.time_trajectory[0]}')
    
    def control_timer_callback(self):
        """根据当前MPC策略计算并发布控制指令"""
        current_time = self.get_elapsed_time()
        
        with self.mpc_policy_lock:
            if len(self.mpc_policy.time_trajectory) > 0:
                # 使用线性插值计算当前时间对应的控制输入
                control_input = interpolate_trajectory(
                    self.mpc_policy.time_trajectory,
                    [input_msg.value for input_msg in self.mpc_policy.input_trajectory],
                    current_time
                )
                
                if control_input:
                    # 发布底盘速度命令
                    # 假设控制输入的前两个元素分别是线速度和角速度
                    cmd_vel = Twist()
                    cmd_vel.linear.x = float(control_input[0])  # 线速度
                    cmd_vel.angular.z = float(control_input[1])  # 角速度
                    self.cmd_vel_pub.publish(cmd_vel)
                    
                    # 发布关节速度命令
                    # 假设控制输入的后7个元素是关节速度
                    joint_vel = Float64MultiArray()
                    joint_vel.data = [float(val) for val in control_input[2:9]]  # 提取关节速度
                    self.joint_vel_pub.publish(joint_vel)
                    
                    self.get_logger().debug(
                        f'发布控制命令: 线速度={cmd_vel.linear.x}, 角速度={cmd_vel.angular.z}, '
                        f'关节速度={joint_vel.data}'
                    )


def main(args=None):
    rclpy.init(args=args)
    node = RealHexMpcInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 