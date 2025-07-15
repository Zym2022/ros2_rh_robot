#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from ocs2_msgs.msg import MpcTargetTrajectories, MpcState, MpcInput
from ocs2_msgs.srv import Reset, GenerateTraj
from tf2_ros.buffer import Buffer
from tf2_ros import TransformStamped
from tf2_ros.transform_listener import TransformListener
from .utils import generate_traj

class GoalManagerNode(Node):
    """
    任务管理节点
    - 提供 /generate_trajectory 服务来接收高层目标并生成轨迹
    - 监听TF获取末端执行器位姿
    - 发布MPC目标轨迹 (/mobile_manipulator_mpc_target)
    - 调用MPC重置服务 (/mobile_manipulator_mpc_reset)
    """
    def __init__(self):
        super().__init__('goal_manager_node')

        # ============= 发布器 =============
        self.target_traj_pub = self.create_publisher(
            MpcTargetTrajectories, '/mobile_manipulator_mpc_target', 10)

        # ============= 状态变量 =============
        self.arm_pose = None
        self.start_time = self.get_clock().now()
        self.reset_retry_timer = None

        # ============= 服务客户端 =============
        self.reset_client = self.create_client(Reset, '/mobile_manipulator_mpc_reset')

        # ============= 服务服务器 =============
        self.generate_traj_server = self.create_service(
            GenerateTraj, 'generate_trajectory', self.traj_callback)

        # ============= 初始化TF =============
        self.init_tf()

        # ============= 初始化 =============
        self.get_logger().info('等待MPC服务启动...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MPC重置服务不可用，继续等待...')
        
        # 启动一个定时器，该定时器会一直尝试发送重置请求，直到成功
        self.reset_retry_timer = self.create_timer(0.5, self.send_reset_request)
        
        self.get_logger().info('任务管理节点已启动')

    def get_elapsed_time(self):
        current_time = self.get_clock().now()
        return (current_time - self.start_time).nanoseconds * 1e-9

    def init_tf(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.source_frame = 'world'
        self.target_frame = 'gripper_tip_link'
        self.tf_timer = self.create_timer(0.01, self.get_end_effect_transform)

    def get_end_effect_transform(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, Time(), Duration(nanoseconds=100000000))
            
            first_tf_success = self.arm_pose is None
            
            self.arm_pose = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
            
            if first_tf_success:
                self.get_logger().info(f'成功获取TF数据，末端位姿: {self.arm_pose}')
                
        except Exception as e:
            elapsed_time = self.get_elapsed_time()
            if elapsed_time > 10.0:
                self.get_logger().warn(
                    f"TF transform not available after {elapsed_time:.1f}s: {e}")

    def send_reset_request(self):
        # 持续检查TF数据是否就绪
        if self.arm_pose is None:
            self.get_logger().info('等待TF数据可用以发送MPC重置请求...')
            return
        
        # TF数据已可用，取消重试定时器并发送请求
        if self.reset_retry_timer is not None:
            self.reset_retry_timer.cancel()
            self.reset_retry_timer = None
            self.get_logger().info('TF数据已可用，发送MPC重置请求...')

        req = Reset.Request()
        req.reset = True
        
        mpc_state = MpcState(value=self.arm_pose)
        mpc_input = MpcInput(value=[0.0] * 9)
        target_traj = MpcTargetTrajectories(
            time_trajectory=[0.0],
            state_trajectory=[mpc_state],
            input_trajectory=[mpc_input]
        )
        req.target_trajectories = target_traj
        
        future = self.reset_client.call_async(req)
        future.add_done_callback(self.reset_response_callback)
    
    def reset_response_callback(self, future):
        try:
            response = future.result()
            if response.done:
                self.get_logger().info('MPC重置成功')
            else:
                self.get_logger().error('MPC重置失败')
        except Exception as e:
            self.get_logger().error(f'MPC重置服务调用失败: {e}')
            
    def traj_callback(self, request, response):
        try:
            if self.arm_pose is None:
                self.get_logger().error('TF数据尚未可用，无法生成轨迹')
                response.done = False
                return response
            
            time_now = self.get_elapsed_time()
            
            time_trajectory, state_trajectory = generate_traj(
                self.arm_pose, 
                request.goal_pose, 
                time_now, 
                time_now + request.time, 
                request.timestep
            )
            
            self.get_logger().info(f'生成目标位姿轨迹: {request.goal_pose}')
            
            target_trajectories = MpcTargetTrajectories()
            target_trajectories.time_trajectory = [float(t) for t in time_trajectory]
            target_trajectories.state_trajectory = [MpcState(value=s) for s in state_trajectory]
            target_trajectories.input_trajectory = [
                MpcInput(value=[0.0] * 9) for _ in time_trajectory
            ]
            
            self.target_traj_pub.publish(target_trajectories)
            self.get_logger().info('已发布目标轨迹')
            
            response.done = True
            return response
            
        except Exception as e:
            self.get_logger().error(f'生成轨迹失败: {e}')
            response.done = False
            return response

def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 