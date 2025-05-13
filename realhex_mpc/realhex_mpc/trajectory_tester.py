#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ocs2_msgs.srv import GenerateTraj
import time
import numpy as np
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import math


class TrajectoryTester(Node):
    """
    RealHex机械臂轨迹测试节点
    
    该节点循环执行四个预定义位姿的轨迹跟踪控制，
    通过监听末端Link7到world的TF来判断轨迹是否执行完成
    """
    
    def __init__(self):
        super().__init__('trajectory_tester')
        
        # 创建轨迹生成服务客户端
        self.traj_client = self.create_client(
            GenerateTraj, 
            '/generate_trajectory'
        )
        
        # 初始化TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 等待服务可用
        self.get_logger().info('等待轨迹生成服务...')
        while not self.traj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('轨迹生成服务不可用，继续等待...')
            
        self.get_logger().info('轨迹生成服务已连接')
        
        # 预定义的四个目标位姿
        self.goal_poses = [
            [-2.0, 2.0, 0.5, 0.0, -1.0, 0.0, 0.0],   # 左上角
            [-2.0, -2.0, 0.5, 0.0, -1.0, 0.0, 0.0],  # 左下角
            [0.0, -0.0, 0.5, 0.0, -1.0, 0.0, 0.0],    # 右下角
            [2.0, 2.0, 0.5, 0.0, -1.0, 0.0, 0.0]   # 右上角
        ]
        
        # 当前位姿索引
        self.current_pose_index = 0
        
        # 是否正在执行轨迹
        self.is_executing = False
        
        # 目标点位置容差（单位：米）
        self.position_tolerance = 0.02
        
        # 执行时间记录
        self.trajectory_start_time = None
        self.trajectory_times = []
        
        # 创建定时器，定期检查是否需要发送新的轨迹
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 创建定时器，定期检查末端位姿
        self.check_pose_timer = self.create_timer(0.1, self.check_pose_timer_callback)
        
        self.get_logger().info('轨迹测试节点已启动')
    
    def get_end_effector_pose(self):
        """获取末端执行器位姿 (Link7 到 world 的TF变换)"""
        try:
            # 查询Link7到world的变换
            trans = self.tf_buffer.lookup_transform(
                'world',
                'Link7',  # 可能需要根据实际情况调整末端连杆名称
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 提取位置和姿态
            position = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ]
            
            orientation = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ]
            
            return position + orientation
        
        except Exception as e:
            self.get_logger().debug(f'获取TF失败: {e}')
            return None
    
    def check_pose_timer_callback(self):
        """定期检查末端位姿，判断是否到达目标"""
        if self.is_executing:
            end_effector_pose = self.get_end_effector_pose()
            
            if end_effector_pose is not None:
                self.check_goal_reached(end_effector_pose)
    
    def check_goal_reached(self, end_effector_pose):
        """检查是否到达目标位置"""
        current_goal = self.goal_poses[self.current_pose_index]
        
        # 计算当前位置与目标位置的距离
        current_position = end_effector_pose[:3]
        goal_position = current_goal[:3]
        distance = np.linalg.norm(np.array(current_position) - np.array(goal_position))
        
        # 打印调试信息
        self.get_logger().debug(f'当前位置: {current_position}, 目标位置: {goal_position}, 距离: {distance}')
        
        # 如果距离小于容差，认为到达目标位置
        if distance < self.position_tolerance:
            # 计算并记录执行时间
            if self.trajectory_start_time is not None:
                execution_time = time.time() - self.trajectory_start_time
                self.trajectory_times.append(execution_time)
                self.get_logger().info(f'轨迹执行完成，用时: {execution_time:.2f}秒')
                
                # 打印所有执行时间记录
                if len(self.trajectory_times) > 1:
                    avg_time = sum(self.trajectory_times) / len(self.trajectory_times)
                    self.get_logger().info(f'平均执行时间: {avg_time:.2f}秒')
                    self.get_logger().info(f'执行时间记录: {[f"{t:.2f}" for t in self.trajectory_times]}秒')
            
            # 更新到下一个位姿
            self.current_pose_index = (self.current_pose_index + 1) % len(self.goal_poses)
            self.get_logger().info(f'已到达目标位置，准备下一个位姿')
            
            # 标记当前轨迹已执行完成
            self.is_executing = False
    
    def timer_callback(self):
        """定时器回调函数，检查并发送轨迹请求"""
        if not self.is_executing:
            self.send_trajectory_request()
    
    def send_trajectory_request(self):
        """发送轨迹生成请求"""
        self.is_executing = True
        
        # 创建请求
        request = GenerateTraj.Request()
        request.goal_pose = self.goal_poses[self.current_pose_index]
        request.time = 10.0  # 轨迹执行时间为10秒
        request.timestep = 2.0  # 轨迹时间步长
        
        goal_position = request.goal_pose[:3]
        self.get_logger().info(f'发送轨迹请求到位置: {goal_position}')
        
        # 记录开始时间
        self.trajectory_start_time = time.time()
        
        # 发送请求
        future = self.traj_client.call_async(request)
        future.add_done_callback(self.trajectory_response_callback)
    
    def trajectory_response_callback(self, future):
        """处理轨迹生成响应"""
        try:
            response = future.result()
            
            if response.done:
                self.get_logger().info(f'轨迹生成成功，执行中...')
            else:
                self.get_logger().error('轨迹生成失败')
                self.is_executing = False
                
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {e}')
            self.is_executing = False


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 