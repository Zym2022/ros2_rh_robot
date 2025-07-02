#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ocs2_msgs.msg import MpcFlattenedController, MpcObservation, MpcTargetTrajectories, MpcState, MpcInput
from ocs2_msgs.srv import Reset, GenerateTraj
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rm_ros_interfaces.msg import Jointpos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import threading
import numpy as np
import math
from transforms3d.euler import quat2euler
from .utils import interpolate_trajectory, find_nearest_timestamp_index, generate_traj, transform_odom_to_state


class RealHexMpcInterface(Node):
    """
    RealHex MPC接口节点 - 整合版本
    
    功能1: MPC控制指令发布
    订阅：
    - /mobile_manipulator_mpc_policy: MPC优化控制策略
    
    发布：
    - /cmd_vel: 底盘速度控制指令
    - /joint_velocity_controller/commands: 关节速度控制指令(仿真)
    - /rm_driver/movej_canfd_cmd: 关节位置控制指令(实物)
    - /mobile_manipulator_mpc_target: 目标轨迹
    
    功能2: MPC状态发布
    订阅：
    - /odom: 里程计数据，获取底盘位置和姿态
    - /joint_states: 关节状态，获取机械臂关节角度
    
    发布：
    - /mobile_manipulator_mpc_observation: MPC观测消息
    
    服务：
    - /generate_trajectory: 生成目标轨迹服务
    """
    
    def __init__(self):
        super().__init__('realhex_mpc_interface')
        
        # ============= 参数配置 =============
        self.declare_parameter('mpc_freq', 20.0)  
        self.mpc_freq = self.get_parameter('mpc_freq').get_parameter_value().double_value
        self.declare_parameter('is_sim', True)
        self.is_sim = self.get_parameter('is_sim').value
        self.declare_parameter('realman_control_freq', 200.0)
        self.realman_control_freq = self.get_parameter('realman_control_freq').get_parameter_value().double_value
        self.declare_parameter('hexmove_control_freq', 20.0)
        self.hexmove_control_freq = self.get_parameter('hexmove_control_freq').get_parameter_value().double_value
        
        # ============= MPC控制相关订阅器 =============
        # 订阅MPC策略
        self.mpc_policy_sub = self.create_subscription(
            MpcFlattenedController,
            '/mobile_manipulator_mpc_policy',
            self.mpc_policy_callback,
            10
        )
        
        # ============= MPC状态相关订阅器 =============
        # 订阅底盘里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 订阅关节状态
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # ============= 发布器 =============
        # 底盘速度发布器
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 仿真关节速度发布器
        self.joint_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocity_controller/commands',
            10
        )

        # 实物关节位置发布器
        self.joint_pos_pub = self.create_publisher(
            Jointpos, 
            '/rm_driver/movej_canfd_cmd', 
            1
        )
        
        # 目标轨迹发布器
        self.target_traj_pub = self.create_publisher(
            MpcTargetTrajectories,
            '/mobile_manipulator_mpc_target',
            10
        )
        
        # MPC观测消息发布器
        self.mpc_observation_pub = self.create_publisher(
            MpcObservation, 
            '/mobile_manipulator_mpc_observation', 
            10
        )

        # ============= 状态变量 =============
        # MPC策略相关
        self.mpc_policy = MpcFlattenedController()
        # TODO 参考real-time chucking inpainting 或者 Temporal ensembling方式去执行策略的切换部分
        # https://pi.website/research/real_time_chunking
        self.old_mpc_policy = MpcFlattenedController()
        self.mpc_policy_lock = threading.Lock()
        
        # 底盘状态相关
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.prev_theta = 0.0  # 前一个theta值来跟踪角度变化
        self.cumulative_theta = 0.0  # 累积的theta值
        self.first_odom = True  # 标记是否是第一次收到里程计数据
        
        # 关节状态相关
        self.joint_positions = [0.0] * 7  # 7个关节
        self.current_joint = [0.0] * 7    # 当前关节位置(用于实物控制)
        self.current_input = [0.0] * 9    # 2个底盘 + 7个关节的输入
        
        # 末端位姿
        # self.arm_pose = [0.25, 0.0, 0.67, -0.881603, 0.0226216, -0.4702807, 0.0331615]  # 默认位姿
        self.arm_pose = [0.0, 0.0, 1.32, 0.0, 0.0, 0.0, 1.0]  # 默认位姿
        
        # 互斥锁保护共享数据
        self.state_lock = threading.Lock()
        
        # ============= 定时器 =============
        # 控制定时器
        self.realman_control_timer = self.create_timer(1.0/self.realman_control_freq, self.realman_control_timer_callback)
        self.hexmove_control_timer = self.create_timer(1.0/self.hexmove_control_freq, self.hexmove_control_timer_callback)
        
        # MPC状态发布定时器
        self.mpc_timer = self.create_timer(1.0/self.mpc_freq, self.mpc_timer_callback)
        
        # 记录开始时间
        self.start_time = self.get_clock().now()
        
        # ============= 服务客户端 =============
        # 创建MPC重置服务客户端
        self.reset_client = self.create_client(Reset, '/mobile_manipulator_mpc_reset')
        
        # ============= 服务服务器 =============
        # 创建轨迹生成服务
        self.generate_traj_server = self.create_service(
            GenerateTraj, 
            'generate_trajectory', 
            self.traj_callback
        )
        
        # ============= 初始化 =============
        # 等待MPC启动并发送初始重置
        self.get_logger().info('等待MPC服务启动...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MPC重置服务不可用，继续等待...')
        
        self.send_reset_request()
        
        self.get_logger().info('RealHex MPC接口节点已启动（整合版本）')
    
    def get_elapsed_time(self):
        """获取从节点启动到现在的时间，单位为秒"""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds * 1e-9
        return float(elapsed_time)
    
    # ============= MPC控制相关回调函数 =============
    def send_reset_request(self):
        """发送MPC重置请求"""
        req = Reset.Request()
        req.reset = True
        
        # 创建一个空的目标轨迹（使用当前状态）
        # mpc_state = MpcState(value=[0.25, 0.0, 0.67, -0.881603, 0.0226216, -0.4702807, 0.0331615])  # 3 position + 4 quaternion
        mpc_state = MpcState(value=[0.0, 0.0, 1.32, 0.0, 0.0, 0.0, 1.0])  # 3 position + 4 quaternion
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
    
    def realman_control_timer_callback(self):
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
                    # 对小速度进行死区处理
                    if abs(control_input[0]) < 0.05:
                        control_input[0] = 0.0
                    if abs(control_input[1]) < 0.05:
                        control_input[1] = 0.0
                    # 更新当前输入（用于状态发布）
                    with self.state_lock:
                        self.current_input = control_input

                    if self.is_sim:
                        # 发布关节速度命令
                        # 假设控制输入的后7个元素是关节速度
                        joint_vel = Float64MultiArray()
                        joint_vel.data = [float(val) for val in control_input[2:9]]  # 提取关节速度
                        self.joint_vel_pub.publish(joint_vel)
                        
                        self.get_logger().debug(
                            f'发布控制命令: 关节速度={joint_vel.data}'
                        )
                    else:
                        # 发布关节位置命令

                        
                        dt = 1.0 / self.realman_control_freq
                        joint_pos = Jointpos()
                        joint_pos.dof = 7
                        joint_pos.expand = 0.0
                        joint_pos.follow = True
                        joint_pos.joint = [
                            self.current_joint[i] + control_input[i+2] * dt
                            for i in range(len(self.current_joint))
                        ]
                        # TODO 可以试试直接发布state_trajectory
                        desired_joint_pos = interpolate_trajectory(
                            self.mpc_policy.time_trajectory,
                            [state_msg.value for state_msg in self.mpc_policy.state_trajectory],
                            current_time
                        )
                        joint_pos.joint = [float(val) for val in desired_joint_pos[3:10]]
                        
                        self.joint_pos_pub.publish(joint_pos)

                        self.get_logger().debug(
                            f'发布控制命令: 关节位置={joint_pos.joint}'
                        )
    
    def hexmove_control_timer_callback(self):
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
                    # 对小速度进行死区处理
                    if abs(control_input[0]) < 0.05:
                        control_input[0] = 0.0
                    if abs(control_input[1]) < 0.05:
                        control_input[1] = 0.0
                    # 更新当前输入（用于状态发布）
                    with self.state_lock:
                        self.current_input = control_input
                    
                    # 发布底盘速度命令
                    cmd_vel = Twist()
                    cmd_vel.linear.x = float(control_input[0])  # 线速度
                    cmd_vel.angular.z = float(control_input[1])  # 角速度
                    self.cmd_vel_pub.publish(cmd_vel)
                    
                    
    
    # ============= MPC状态相关回调函数 =============
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
            _, _, raw_theta = quat2euler(quat)
            
            # 处理角度的连续性
            if self.first_odom:
                self.cumulative_theta = raw_theta
                self.prev_theta = raw_theta
                self.first_odom = False
            else:
                # 计算角度差，考虑角度环绕
                delta_theta = raw_theta - self.prev_theta
                
                # 处理角度跳变（当角度差超过π时）
                if delta_theta > math.pi:
                    delta_theta -= 2 * math.pi
                elif delta_theta < -math.pi:
                    delta_theta += 2 * math.pi
                
                # 更新累积角度
                self.cumulative_theta += delta_theta
                self.prev_theta = raw_theta
            
            # 使用累积的角度作为朝向
            self.odom_theta = self.cumulative_theta
    
    def joint_states_callback(self, msg):
        """处理关节状态消息"""
        with self.state_lock:
            # 过滤出机械臂的7个关节角度
            # 这里假设机械臂关节名称为'joint1' ~ 'joint7'
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
            
            # 确保关节名称存在于消息中
            for joint_name in joint_names:
                if joint_name in msg.name:
                    index = msg.name.index(joint_name)
                    joint_index = int(joint_name[5:]) - 1  # 从'joint1'提取索引1并减1得到0
                    if 0 <= joint_index < len(self.joint_positions):
                        self.joint_positions[joint_index] = msg.position[index]
                        self.current_joint[joint_index] = msg.position[index]  # 同时更新当前关节位置
    
    def mpc_timer_callback(self):
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
    
    # ============= 服务回调函数 =============
    def traj_callback(self, request, response):
        """处理轨迹生成服务请求"""
        try:
            goal_pose = request.goal_pose
            time_duration = request.time
            timestep = request.timestep
            time_now = self.get_elapsed_time()
            
            # 生成轨迹
            time_trajectory, state_trajectory = generate_traj(
                self.arm_pose, 
                goal_pose, 
                time_now, 
                time_now + time_duration, 
                timestep
            )
            
            self.get_logger().info(f'生成目标位姿轨迹: {goal_pose}')
            
            # 创建目标轨迹消息
            target_trajectories = MpcTargetTrajectories()
            target_trajectories.time_trajectory = [float(i) for i in time_trajectory]
            target_trajectories.state_trajectory = [MpcState(value=i) for i in state_trajectory]
            
            # 为每个时间点创建零输入
            input_size = 9  # 2个底盘速度 + 7个关节速度
            target_trajectories.input_trajectory = [
                MpcInput(value=[0.0] * input_size) for _ in time_trajectory
            ]
            
            # 发布目标轨迹
            self.target_traj_pub.publish(target_trajectories)
            self.get_logger().info('已发布目标轨迹')
            
            # 更新当前末端位姿为目标位姿（假设轨迹会被执行）
            self.arm_pose = goal_pose
            
            response.done = True
            return response
            
        except Exception as e:
            self.get_logger().error(f'生成轨迹失败: {e}')
            response.done = False
            return response


def main(args=None):
    rclpy.init(args=args)
    node = RealHexMpcInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 