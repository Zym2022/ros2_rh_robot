#! /usr/bin/env python3

import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import qpsolvers as qp
import numpy as np
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler


def step_robot(r: rtb.ERobot, Tep):
    # 获取当前末端执行器位姿
    wTe = r.fkine(r.q)

    # 计算相对误差
    eTep = np.linalg.inv(wTe) @ Tep

    # 空间误差
    et = np.sum(np.abs(eTep[:3, -1]))
    
    # 控制增益
    Y = 0.01

    # 目标函数的二次项
    Q = np.eye(r.n + 6)

    # 关节速度分量
    Q[: r.n, : r.n] *= Y
    Q[:2, :2] *= 1.0 / et

    # 松弛变量分量
    Q[r.n :, r.n :] = (1.0 / et) * np.eye(6)

    # 计算期望的笛卡尔速度
    v, _ = rtb.p_servo(wTe, Tep, 1.5)
    v[3:] *= 1.3

    # 等式约束（雅可比约束）
    Aeq = np.c_[r.jacobe(r.q), np.eye(6)]
    beq = v.reshape((6,))

    # 关节限位避免的不等式约束
    Ain = np.zeros((r.n + 6, r.n + 6))
    bin = np.zeros(r.n + 6)

    # 关节限位参数
    ps = 0.1  # 最小允许角度（弧度）
    pi = 0.9  # 速度阻尼器生效角度（弧度）

    # 构建关节限位速度阻尼器
    Ain[: r.n, : r.n], bin[: r.n] = r.joint_velocity_damper(ps, pi, r.n)

    # 目标函数的线性项：操作度雅可比
    c = np.concatenate(
        (np.zeros(2), -r.jacobm(start=r.links[4]).reshape((r.n - 2,)), np.zeros(6))
    )

    # 控制底盘朝向末端执行器
    kε = 0.5
    bTe = r.fkine(r.q, include_base=False).A
    θε = math.atan2(bTe[1, -1], bTe[0, -1])
    ε = kε * θε
    c[0] = -ε

    # 关节速度和松弛变量的上下界
    lb = -np.r_[r.qdlim[: r.n], 10 * np.ones(6)]
    ub = np.r_[r.qdlim[: r.n], 10 * np.ones(6)]

    # 求解关节速度
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver="osqp")
    qd = qd[: r.n]

    # 根据误差调整速度
    if et > 0.5:
        qd *= 0.7 / et
    else:
        qd *= 1.4

    # 判断是否到达目标
    if et < 0.02:
        return True, qd
    else:
        return False, qd


class RealHexNode(Node):
    def __init__(self):
        super().__init__('realhex_node')
        
        # 创建数据锁
        self.state_lock = threading.Lock()
        
        # 创建关节速度命令发布器
        self.joint_vel_publisher = self.create_publisher(
            Float64MultiArray, 
            '/joint_velocity_controller/commands', 
            1
        )
        
        # 创建底盘速度命令发布器
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )
        
        # 创建关节状态订阅器
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 创建里程计订阅器
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 导入必要的模块（在这里导入以避免全局导入问题）
        import roboticstoolbox as rtb
        import spatialgeometry as sg
        import spatialmath as sm
        import qpsolvers as qp
        from rh_control.realhex import RealHex
        
        # 保存模块引用以供后续使用
        self.rtb = rtb
        self.sm = sm
        self.qp = qp
        
        # 创建RealHex机器人实例
        self.realhex = RealHex()
        self.get_logger().info(f'机器人关节数量: {self.realhex.n}')
        
        # 初始化机器人位置和目标
        self.realhex.q = self.realhex.qr
        self.base_new = self.realhex._T
        self.arrived = False
        
        # 初始化底盘位置和方向
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw = 0.0
        
        # 标记是否已收到关节状态和里程计数据
        self.joint_state_received = False
        self.odom_received = False
        
        # 设置目标位姿
        self.wTep = self.realhex.fkine(self.realhex.q) * sm.SE3.Rz(np.pi)
        self.wTep.A[:3, :3] = np.diag([-1, 1, -1])
        self.wTep.A[0, -1] -= 4.0  # 向x轴负方向移动4米
        self.wTep.A[2, -1] -= 0.5  # 向下移动0.3米
        
        # 输出目标位姿（只输出一次）
        target_pos = self.wTep.A[:3, 3]
        self.get_logger().info(f'目标位姿: 位置=[{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]')
        
        # 用于控制位姿输出频率的计数器
        self.pose_output_counter = 0
        self.pose_output_interval = 20  # 每20个周期输出一次位姿（约1秒）
        
        # 创建定时器，以0.05秒的间隔更新关节角度和发布命令
        self.timer = self.create_timer(0.05, self.update_and_publish)
        
        self.get_logger().info('RealHex节点已初始化')
    
    def joint_state_callback(self, msg):
        """处理关节状态消息"""
        # 创建一个字典，将关节名称映射到它们的位置
        joint_positions = {}
        for i, name in enumerate(msg.name):
            joint_positions[name] = msg.position[i]
        
        # 只提取我们需要的关节(joint1-joint7)，按照正确的顺序
        arm_joints = []
        for joint_name in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']:
            if joint_name in joint_positions:
                arm_joints.append(joint_positions[joint_name])
            else:
                self.get_logger().warn(f'关节 {joint_name} 不在消息中')
                return  # 如果缺少任何所需关节，则退出
        
        # 确保我们有所有7个关节角度
        if len(arm_joints) == 7:
            # 获取锁，确保数据一致性
            with self.state_lock:
                # 保持底盘关节角度不变，更新机械臂关节
                self.realhex.q[2:] = arm_joints
                self.joint_state_received = True
            self.get_logger().debug(f'已更新关节状态: {arm_joints}')
        else:
            self.get_logger().warn(f'关节数量不正确: 期望7个，实际{len(arm_joints)}个')
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        # 更新底盘位置
        self.base_x = msg.pose.pose.position.x
        self.base_y = msg.pose.pose.position.y
        
        # 从四元数中提取偏航角
        orientation_q = msg.pose.pose.orientation
        _, _, self.base_yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])

        # 获取锁，确保数据一致性
        with self.state_lock:
            # 更新机器人底盘位置
            self.base_new[:2, 3] = [self.base_x, self.base_y]
            
            # 更新底盘方向（假设底盘只绕z轴旋转）
            cos_yaw = np.cos(self.base_yaw)
            sin_yaw = np.sin(self.base_yaw)
            self.base_new[:2, :2] = [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]]
            
            self.odom_received = True
        
        self.get_logger().debug(f'已更新底盘位置: x={self.base_x:.2f}, y={self.base_y:.2f}, yaw={self.base_yaw:.2f}')
    
    def update_and_publish(self):
        """计算控制命令并发布"""
        # 如果尚未收到关节状态和里程计数据，则等待
        if not (self.joint_state_received and self.odom_received):
            self.get_logger().info('等待关节状态和里程计数据...')
            return
        
        self.realhex._T = self.base_new
        
        # 获取锁，确保在计算过程中数据不会被更新
        with self.state_lock:
            # 如果尚未到达目标，计算控制命令
            if not self.arrived:
                # 计算关节速度
                self.arrived, self.realhex.qd = step_robot(self.realhex, self.wTep.A)

                self.realhex._T = self.base_new

                self.realhex.q[:2] = 0
                
                # 以较低频率输出当前末端位姿
                self.pose_output_counter += 1
                if self.pose_output_counter >= self.pose_output_interval:
                    self.pose_output_counter = 0
                    
                    # 获取当前末端位姿
                    current_ee_pose = self.realhex.fkine(self.realhex.q)
                    current_pos = current_ee_pose.A[:3, 3]
                    
                    # 输出当前位姿和是否到达目标
                    self.get_logger().info(f'当前末端位姿: 位置=[{current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}], 是否到达: {self.arrived}')
            elif self.arrived and self.pose_output_counter != -1:
                # 如果刚刚到达目标，输出一次到达信息
                self.pose_output_counter = -1
                self.get_logger().info('已到达目标位姿!')
            else:
                pass
        
        # 发布底盘速度命令和关节速度命令
        self.publish_cmd_vel()
        self.publish_joint_velocities()
    
    def publish_cmd_vel(self):
        """发布底盘速度命令"""
        # 创建Twist消息
        cmd_vel = Twist()
        
        # 设置线速度和角速度
        # 底盘的速度是realhex.qd的前两个元素
        # 第一个元素通常是线速度，第二个元素是角速度
        if not self.arrived:
            cmd_vel.linear.x = self.realhex.qd[1]  # 前后移动
            cmd_vel.angular.z = self.realhex.qd[0]  # 旋转
        else:
            # 如果已到达目标，停止移动
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        
        # 发布消息
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # 记录日志
        if self.pose_output_counter == 0:
            self.get_logger().debug(f'发布底盘速度命令: 线速度={cmd_vel.linear.x:.3f}, 角速度={cmd_vel.angular.z:.3f}')
    
    def publish_joint_velocities(self):
        """发布关节速度命令"""
        # 创建Float64MultiArray消息
        msg = Float64MultiArray()
        
        # 设置数据为机械臂的关节速度 (realhex.qd[2:]表示跳过底盘的两个自由度)
        if not self.arrived:
            msg.data = self.realhex.qd[2:].tolist()
        else:
            # 如果已到达目标，停止关节移动
            msg.data = [0.0] * (self.realhex.n - 2)
        
        # 发布消息
        self.joint_vel_publisher.publish(msg)
        
        # 记录日志
        if self.pose_output_counter == 0:
            self.get_logger().info(f'发布关节速度命令: {msg.data}')

def main():
    """ROS2节点的入口函数"""
    rclpy.init()
    
    # 创建并运行节点
    node = RealHexNode()
    
    try:
        # 保持节点运行
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()
    
    return 0

if __name__ == "__main__":
    main()