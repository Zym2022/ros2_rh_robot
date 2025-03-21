#! /usr/bin/env python3

import roboticstoolbox as rtb
import spatialmath as sm
import qpsolvers as qp
import numpy as np
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import tf2_ros
from tf2_ros import TransformBroadcaster


def step_robot(r: rtb.ERobot, Tep):
    """计算机器人的关节速度以达到目标位姿"""
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

class RobotState:
    """机器人状态类，用于存储和更新机器人的状态信息"""
    def __init__(self):
        # 关节状态
        self.joint_positions = {}
        self.arm_joints = [0.0] * 7  # 7个机械臂关节
        
        # 底盘状态
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw = 0.0
        
        # 状态标志
        self.joint_state_updated = False
        self.odom_updated = False
        
        # 数据锁
        self.lock = threading.Lock()
    
    def update_joint_state(self, msg):
        """更新关节状态"""
        with self.lock:
            # 创建一个字典，将关节名称映射到它们的位置
            for i, name in enumerate(msg.name):
                self.joint_positions[name] = msg.position[i]
            
            # 提取机械臂关节
            for i, joint_name in enumerate(['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']):
                if joint_name in self.joint_positions:
                    self.arm_joints[i] = self.joint_positions[joint_name]
            
            self.joint_state_updated = True
    
    def update_odom(self, msg):
        """更新里程计状态"""
        with self.lock:
            # 更新底盘位置
            self.base_x = msg.pose.pose.position.x
            self.base_y = msg.pose.pose.position.y
            
            # 从四元数中提取偏航角
            orientation_q = msg.pose.pose.orientation
            _, _, self.base_yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
            
            self.odom_updated = True
    
    def get_state(self):
        """获取当前状态的副本"""
        with self.lock:
            return {
                'arm_joints': self.arm_joints.copy(),
                'base_x': self.base_x,
                'base_y': self.base_y,
                'base_yaw': self.base_yaw,
                'joint_state_updated': self.joint_state_updated,
                'odom_updated': self.odom_updated
            }


class RealHexController(Node):
    def __init__(self):
        super().__init__('realhex_controller')

        self.count = 0
        
        # 创建机器人状态对象
        self.robot_state = RobotState()
        
        # 创建发布器
        self.joint_pos_publisher = self.create_publisher(
            Float64MultiArray, 
            '/joint_position_controller/commands', 
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )
        
        # 创建订阅器
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.robot_state.update_joint_state,
            10
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.robot_state.update_odom,
            10
        )
        
        # 导入RealHex机器人模型
        from rh_control.realhex import RealHex
        
        # 创建RealHex机器人实例
        self.realhex = RealHex()
        self.get_logger().info(f'机器人关节数量: {self.realhex.n}')
        
        # 初始化机器人位置和目标
        self.realhex.q = self.realhex.qr
        self.base_transform = self.realhex._T.copy()
        self.arrived = False
        
        # 设置目标位姿
        self.wTep = self.realhex.fkine(self.realhex.q) * sm.SE3.Rz(np.pi)
        self.wTep.A[:3, :3] = np.diag([-1, 1, -1])
        self.wTep.A[0, -1] -= 3.0  # 向x轴负方向移动3米
        self.wTep.A[2, -1] -= 0.5  # 向下移动0.5米
        
        # 输出目标位姿
        target_pos = self.wTep.A[:3, 3]
        self.get_logger().info(f'目标位姿: 位置=[{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]')
        
        # 用于控制位姿输出频率的计数器
        self.pose_output_counter = 0
        self.pose_output_interval = 20  # 每20个周期输出一次位姿（约1秒）
        
        # 创建控制循环定时器
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # 存储当前关节位置
        self.current_joint_positions = [0.0] * (self.realhex.n - 2)
        
        self.get_logger().info('RealHex控制器已初始化')
        
        # 添加 tf 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
    
    def update_robot_state(self):
        """从RobotState更新机器人模型状态"""
        state = self.robot_state.get_state()
        
        if not (state['joint_state_updated'] and state['odom_updated']):
            return False
        
        # 更新机械臂关节角度
        self.realhex.q[2:] = state['arm_joints']
        
        # 更新底盘位置和方向
        self.base_transform[:2, 3] = [state['base_x'], state['base_y']]
        
        # 更新底盘方向（假设底盘只绕z轴旋转）
        cos_yaw = np.cos(state['base_yaw'])
        sin_yaw = np.sin(state['base_yaw'])
        self.base_transform[:2, :2] = [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]]
        
        # 更新机器人基座变换
        self.realhex._T = self.base_transform
        
        # 确保底盘关节角度为0（因为底盘位置由里程计直接提供）
        self.realhex.q[:2] = 0
        
        return True
    
    def test_vel_ctl(self):
        self.count += 1
        if (self.count // 200) % 2 == 0:
            self.realhex.qd[1] = 0.3
        else:
            self.realhex.qd[1] = -0.3

        self.realhex.qd[0] = 0
        self.realhex.qd[2:] = 0
    
    def control_loop(self):
        """主控制循环"""
        # 更新机器人状态
        # if not self.update_robot_state():
        #     self.get_logger().info('等待关节状态和里程计数据...')
        #     return
        
        # 获取当前关节位置
        # state = self.robot_state.get_state()
        # self.current_joint_positions = state['arm_joints']
        
        # 如果尚未到达目标，计算控制命令
        if not self.arrived:
            # 计算关节速度
            self.arrived, self.realhex.qd = step_robot(self.realhex, self.wTep.A)

            self.test_vel_ctl()

            dt = 0.05
            self.realhex.q = self.realhex.q + self.realhex.qd * dt
            
            # 重置底盘位置
            base_new = self.realhex.fkine(self.realhex._q, end=self.realhex.links[2])
            self.realhex._T = base_new.A
            
            # 重置底盘关节角度为0
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
                print(self.realhex.qd)
        elif self.arrived and self.pose_output_counter != -1:
            # 如果刚刚到达目标，输出一次到达信息
            self.pose_output_counter = -1
            self.get_logger().info('已到达目标位姿!')
        
        # 发布控制命令
        self.publish_cmd_vel()
        self.publish_joint_positions()
        self.publish_base_tf()
    
    def publish_cmd_vel(self):
        """发布底盘速度命令"""
        # 创建Twist消息
        cmd_vel = Twist()
        
        # 设置线速度和角速度
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
    
    def publish_joint_positions(self):
        """发布关节位置命令"""
        # 创建Float64MultiArray消息
        msg = Float64MultiArray()
        
        # 计算时间步长
        dt = 0.05  # 与定时器间隔一致
        
        # 计算新的关节位置 = 当前位置 + 速度 * 时间
        if not self.arrived:
            # 获取机械臂关节速度 (realhex.qd[2:]表示跳过底盘的两个自由度)
            new_positions = list(self.realhex.q[2:])
            
            msg.data = new_positions
        else:
            # 如果已到达目标，保持当前位置
            msg.data = list(self.realhex.q[2:])
        
        # 发布消息
        self.joint_pos_publisher.publish(msg)
        
        # 记录日志
        if self.pose_output_counter == 0:
            self.get_logger().debug(f'发布关节位置命令: {msg.data}')
    
    def publish_base_tf(self):
        """发布基座坐标变换"""
        t = TransformStamped()
        
        # 设置时间戳和坐标系
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'cal_odom'
        
        # 从变换矩阵中提取平移
        t.transform.translation.x = self.realhex._T[0, 3]
        t.transform.translation.y = self.realhex._T[1, 3]
        t.transform.translation.z = self.realhex._T[2, 3]
        
        # 从旋转矩阵计算四元数
        # 提取旋转矩阵
        R = self.realhex._T[:3, :3]
        
        # 计算四元数的中间变量
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                w = (R[2, 1] - R[1, 2]) / S
                x = 0.25 * S
                y = (R[0, 1] + R[1, 0]) / S
                z = (R[0, 2] + R[2, 0]) / S
            elif R[1, 1] > R[2, 2]:
                S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                w = (R[0, 2] - R[2, 0]) / S
                x = (R[0, 1] + R[1, 0]) / S
                y = 0.25 * S
                z = (R[1, 2] + R[2, 1]) / S
            else:
                S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                w = (R[1, 0] - R[0, 1]) / S
                x = (R[0, 2] + R[2, 0]) / S
                y = (R[1, 2] + R[2, 1]) / S
                z = 0.25 * S
        
        # 设置四元数
        t.transform.rotation.w = w
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        
        # 发布变换
        self.tf_broadcaster.sendTransform(t)


def main():
    """ROS2节点的入口函数"""
    rclpy.init()
    
    # 创建并运行节点
    controller = RealHexController()
    
    try:
        # 保持节点运行
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        controller.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == "__main__":
    main()
