#! /usr/bin/env python3

import os
import numpy as np
from roboticstoolbox.robot.Robot import Robot
from roboticstoolbox.robot.Link import Link
from roboticstoolbox.robot.ETS import ETS
from roboticstoolbox.robot.ET import ET
from spatialmath import SE3
from ament_index_python.packages import get_package_share_directory

class RealHex(Robot):
    """
    Class that imports a RealHex robot model combining RM65 arm with Echo Plus base

    ``RealHex()`` 是一个将RM65机械臂和Echo Plus移动底盘组合在一起的机器人类。y
    该模型描述了机器人的运动学和图形特征。

    定义的关节配置包括：
    - qz: 零位配置
    - qr: 准备位置配置
    """

    def __init__(self):
        # 读取Echo Plus底盘的URDF
        # base_urdf_path = os.path.join(
        #     get_package_share_directory('xpkg_urdf_echo_plus'), f'/urdf/model.urdf')
        
        # arm_urdf_path = os.path.join(
        #     get_package_share_directory('rm_description'),
        #     '/urdf/RM75_6F/rm_75_6f_description.urdf'
        # )
        base_urdf_path = '/home/zhuyiming/robot_ws/src/ros2_mobile/urdf/xpkg_urdf_echo_plus/urdf/model.urdf'
        arm_urdf_path = '/home/zhuyiming/robot_ws/src/ros2_rm_robot/rm_description/urdf/rm_75_6f_description.urdf'

        links_base, _, urdf_string_base, urdf_filepath_base = self.URDF_read(base_urdf_path)

        # 读取RM65机械臂的URDF
        links_arm, _, urdf_string_arm, _ = self.URDF_read(arm_urdf_path)

        # 重命名底盘的链接，避免名称冲突
        for link in links_base:
            link.name = "base_" + link.name

        # 重命名机械臂的链接
        for link in links_arm:
            link.name = "arm_" + link.name

        # 只保留实际运动关节对应的链接
        active_links = []
        
        # 添加底盘的基座链接
        base_link = links_base[0]  # base_base_link
        base_link.jindex = None  # 基座没有关节
        active_links.append(base_link)
        
        # 添加底盘的运动学链接（旋转和平移）
        base_rot = Link(
            ETS(ET.Rz()),
            name="base_rotation",
            parent=base_link,
            jindex=0,  # 第一个关节
            qlim=np.array([-np.pi, np.pi])  # 旋转限位 ±180度
        )
        
        base_trans = Link(
            ETS(ET.tx()),
            name="base_translation",
            parent=base_rot,
            jindex=1,  # 第二个关节
            qlim=np.array([-10, 10])  # 平移限位 ±10米
        )
        
        active_links.extend([base_rot, base_trans])
        
        # 创建连接底盘和机械臂的中间链接
        base_arm = Link(
            ETS(ET.tz(0.36) * ET.tx(0.02171)),
            name="base_arm_connector",
            parent=base_trans,
            jindex=None,  # 固定连接，无关节
            qlim=np.array([0, 0])  # 固定连接，无运动
        )
        
        active_links.append(base_arm)
        
        # 添加机械臂的所有链接
        arm_links = [link for link in links_arm if link.name in [
            'arm_base_link',
            'arm_Link1',
            'arm_Link2',
            'arm_Link3',
            'arm_Link4',
            'arm_Link5',
            'arm_Link6',
            'arm_Link7'  # 添加Link7，因为URDF模型中已经包含了这个链接
        ]]
        
        # 设置机械臂链接的父子关系和关节索引
        arm_links[0]._parent = base_arm  # 将机械臂的基座连接到base_arm
        arm_links[0].jindex = None  # 基座没有关节
        
        for i in range(1, len(arm_links)):
            arm_links[i]._parent = arm_links[i-1]  # 设置机械臂链接之间的父子关系
            arm_links[i].jindex = i + 1  # 设置关节索引，从2开始（因为0,1被底盘使用）
        
        # 创建末端执行器链接
        end_effector = Link(
            ETS(),  # 沿z轴正方向平移0.1725米
            name="arm_end_effector",
            parent=arm_links[-1],  # 父链接是Link7
            jindex=None,  # 固定链接，无关节
            qlim=np.array([0, 0])  # 固定链接，无运动
        )
        
        arm_links.append(end_effector)
        active_links.extend(arm_links)

        # 创建机器人模型时使用活动链接
        super().__init__(
            active_links,
            name="RealHex",
            manufacturer="Custom",
            urdf_string=urdf_string_base,
            urdf_filepath=urdf_filepath_base,
        )

        print("Active links:", [link.name for link in active_links])  # 调试用
        # print("Number of joints:", len(self.joints))  # 打印实际关节数量

        # EHCO-PLUS底盘最大速度2m/s,最大旋转速度1rad/s
        self.qdlim = np.array([
            4.0,  # 底盘旋转限位
            4.0,  # 底盘平移限位
            np.pi * 178/180,  # 关节1限位 J1 ±178° 
            np.pi * 130/180,  # 关节2限位 J2 ±130° 
            np.pi * 135/180,  # 关节3限位 J3 ±135°
            np.pi * 178/180,  # 关节4限位 J4 ±178°
            np.pi * 128/180,  # 关节5限位 J5 ±128°
            np.pi * 360/180,  # 关节6限位 J6 ±360°
            np.pi * 360/180,  # 关节7限位 J7 ±360°
        ])

        # 定义一些预设的机器人构型
        self.qr = np.array([0, 0,           # 底盘旋转和平移
                           0, 0.1, 0, 0, 0.1, 0, 0])  # RM65七个关节
        self.qz = np.zeros(9)  # 9个自由度：2个底盘 + 7个机械臂

        # 添加构型到机器人配置中
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

# def main():
#     # 初始化ROS2
#     rclpy.init()

#     # 创建节点
#     node = Node("realhex_node")
#     node.get_logger().info("realhex_node start")

#     # 创建RealHex机器人实例
#     robot = RealHex()
#     node.get_logger().info(f'机器人关节数量: {robot.n}')
#     node.get_logger().info(f'机器人模型: {robot}')

#     try:
#         # 保持节点运行
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # 清理资源
#         node.destroy_node()
#         rclpy.shutdown()
    
#     return 0
    

if __name__ == "__main__":  # pragma nocover
    robot = RealHex()
    # 打印实际关节数量以进行调试
    print("Number of joints:", robot.n)
    print(robot)

