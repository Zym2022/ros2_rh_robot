import numpy as np
import bisect
from typing import List, Tuple

def generate_traj(start_pose, goal_pose, start_time, goal_time, timestep):
    """
    在起始位姿和目标位姿之间生成平滑轨迹
    
    Args:
        start_pose: 起始位姿 [x, y, z, qx, qy, qz, qw]
        goal_pose: 目标位姿 [x, y, z, qx, qy, qz, qw]
        start_time: 轨迹起始时间
        goal_time: 轨迹结束时间
        timestep: 时间步长
        
    Returns:
        time_trajectory: 时间轨迹
        state_trajectory: 状态轨迹
    """
    time_trajectory = np.arange(start_time, goal_time, timestep)
    time_trajectory = np.append(time_trajectory, goal_time)
    num_steps = len(time_trajectory)
    
    # 分离起始和目标位姿的平移部分和旋转四元数部分
    start_translation = start_pose[:3]
    start_quaternion = start_pose[3:]
    goal_translation = goal_pose[:3]
    goal_quaternion = goal_pose[3:]
    
    # 插值平移部分
    translation_trajectory = np.zeros((num_steps, 3))
    for i in range(3):
        translation_trajectory[:, i] = np.linspace(start_translation[i], goal_translation[i], num_steps)
    
    # 插值四元数部分
    quaternion_trajectory = np.zeros((num_steps, 4))
    for t in range(num_steps):
        frac = t / (num_steps - 1) if num_steps > 1 else 1.0
        quaternion_trajectory[t] = np.array(start_quaternion) * (1 - frac) + np.array(goal_quaternion) * frac
        quaternion_trajectory[t] /= np.linalg.norm(quaternion_trajectory[t])
    
    # 组合轨迹
    state_trajectory = np.hstack((translation_trajectory, quaternion_trajectory))
    
    return time_trajectory, state_trajectory

def interpolate_trajectory(timestamps, values, query_time):
    """
    在轨迹点之间进行线性插值
    
    Args:
        timestamps: 时间序列
        values: 对应的值序列
        query_time: 查询时间点
    
    Returns:
        插值后的值
    """
    if len(timestamps) == 0:
        return []
        
    if query_time <= timestamps[0]:
        return values[0]
    if query_time >= timestamps[-1]:
        return values[-1]
    
    # 找到查询时间所在的区间
    idx = bisect.bisect_right(timestamps, query_time) - 1
    
    # 计算插值
    t0, t1 = timestamps[idx], timestamps[idx + 1]
    v0, v1 = values[idx], values[idx + 1]
    alpha = (query_time - t0) / (t1 - t0)
    return [v0[i] + alpha * (v1[i] - v0[i]) for i in range(len(v0))]

def find_nearest_timestamp_index(timestamps: List[float], current_time: float) -> int:
    """
    找到列表中最接近给定当前时间的时间戳索引。
    
    Args:
        timestamps: 按升序排列的时间戳列表。
        current_time: 要查找最近时间戳的当前时间。
        
    Returns:
        列表中最接近当前时间的时间戳索引。
    """
    if not timestamps:
        return -1
        
    # 使用bisect_left找到大于或等于current_time的第一个时间戳的索引
    insertion_point = bisect.bisect_left(timestamps, current_time)
    
    # 如果插入点是0，则返回0，因为没有比current_time小的时间戳
    if insertion_point == 0:
        return 0
    
    # 如果插入点是len(timestamps)，则返回len(timestamps) - 1，因为没有比current_time大的时间戳
    if insertion_point == len(timestamps):
        return len(timestamps) - 1
    
    # 否则，检查哪个时间戳更接近current_time并返回相应的索引
    if current_time - timestamps[insertion_point - 1] < timestamps[insertion_point] - current_time:
        return insertion_point - 1
    else:
        return insertion_point

def transform_odom_to_state(odom_x: float, odom_y: float, odom_theta: float, 
                          joint_positions: List[float]) -> List[float]:
    """
    将里程计和关节位置转换为状态向量
    
    Args:
        odom_x: 里程计X坐标
        odom_y: 里程计Y坐标
        odom_theta: 里程计航向角
        joint_positions: 关节位置列表
        
    Returns:
        状态向量
    """
    # 创建状态向量 [x, y, theta, joint1, joint2, ..., joint7]
    state = [odom_x, odom_y, odom_theta] + joint_positions
    return state 