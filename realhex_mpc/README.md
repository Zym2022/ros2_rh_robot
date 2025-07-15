# RealHex MPC 接口 (重构版)

本项目包含用于连接 OCS2 MPC 控制器与 RealHex 机器人的 ROS2 接口。原始的单一脚本已被重构为多个职责明确的节点，以提高模块化、可维护性和可扩展性。

## 重构概述

原始的 `test_realhex_mpc_interface.py` 脚本将状态估计、策略执行和任务管理等多个功能耦合在一个文件中。这种做法虽然在初期便于快速开发，但随着项目复杂度的增加，会变得难以维护和调试。

为了解决这个问题，我们遵循**单一职责原则**，将其拆分为三个核心节点。

## 新节点架构

重构后的系统由以下三个独立的节点组成：

### 1. 状态观测节点 (`state_observer_node.py`)

**职责**: 作为机器人的“感官系统”，专门负责收集传感器数据并将其打包成 MPC 所需的格式。
- **订阅**:
  - `/odom`: 底盘里程计
  - `/joint_states`: 关节状态
- **发布**:
  - `/mobile_manipulator_mpc_observation`: 组合后的 MPC 观测消息

### 2. 策略执行节点 (`policy_executor_node.py`)

**职责**: 作为机器人的“运动神经中枢”，负责接收 MPC 的决策并将其转化为底层硬件可以理解的命令。
- **订阅**:
  - `/mobile_manipulator_mpc_policy`: MPC 输出的控制策略
- **发布**:
  - `/cmd_vel`: 底盘速度指令
  - `/joint_velocity_controller/commands`: 仿真用的关节速度指令
  - `/rm_driver/movej_canfd_cmd`: 实物机器人用的关节位置指令
  - `/mpc_applied_input`: 当前应用的控制输入（供状态观测节点使用）

### 3. 任务管理节点 (`goal_manager_node.py`)

**职责**: 作为系统的“大脑”，负责与用户或上层应用交互，设定目标并管理 MPC 求解器的生命周期。
- **提供服务**:
  - `/generate_trajectory`: 接收高层目标（如末端位姿），生成目标轨迹
- **发布**:
  - `/mobile_manipulator_mpc_target`: MPC 目标轨迹
- **调用服务**:
  - `/mobile_manipulator_mpc_reset`: 在需要时重置 MPC 求解器

## 如何运行

### 1. 构建

在修改代码或添加新节点后，您需要重新构建软件包以使更改生效。在您的工作空间根目录 (`~/robot_ws`) 下运行：

```bash
colcon build --packages-select realhex_mpc
```

### 2. 启动

使用新创建的 `launch` 文件来启动所有重构后的节点：

```bash
# 首先 source 您的工作空间
source install/setup.bash

# 启动节点
ros2 launch realhex_mpc realhex_mpc_refactored.launch.py
```

这个 `launch` 文件会同时启动 `state_observer_node`, `policy_executor_node`, 和 `goal_manager_node`，使整个 MPC 控制系统准备就绪。 