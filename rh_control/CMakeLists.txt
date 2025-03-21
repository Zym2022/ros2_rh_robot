cmake_minimum_required(VERSION 3.8)
project(rh_control)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 安装Python模块
ament_python_install_package(${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  scripts/hmm_node.py
  scripts/test.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# 安装配置文件
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# 导出依赖
ament_export_dependencies(
  rclpy
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

ament_package()