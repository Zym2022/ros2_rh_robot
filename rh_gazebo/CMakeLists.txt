cmake_minimum_required(VERSION 3.8)
project(rh_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(xacro REQUIRED)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装config文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 如果有其他目录需要安装，添加在这里
# install(DIRECTORY
#   src/rh_gazebo/...
#   DESTINATION share/${PROJECT_NAME}
# )

# 如果有可执行文件，添加在这里
# add_executable(your_node src/your_node.cpp)
# ament_target_dependencies(your_node rclcpp std_msgs)
# install(TARGETS your_node DESTINATION lib/${PROJECT_NAME})

ament_package()