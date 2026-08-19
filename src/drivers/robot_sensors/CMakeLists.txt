cmake_minimum_required(VERSION 3.8)
project(robot_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ylr1d_description REQUIRED)

# 传感器层：SDK 原始话题 -> 框架 30 关节名 /joint_states（ylr1d_perception 输入源）
add_executable(sensor_bridge src/sensor_bridge.cpp)
ament_target_dependencies(sensor_bridge rclcpp robot_interfaces sensor_msgs ylr1d_description)
install(TARGETS sensor_bridge DESTINATION lib/${PROJECT_NAME})

# 里程计适配：/sensors/vehicle_raw 速度积分 -> /odom + odom->Link_Base TF（真机无 EKF）
add_executable(odom_pub src/odom_pub.cpp)
ament_target_dependencies(odom_pub rclcpp robot_interfaces nav_msgs geometry_msgs tf2 tf2_ros)
install(TARGETS odom_pub DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()