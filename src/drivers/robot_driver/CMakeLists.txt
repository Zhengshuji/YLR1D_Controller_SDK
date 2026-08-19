cmake_minimum_required(VERSION 3.8)
project(robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(ylr1d_translate REQUIRED)

# SDK（复用 robot_package 内的 SDK 副本；真机侧 SDK 封装层仍在 robot_package）
set(SDK_ROOT "${PROJECT_SOURCE_DIR}/../robot_package/RobotConSys_SDK")
set(SDK_LIB_DIR ${SDK_ROOT}/lib)
include_directories(
    ${SDK_ROOT}/include
    ${SDK_ROOT}/include/3rdLib
    ${SDK_ROOT}/include/SystemLayer
    ${SDK_ROOT}/include/DeviceLayer
    ${SDK_ROOT}/include/RobSoft
    ${SDK_ROOT}/include/RobotConSys
    ${SDK_ROOT}/include/RobotInterpreter
)
link_directories(${SDK_LIB_DIR})

find_library(SYSTEMLAYER_LIB NAMES "libSystemLayer.so" PATHS ${SDK_ROOT}/lib REQUIRED)
find_library(ROBSOFT_LIB NAMES "libRobSoft.so" PATHS ${SDK_ROOT}/lib REQUIRED)

# 驱动节点：3 action（复用 ylr1d_translate 类型）+ SDK 直调 + /sensors/*_raw + /health
add_executable(robot_driver src/robot_driver.cpp)
target_include_directories(robot_driver PRIVATE include)
ament_target_dependencies(robot_driver rclcpp rclcpp_action std_msgs robot_interfaces ylr1d_translate)
target_link_libraries(robot_driver ${SYSTEMLAYER_LIB} ${ROBSOFT_LIB} pthread dl rt)
set_target_properties(robot_driver PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../lib"
    BUILD_RPATH "${SDK_LIB_DIR}"
)
install(TARGETS robot_driver DESTINATION lib/${PROJECT_NAME})

# SDK 运行库随包安装（与 robot_package 同款，保证 $ORIGIN/../lib 命中）
install(FILES
    "${SDK_LIB_DIR}/libRobotConSys_Client.so"
    "${SDK_LIB_DIR}/libCamDev_Client.so"
    "${SDK_LIB_DIR}/libTCPConDev_Client.so"
    "${SDK_LIB_DIR}/libmodbus.so"
    "${SDK_LIB_DIR}/libmodbus.so.5"
    "${SDK_LIB_DIR}/libmodbus.so.5.1.0"
    "${SDK_LIB_DIR}/libRobSoft.so"
    "${SDK_LIB_DIR}/libSystemLayer.so"
    DESTINATION lib)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()