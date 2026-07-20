cmake_minimum_required(VERSION 3.8)
project(robot_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED) 


find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)

# 插入 SDK 包
set(SDK_ROOT "${PROJECT_SOURCE_DIR}/RobotConSys_SDK")
message("SDK_ROOT = " ${SDK_ROOT})
# 头文件
include_directories(${SDK_ROOT}/include)
set(3RDLIB_HEAD     "${SDK_ROOT}/include/3rdLib")
set(SYSLAYER_HEAD   "${SDK_ROOT}/include/SystemLayer")
set(DEVLAYER_HEAD   "${SDK_ROOT}/include/DeviceLayer")
set(ROBSOFT_HEAD    "${SDK_ROOT}/include/RobSoft")
set(ROBOTCONSYS_HEAD "${SDK_ROOT}/include/RobotConSys")
set(ROBOTINTERPRETER_HEAD "${SDK_ROOT}/include/RobotInterpreter")
include_directories(
    ${3RDLIB_HEAD}
    ${SYSLAYER_HEAD}
    ${DEVLAYER_HEAD}
    ${ROBSOFT_HEAD}
    ${ROBOTCONSYS_HEAD}
    ${ROBOTINTERPRETER_HEAD}
)
# 库文件
set(SDK_LIB_DIR ${SDK_ROOT}/lib)
message("SDK_LIB_DIR = " ${SDK_LIB_DIR})
link_directories(${SDK_LIB_DIR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


# 添加可执行文件
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp std_msgs robot_interfaces)
install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)

find_library(SYSTEMLAYER_LIB NAMES "libSystemLayer.so" PATHS ${SDK_ROOT}/lib REQUIRED)
find_library(CAMDEV_LIB NAMES "libCamDev_Client.so" PATHS ${SDK_ROOT}/lib REQUIRED)
find_library(ROBSOFT_LIB NAMES "libRobSoft.so" PATHS ${SDK_ROOT}/lib REQUIRED)

# camDemo 添加可执行文件
add_executable(camDemo src/camDemo.cpp)
ament_target_dependencies(camDemo
    rclcpp
    cv_bridge
    sensor_msgs
    OpenCV
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)

# 链接库
target_link_libraries(camDemo
    ${SYSTEMLAYER_LIB}
    ${CAMDEV_LIB}
    ${CURL_LIBRARIES}
    pthread dl rt
)
target_include_directories(camDemo PRIVATE ${CURL_INCLUDE_DIRS})
set_target_properties(camDemo PROPERTIES
    INSTALL_RPATH "${SDK_LIB_DIR}"
    BUILD_RPATH "${SDK_LIB_DIR}"
)
install(TARGETS camDemo
    DESTINATION lib/${PROJECT_NAME}
)

# ---------- 机械臂测试节点 ----------
add_executable(armDemo src/armDemo.cpp)
ament_target_dependencies(armDemo rclcpp)
target_link_libraries(armDemo 
    ${SYSTEMLAYER_LIB} 
    ${ROBSOFT_LIB}
    pthread dl rt)
set_target_properties(armDemo PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../lib"
    BUILD_RPATH "${SDK_LIB_DIR}"
)
install(TARGETS armDemo DESTINATION lib/${PROJECT_NAME})

# ---------- 移动底盘测试节点 ----------
add_executable(vehicleDemo src/vehicleDemo.cpp)
ament_target_dependencies(vehicleDemo rclcpp)
target_link_libraries(vehicleDemo 
    ${SYSTEMLAYER_LIB}
    ${ROBSOFT_LIB}
    pthread dl rt)
set_target_properties(vehicleDemo PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../lib"
    BUILD_RPATH "${SDK_LIB_DIR}"
)
install(TARGETS vehicleDemo DESTINATION lib/${PROJECT_NAME})

install(FILES "${SDK_LIB_DIR}/libCamDev_Client.so" DESTINATION lib)
install(FILES "${SDK_LIB_DIR}/libRobotConSys_Client.so" DESTINATION lib)
install(FILES "${SDK_LIB_DIR}/libTCPConDev_Client.so" DESTINATION lib)
install(FILES "${SDK_LIB_DIR}/libmodbus.so.5" DESTINATION lib)
install(FILES "${SDK_LIB_DIR}/libRobSoft.so" DESTINATION lib)
install(FILES "${SDK_LIB_DIR}/libSystemLayer.so" DESTINATION lib)
install(FILES "${SDK_LIB_DIR}/libmodbus.so" DESTINATION lib)
install(FILES "${SDK_LIB_DIR}/libmodbus.so.5.1.0" DESTINATION lib)

ament_package()
