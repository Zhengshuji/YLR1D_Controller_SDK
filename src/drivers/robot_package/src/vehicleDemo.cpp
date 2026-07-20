#include <rclcpp/rclcpp.hpp>
#include "RobotConSys/RobotConSys.h"
#include "SystemOperation.h"

#include <stdio.h>
#include <chrono>
#include <thread>

#define ROBOTCONSYS_IP "127.0.0.1"
#define ROBOTCONSYS_PORT 8109

class VehicleTestNode : public rclcpp::Node
{
public:
    VehicleTestNode() : Node("vehicle_test_node") {}

    int run()
    {
        RCLCPP_INFO(this->get_logger(), "=== Vehicle Test Node Started ===");

        // 加载动态库
        SysLayer::CLoadLibrary libRobotConSys;
        int ret = libRobotConSys.open("install/robot_package/lib", "RobotConSys_Client");
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "load RobotConSys_Client failed!");
            return -1;
        }
        RCLCPP_INFO(this->get_logger(), "load RobotConSys_Client succeeded!");

        rclib::RobotConSys* (*createRobotConSys)();
        createRobotConSys = (rclib::RobotConSys* (*)())libRobotConSys.loadFunc("createRobotConSys_Client");
        if(createRobotConSys == NULL){
            libRobotConSys.loadErrorPrint();
            RCLCPP_ERROR(this->get_logger(), "get createRobotConSys_Client failed");
            return -1;
        }
        RCLCPP_INFO(this->get_logger(), "get createRobotConSys_Client succeeded!");

        void (*freeRobotConSys)(rclib::RobotConSys* sys);
        freeRobotConSys = (void (*)(rclib::RobotConSys*))libRobotConSys.loadFunc("freeRobotConSys_Client");
        if(freeRobotConSys == NULL){
            libRobotConSys.loadErrorPrint();
            RCLCPP_ERROR(this->get_logger(), "get freeRobotConSys_Client failed");
            return -1;
        }
        RCLCPP_INFO(this->get_logger(), "get freeRobotConSys_Client succeeded!");

        // 创建机器人控制对象
        rclib::RobotConSys* robot = (*createRobotConSys)();

        // 初始化连接
        ret = robot->init(ROBOTCONSYS_IP, ROBOTCONSYS_PORT);
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "RobotConSys init failed, ret=%d", ret);
            (*freeRobotConSys)(robot);
            return -1;
        }
        robot->setAuthority(DevLayer::AUTHORITY_ROOT);

        // ---------- 底盘测试序列 ----------
        robot->setVehicleControlType(DevLayer::VEHICLE_CONTROL_TYPE::VEHICLE_CONTROL_UART);
        robot->setVehicleServoState(rclib::SWITCHON);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 前进 200mm/s 持续 5s
        robot->setMotionControl(DevLayer::VEHICLE_MODE::VEHICLE_MODE_NORMAL, 200, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        // 自旋 0.314 rad/s 持续 5s
        robot->setMotionControl(DevLayer::VEHICLE_MODE::VEHICLE_MODE_ROTATE, 0, 0, 0.314);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        // 停止
        robot->setMotionControl(DevLayer::VEHICLE_MODE::VEHICLE_MODE_NORMAL, 0, 0, 0);

        RCLCPP_INFO(this->get_logger(), "Vehicle test completed successfully.");

        // 释放权限并释放对象
        robot->setAuthority(DevLayer::AUTHORITY_NONE);
        (*freeRobotConSys)(robot);

        return 0;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleTestNode>();
    int ret = node->run();
    rclcpp::shutdown();
    return ret;
}