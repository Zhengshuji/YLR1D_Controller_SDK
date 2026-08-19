#include <rclcpp/rclcpp.hpp>
#include "RobotConSys/RobotConSys.h"
#include "SystemOperation.h"
#include "robot_interfaces/msg/tr_vehicle_msg.hpp"

#include <stdio.h>
#include <chrono>
#include <thread>
#include <atomic>

#define ROBOTCONSYS_IP "172.22.224.1"
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

        // ---------- 传感器原始话题发布（供感知层测试） ----------
        // 20Hz 轮询 SDK 状态发布 /sensors/vehicle_raw；速度源策略：
        //   优先 SDK 上报实际速度（Vehicle_State_Data.vehicle_x/y/z_vel）；
        //   若上报恒 0 而命令非 0（模拟器可能不反馈），回落命令速度并打日志（判定记录）。
        vehicle_state_pub_ = this->create_publisher<robot_interfaces::msg::TRVehicleMsg>("/sensors/vehicle_raw", 10);
        pub_running_ = true;
        pub_thread_ = std::thread([this, robot]() {
            while (pub_running_) {
                robot->updateVehicleState();
                DevLayer::Vehicle_State_Data st = robot->getVehicleState();

                float rx = st.vehicle_x_vel, ry = st.vehicle_y_vel, rz = st.vehicle_z_vel;
                const bool reported_zero = (rx == 0.0f && ry == 0.0f && rz == 0.0f);
                const bool cmd_nonzero  = (cmd_x_ != 0.0f || cmd_y_ != 0.0f || cmd_z_ != 0.0f);
                if (reported_zero && cmd_nonzero) {
                    if (!fallback_logged_) {
                        printf("[vehicleDemo] reported vel all-zero while commanded nonzero -> fallback to commanded\n");
                        fallback_logged_ = true;
                    }
                    rx = cmd_x_; ry = cmd_y_; rz = cmd_z_;
                } else if (!reported_zero) {
                    fallback_logged_ = false;   // 上报恢复，回到上报速度
                }

                robot_interfaces::msg::TRVehicleMsg m;
                m.vehicle_servo_state = static_cast<int32_t>(st.vehicleServoState);
                m.vehicle_x_vel = rx;
                m.vehicle_y_vel = ry;
                m.vehicle_z_vel = rz;
                for (int i = 0; i < 4; ++i) {
                    m.wheel_vel.push_back(st.wheelVel[i]);
                    m.servo_pos.push_back(st.servoPos[i]);
                }
                for (int i = 0; i < 8; ++i) {
                    m.sonar.push_back(static_cast<int8_t>(st.sonar[i]));
                }
                vehicle_state_pub_->publish(m);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });

        // ---------- 底盘测试序列 ----------
        robot->setVehicleControlType(DevLayer::VEHICLE_CONTROL_TYPE::VEHICLE_CONTROL_UART);
        robot->setVehicleServoState(rclib::SWITCHON);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 前进 200mm/s 持续 5s
        setCmd(DevLayer::VEHICLE_MODE::VEHICLE_MODE_NORMAL, 200, 0, 0);
        robot->setMotionControl(DevLayer::VEHICLE_MODE::VEHICLE_MODE_NORMAL, 200, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        // 自旋 0.314 rad/s 持续 5s
        setCmd(DevLayer::VEHICLE_MODE::VEHICLE_MODE_ROTATE, 0, 0, 0.314f);
        robot->setMotionControl(DevLayer::VEHICLE_MODE::VEHICLE_MODE_ROTATE, 0, 0, 0.314f);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        // 停止
        setCmd(DevLayer::VEHICLE_MODE::VEHICLE_MODE_NORMAL, 0, 0, 0);
        robot->setMotionControl(DevLayer::VEHICLE_MODE::VEHICLE_MODE_NORMAL, 0, 0, 0);

        RCLCPP_INFO(this->get_logger(), "Vehicle test completed successfully.");

        // 释放权限并释放对象
        pub_running_ = false;
        if (pub_thread_.joinable()) { pub_thread_.join(); }
        robot->setAuthority(DevLayer::AUTHORITY_NONE);
        (*freeRobotConSys)(robot);

        return 0;
    }

private:
    void setCmd(DevLayer::VEHICLE_MODE mode, float x, float y, float z)
    {
        cmd_x_ = x; cmd_y_ = y; cmd_z_ = z;
    }

    rclcpp::Publisher<robot_interfaces::msg::TRVehicleMsg>::SharedPtr vehicle_state_pub_;
    std::atomic<bool> pub_running_{false};
    std::thread pub_thread_;
    float cmd_x_ = 0.0f, cmd_y_ = 0.0f, cmd_z_ = 0.0f;
    bool fallback_logged_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleTestNode>();
    int ret = node->run();
    rclcpp::shutdown();
    return ret;
}