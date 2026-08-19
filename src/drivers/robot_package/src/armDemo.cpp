#include <rclcpp/rclcpp.hpp>
#include "RobotConSys/RobotConSys.h"
#include "SystemOperation.h"
#include "robot_interfaces/msg/tr_arm_msg.hpp"

#include <stdio.h>
#include <chrono>
#include <thread>
#include <atomic>

#define ROBOTCONSYS_IP "172.22.224.1"
#define ROBOTCONSYS_PORT 8109

class ArmTestNode : public rclcpp::Node
{
public:
    ArmTestNode() : Node("arm_test_node") {}

    int run()
    {
        RCLCPP_INFO(this->get_logger(), "=== Arm Test Node Started ===");

        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
            RCLCPP_INFO(this->get_logger(), "Current working directory: %s", cwd);
        }

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
        robot->setAuthority(DevLayer::AUTHORITY_OPERATOR);

        // ---------- 传感器原始话题发布（供感知层测试） ----------
        // 20Hz 轮询 SDK 状态，按 printState 模式取 joints + terminal，发布 /sensors/arm_raw
        arm_state_pub_ = this->create_publisher<robot_interfaces::msg::TRArmMsg>("/sensors/arm_raw", 10);
        auto stopPub = [this]() {
            pub_running_ = false;
            if (pub_thread_.joinable()) { pub_thread_.join(); }
        };
        pub_running_ = true;
        pub_thread_ = std::thread([this, robot]() {
            while (pub_running_) {
                robot->updateRobotMotion(rclib::ROBOTCONSYS_ARM_1);
                robsoft::RobotMotion motion = robot->getRobotMotion(rclib::ROBOTCONSYS_ARM_1);
                robsoft::Joints j = motion.getCurrentJointPosition();
                robsoft::Terminal t = motion.getCurrentTerminal();
                robot->updateRobotState(rclib::ROBOTCONSYS_ARM_1);
                robot->updateClawState(rclib::ROBOTCONSYS_ARM_1);

                robot_interfaces::msg::TRArmMsg m;
                m.name = "arm0";
                m.dof = j.getJointsDOF();
                m.servo_state = (robot->getServoState(rclib::ROBOTCONSYS_ARM_1) == rclib::SWITCHON);
                m.claw_state = (robot->getClawState(rclib::ROBOTCONSYS_ARM_1) == rclib::SWITCHON);
                for (int i = 0; i < j.getJointsDOF(); ++i) {
                    m.joints.push_back(static_cast<float>(j[robsoft::JOINTINDEX(i)]));
                }
                for (int i = 0; i < 6; ++i) {
                    m.terminal.push_back(static_cast<float>(t[robsoft::TERMINALINDEX(i)]));
                }
                arm_state_pub_->publish(m);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });

        // ---------- 机械臂测试序列 ----------
        robsoft::RobotParameter rParam = robot->getRobotParameter(rclib::ROBOTCONSYS_ARM_1);

        robot->setServoState(rclib::ROBOTCONSYS_ARM_1, rclib::SWITCHON);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        robsoft::Joints joint(rParam.getWholeDOF());
        for(int i=0; i<joint.getJointsDOF(); i++){
            joint[robsoft::JOINTINDEX(i)] = 0.3*i;   // 限位内（URDF: J1±2.62 J2±1.83 J3±2.62 J4±1.57 J5±2.62 J6±2.09 J7±6.28）；曾用 5*i 会停到越限位导致 moveit 起点无效
        }
        ret = robot->moveABSJoint(rclib::ROBOTCONSYS_ARM_1, joint, 0.2);
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "moveABSJoint error, ret=%d", ret);
            stopPub();
            robot->close();
            (*freeRobotConSys)(robot);
            return -1;
        }
        ret = robot->waitMotionCMDFinish(rclib::ROBOTCONSYS_ARM_1);
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "waitMotionCMDFinish error, ret=%d", ret);
            stopPub();
            robot->close();
            (*freeRobotConSys)(robot);
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        ret = robot->returnZero(rclib::ROBOTCONSYS_ARM_1, 0.2);
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "returnZero error, ret=%d", ret);
            stopPub();
            robot->close();
            (*freeRobotConSys)(robot);
            return -1;
        }
        ret = robot->waitMotionCMDFinish(rclib::ROBOTCONSYS_ARM_1);
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "waitMotionCMDFinish error, ret=%d", ret);
            stopPub();
            robot->close();
            (*freeRobotConSys)(robot);
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        robot->setServoState(rclib::ROBOTCONSYS_ARM_1, rclib::SWITCHOFF);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        RCLCPP_INFO(this->get_logger(), "Arm test completed successfully.");

        // 关闭并释放资源
        stopPub();
        robot->close();
        (*freeRobotConSys)(robot);

        return 0;
    }

private:
    rclcpp::Publisher<robot_interfaces::msg::TRArmMsg>::SharedPtr arm_state_pub_;
    std::atomic<bool> pub_running_{false};
    std::thread pub_thread_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTestNode>();
    int ret = node->run();
    rclcpp::shutdown();
    return ret;
}