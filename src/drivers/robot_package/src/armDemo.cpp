#include <rclcpp/rclcpp.hpp>
#include "RobotConSys/RobotConSys.h"
#include "SystemOperation.h"

#include <stdio.h>
#include <chrono>
#include <thread>

#define ROBOTCONSYS_IP "127.0.0.1"
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

        // ---------- 机械臂测试序列 ----------
        robsoft::RobotParameter rParam = robot->getRobotParameter(rclib::ROBOTCONSYS_ARM_1);

        robot->setServoState(rclib::ROBOTCONSYS_ARM_1, rclib::SWITCHON);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        robsoft::Joints joint(rParam.getWholeDOF());
        for(int i=0; i<joint.getJointsDOF(); i++){
            joint[robsoft::JOINTINDEX(i)] = 5*i;
        }
        ret = robot->moveABSJoint(rclib::ROBOTCONSYS_ARM_1, joint, 0.2);
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "moveABSJoint error, ret=%d", ret);
            robot->close();
            (*freeRobotConSys)(robot);
            return -1;
        }
        ret = robot->waitMotionCMDFinish(rclib::ROBOTCONSYS_ARM_1);
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "waitMotionCMDFinish error, ret=%d", ret);
            robot->close();
            (*freeRobotConSys)(robot);
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        ret = robot->returnZero(rclib::ROBOTCONSYS_ARM_1, 0.2);
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "returnZero error, ret=%d", ret);
            robot->close();
            (*freeRobotConSys)(robot);
            return -1;
        }
        ret = robot->waitMotionCMDFinish(rclib::ROBOTCONSYS_ARM_1);
        if(ret != 0){
            RCLCPP_ERROR(this->get_logger(), "waitMotionCMDFinish error, ret=%d", ret);
            robot->close();
            (*freeRobotConSys)(robot);
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        robot->setServoState(rclib::ROBOTCONSYS_ARM_1, rclib::SWITCHOFF);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        RCLCPP_INFO(this->get_logger(), "Arm test completed successfully.");

        // 关闭并释放资源
        robot->close();
        (*freeRobotConSys)(robot);

        return 0;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTestNode>();
    int ret = node->run();
    rclcpp::shutdown();
    return ret;
}