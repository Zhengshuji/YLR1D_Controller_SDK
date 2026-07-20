#ifndef CROBOTICS_HPP
#define CROBOTICS_HPP

#include "CRobotParameter.hpp"
#include "CMathBasic.hpp"

namespace robsoft{

class DECLSPEC_DLLEXPORT Robotics:public RobotParameter{
public:
    Robotics();
    ~Robotics();

    void setCurrentRobotMotion(const RobotMotion& currentRobotMotion);  // 设置当前运动状态

    HomogeneousMatrix getTCPFrame() const;
    HomogeneousMatrix getWaistFrame() const;
    HomogeneousMatrix getHeadFrame() const;

    RobotMotion getCurrentRobotMotion() const;  // 获取当前运动状态
    void setCurrentJointPosition(const Joints& currentJointPosition);   // 设置关节角度
    void setCurrentJointVelocity(const Joints& currentJointVelocity);   // 设置关节速度
    void setCurrentJointAcceleration(const Joints& currentJointAcceleration);   // 设置关节加速度
    void setCurrentJointTorque(const Joints& currentJointTorque);   // 设置关节力矩
    Joints getCurrentJointPosition() const; // 获取当前关节角度
    Joints getCurrentJointVelocity() const; // 获取当前关节速度
    Joints getCurrentJointAcceleration() const; // 获取当前关节加速度
    Joints getCurrentJointTorque() const;   // 获取当前关节力矩

    Terminal getCurrentTerminal();  // 获取基坐标系下的末端位姿
    Terminal getCurrentTerminal(const Joints& joints);  // 获取基坐标系下的末端位姿
    Terminal getCurrentWorkTerminal();  // 获取工件坐标系下的末端位姿
    Terminal getCurrentWorkTerminal(const Joints& joints);  // 获取工件坐标系下的末端位姿

    void setTool(const Tool& tool); // 设置工具
    Tool getTool() const;   // 获取工具

    void setWorkFrame(const Terminal& workFrame);   // 设置工件坐标系
    Terminal getWorkFrame() const;  // 获取工件坐标系

    void setForceSensor(const ForceSensor& forceSensor);    // 设置力传感器
    ForceSensor getForceSensor() const; // 获取力传感器

    Terminal forwardKinematics(const Joints& joints, bool toolEnabled = true, bool baseEnable = true, bool endEnable = true) const; // 正运动学
    int inverseKinematics(Joints& joints, const Terminal& terminal, const Joints& lastJoints, COORDINATESYSTEM coord = COORDINATE_BASE, double theta = 0) const;    // 逆运动学
    int inverseKinematics(JointsList& jointslist, const TerminalList &terminalList, const Joints &lastJoints, COORDINATESYSTEM coord = COORDINATE_BASE, double theta = 0) const;    // 逆运动学批处理

    int inverse_kinematics_serial_six_convention(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const;  // 六轴常规型机器人求逆
    int inverse_kinematics_serial_six_cooperation(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const; // 六轴协作型机器人求逆
    int inverse_kinematics_scara_three_onerf(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const;   // 三轴SCARA求逆，一轴为升降轴
    int inverse_kinematics_scara_four_onerf(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const;   // 四轴SCARA求逆，一轴为升降轴
    int inverse_kinematics_scara_four_threerf(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const;  // 四轴SCARA求逆，三轴为升降轴
    int inverse_kinematics_scara_four_fourrf(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const;  // 四轴SCARA求逆，四轴为升降轴
    int inverse_kinematics_delta_four(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const; // DELTA机器人求逆，一个姿态轴
    int inverse_kinematics_delta_six(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const;  // DELTA机器人求逆，三个姿态轴
    int inverse_kinematics_serial_seven_srs(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints, double theta = 0) const;    // 七轴SRS机器人求逆，参数thta为臂形角
	int inverse_kinematics_cartesian(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const; //直角坐标机器人求逆
    int inverse_kinematics_serial_four_convention(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints) const; // 四轴码垛型机器人求逆
    int inverse_kinematics_serial_general(JointsList& jointslist, const Terminal &terminal, const Joints& lastJoints, double epslon = 0.01, int iter = INVERSE_KINEMATICS_ITERATION) const; // 串联型机器人通用求逆
    int inverse_kinematics_serial_general(Joints& joints, const Terminal &terminal, const Joints& lastJoints, JOINTINDEX index, double epslon = 0.1, int iter = INVERSE_KINEMATICS_ITERATION) const; // 冗余机械臂锁定一个轴，串联型机器人通用求逆

    int TCPCalibrate(const JointsList &jointslist, Terminal &frame, double& pre); // 工具坐标系校准，仅校准位置
    int TCPCalibrateZ(const JointsList &jointslist, const Joints &jo, const Joints &jz, Terminal &frame, double& pre);    // 工具坐标系校准，校准位置和工具Ｚ轴方向
    int TCPCalibrateXZ(const JointsList &jointslist, const Joints &jo, const Joints &jx, const Joints &jz, Terminal &frame, double& pre); // 工具坐标系校准，校准位置和姿态

    void TCPCalibrateSCARA(const Joints &j1, const Joints &j2, Terminal &frame);
    void workFrameCalibrateSCARA(const Terminal &to, const Terminal &tx, const Terminal &ty, const Terminal &offset, Terminal &frame);  // SCARA机器人工件坐标系标定
    void workFrameCalibrateSCARACircle(const TerminalList &tCircle, Terminal &frame);

    int forceSensorCalibrate(const JointsList& jointslist, const TerminalList& forcelist, Terminal& baseFrame, Tool& tool, Terminal& forceSensorZeroPoint) const;    // 利用末端六维力传感器校准力传感器零点、基坐标系、工具质量和重心
    Terminal forceSensorCompensation(const Joints& joints, const Terminal& force) const;  // 返回六维力传感器零点和工具补偿后的结果
    Terminal admittanceControl(const Joints& currentJoint, const Joints& currentJointVel, const Terminal& targetTer, const Terminal& targetTerVel, const Joints& force) const; // 导纳控制，其中force为补偿后的末端环境力

    Joints getDragTorqueCompensation(); // 计算拖动力矩补偿

    Matrix6Xd jacobi_matrix(const Joints& joints, bool toolEnabled = true) const;    // 计算雅可比矩阵，相对末端，雅科比速度公式得到的姿态角顺利依次为X，Y，Z
    Matrix6Xd jacobi_matrix(const Joints& joints, JOINTINDEX fixIndex, bool toolEnabled = true) const;    // 冗余机械臂锁定一个轴计算雅可比矩阵，相对末端

    Terminal j2tVel(const Joints& jointsVel, const Joints& joints, COORDINATESYSTEM coord =COORDINATE_BASE) const;  // 利用雅科比矩阵根据当前关节位置和速度计算末端速度
    Joints t2jVel(const Terminal& terVel, const Joints& joints, COORDINATESYSTEM coord =COORDINATE_BASE) const;    // 利用雅科比矩阵根据当前关节位置和末端速度计算关节速度t

    HomogeneousMatrix DHParameterToHomogeneousMatrix(double alpha, double a, double d, double theta) const; // DH模型转齐次矩阵

    double calculateArmAngle(const Joints& joints);  // SRS构型七轴机器人计算臂形角
    int checkJointWithinPerformance(const JointsMotionState& jointMotion);  // 检查是否符合机器人的性能要求

private:
    int inverse_kinematics_newton_raphson_iteration(Joints& joints, const HomogeneousMatrix& homoMat, const Joints& lastJoints, double epslon = 0.1, int iter = INVERSE_KINEMATICS_ITERATION) const;  // 牛顿-拉夫逊迭代求机器人逆解
    int inverse_kinematics_newton_raphson_iteration(Joints& joints, const HomogeneousMatrix& homoMat, const Joints& lastJoints, JOINTINDEX index, double epslon = 0.1, int iter = INVERSE_KINEMATICS_ITERATION) const;  // 冗余机械臂锁定一个轴，牛顿-拉夫逊迭代求机器人逆解
    Vector6d differential_motion_vector(const HomogeneousMatrix& homoMat, const Joints& joints) const;   // 计算微分运动矢量

    Joints chooseBestJoints(const JointsList &jointslist, const Joints &lastJoints) const;    // 根据二范数选择最优关节角度解
    bool isAvailableJoints(const Joints &joints, const Joints& jointsRangeMin, const Joints& jointsRangeMax) const; // 判断角度值是否在关节旋转范围内
    double fitAngleRange(double angle, double angleRef, double min, double max) const;  // 归一化角度值到关节范围内
    void fitJointsRange(Joints &joints, const Joints& lastJoints, const Joints& jointsRangeMin, const Joints& jointsRangeMax) const;    // 归一化所有角度到关节范围内

    Joints newton_eular_dynamics() const;
    void newton_eular_dynamics_extrapolation(int index, const Point& w, const Point& wd, const Point& vd, Point& w1, Point& wd1, Point& vd1, Point& F1, Point& N1) const; // 牛顿欧拉旋转关节外推
    void newton_eular_dynamics_interpolating(int index, const Point& f1, const Point& F, const Point& N, const Point& n1, Point& f, Point& n) const; // 牛顿欧拉旋转关节内推

protected:
    RobotMotion m_currentRobotMotion;
    Tool m_tool;                            // 工具
    ForceSensor m_forceSensor;              // 六维力传感器
    Terminal m_workFrame;                   // 工件坐标系
};

}
#endif
