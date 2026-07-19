#ifndef CROBOTPARAMETER_HPP
#define CROBOTPARAMETER_HPP

#include "CDataStructure.hpp"

namespace robsoft{

enum ROBOTTYPE{                         // 机器人类型
    ROBSOFT_SERIAL_SIX_CONVENTION,      // 六轴常规串联机型
    ROBSOFT_SERIAL_SIX_COOPERATION,     // 六轴协作串联机型
    ROBSOFT_SCARA_THREE_ONERF,          // SCARA机器人，一轴可升降，无四轴
    ROBSOFT_SCARA_FOUR_ONERF,           // SCARA机器人，一轴可升降
    ROBSOFT_SCARA_FOUR_THREERF,         // SCARA机器人，三轴可升降
    ROBSOFT_SCARA_FOUR_FOURRF,          // SCARA机器人，四轴可升降
    ROBSOFT_DELTA_FOUR,                 // DELTA机器人，单姿态
    ROBSOFT_DELTA_SIX,                  // DELTA机器人，三姿态
    ROBSOFT_SERIAL_SEVEN_SRS,           // 七轴SRS串联机型
	ROBSOFT_CARTESIAN,                  // 直角坐标机器人
    ROBSOFT_SERIAL_FOUR_CONVENTION,     // 四轴常规串联机型，码垛
    ROBSOFT_SERIAL_GENERAL,             // 通用串联机型
};

enum EXTERNAL_AXIS_TYPE{                // 外部轴类型
    ROBSOFT_EXTERNAL_AXIS_NORMAL,       // 通用外部轴
    ROBSOFT_EXTERNAL_AXIS_TRANSLATION,  // 行走轴
    ROBSOFT_EXTERNAL_AXIS_POSITIONER,   // 变位机
    ROBSOFT_EXTERNAL_AXIS_ARM_DUAL_3,   // 双臂三轴
    ROBSOFT_EXTERNAL_AXIS_Embodied_SINGLE_3,  // 单臂复合三轴
    ROBSOFT_EXTERNAL_AXIS_Embodied_DUAL_4,  // 双臂复合四轴
};

enum ROBSOFT_JOINT_TYPE{
    ROBSOFT_JOINT_TYPE_ROTATE,          // 旋转机构
    ROBSOFT_JOINT_TYPE_MOVE,            // 平移机构
};

class DECLSPEC_DLLEXPORT RobotParameter{
public:
    RobotParameter();
    ~RobotParameter();

    void alignDOF();    // 对齐自由度

    void readRobotParameter(const char* path);
    void writeRobotParameter(const char* path) const;

    void print() const;

    bool isDriverSame(const RobotParameter& robot) const;       // 判断是否需要重新载入参数
    bool operator ==(const RobotParameter &robot) const;
    bool operator !=(const RobotParameter &robot) const;

    Joints JointsToMotorPos(const Joints& joint) const;
    Joints MotorToJointsPos(const Joints& joint) const;
    Joints JointsToMotorVel(const Joints& joint) const;
    Joints MotorToJointsVel(const Joints& joint) const;
    Joints JointsToMotorTor(const Joints& joint) const;
    Joints MotorToJointsTor(const Joints& joint) const;

    // 参数设置和获取
    void setRobotParameter(const RobotParameter& robot);
    RobotParameter getRobotParameter() const;

    void setSamplePeriod(double samplePeriod);
    void getSamplePeriod(double& samplePeriod) const;
    double getSamplePeriod() const;

    void setRobotDOF(int robotDof);
    int getRobotDOF() const;

    void setExternDOF(int externDof);
    int getExternDOF() const;
    
    int getWholeDOF() const;

    void setRobotType(ROBOTTYPE robotType);
    void getRobotType(ROBOTTYPE& robotType) const;
    ROBOTTYPE getRobotType() const;

    void setExternalAxisType(EXTERNAL_AXIS_TYPE externalAxisType);
    void getExternalAxisType(EXTERNAL_AXIS_TYPE& externalAxisType) const;
    EXTERNAL_AXIS_TYPE getExternalAxisType() const;

    void setRateTorque(const Joints& rateTorque);
    void getRateTorque(Joints& rateTorque) const;
    Joints getRateTorque() const;

    void setPosResolution(const Joints& posResolution);
    void getPosResolution(Joints& posResolution) const;
    Joints getPosResolution() const;

    void setVelResolution(const Joints& velResolution);
    void getVelResolution(Joints& velResolution) const;
    Joints getVelResolution() const;

    void setTorResolution(const Joints& torResolution);
    void getTorResolution(Joints& torResolution) const;
    Joints getTorResolution() const;

    void setReduceRatio(const Joints& reduceRatio);
    void getReduceRatio(Joints& reduceRatio) const;
    Joints getReduceRatio() const;

    void setPitch(const Joints& pitch);
    void getPitch(Joints& pitch) const;
    Joints getPitch() const;

    void setCouplingRatio(const Joints& couplingRatio);
    void getCouplingRatio(Joints& couplingRatio) const;
    Joints getCouplingRatio() const;

    void setJointRange(const Joints& jointRangeMinus, const Joints& jointRangePlus);
    void setJointRangeMinus(const Joints& jointRangeMinus);
    void setJointRangePlus(const Joints& jointRangePlus);
    void getJointRange(Joints& jointRangeMinus, Joints& jointRangePlus) const;
    Joints getJointRangeMinus() const;
    Joints getJointRangePlus() const;

    void setJointMaxVelRange(const Joints& jointMaxVel);
    void getJointMaxVelRange(Joints& jointMaxVel) const;
    Joints getJointMaxVelRange() const;

    void setJointMaxAccRange(const Joints& jointMaxAcc);
    void getJointMaxAccRange(Joints& jointMaxAcc) const;
    Joints getJointMaxAccRange() const;

    void setJointMaxJerkRange(const Joints& jointMaxJerk);
    void getJointMaxJerkRange(Joints& jointMaxJerk) const;
    Joints getJointMaxJerkRange() const;

    void setJointMaxVelRatio(double jointMaxVelRatio);
    void getJointMaxVelRatio(double& jointMaxVelRatio) const;
    double getJointMaxVelRatio() const;

    void setJointMaxAccRatio(double jointMaxAccRatio);
    void getJointMaxAccRatio(double& jointMaxAccRatio) const;
    double getJointMaxAccRatio() const;

    void setJointMaxJerkRatio(double jointMaxJerkRatio);
    void getJointMaxJerkRatio(double& jointMaxJerkRatio) const;
    double getJointMaxJerkRatio() const;

    // 获取最大速度已乘上比例系数
    void getJointMaxVel(Joints& jointMaxVel) const;
    Joints getJointMaxVel() const;

    // 获取最大加速度已乘上比例系数
    void getJointMaxAcc(Joints& jointMaxAcc) const;
    Joints getJointMaxAcc() const;

    // 获取最大冲击已乘上比例系数
    void getJointMaxJerk(Joints& jointMaxJerk) const;
    Joints getJointMaxJerk() const;
    
    void setTerminalRange(const Terminal& terminalRangeMinus, const Terminal& terminalRangePlus);
    void setTerminalRangeMinus(const Terminal& terminalRangeMinus);
    void setTerminalRangePlus(const Terminal& terminalRangePlus);
    void getTerminalRange(Terminal& terminalRangeMinus, Terminal& terminalRangePlus) const;
    Terminal getTerminalRangeMinus() const;
    Terminal getTerminalRangePlus() const;

    void setTerminalMaxVelRange(const Terminal& terminalMaxVel);
    void getTerminalMaxVelRange(Terminal& terminalMaxVel) const;
    Terminal getTerminalMaxVelRange() const;

    void setTerminalMaxAccRange(const Terminal& terminalMaxAcc);
    void getTerminalMaxAccRange(Terminal& terminalMaxAcc) const;
    Terminal getTerminalMaxAccRange() const;

    void setTerminalMaxJerkRange(const Terminal& terminalMaxJerk);
    void getTerminalMaxJerkRange(Terminal& terminalMaxJerk) const;
    Terminal getTerminalMaxJerkRange() const;

    void setTerminalMaxVelRatio(double terminalMaxVelRatio);
    void getTerminalMaxVelRatio(double& terminalMaxVelRatio) const;
    double getTerminalMaxVelRatio() const;

    void setTerminalMaxAccRatio(double terminalMaxAccRatio);
    void getTerminalMaxAccRatio(double& terminalMaxAccRatio) const;
    double getTerminalMaxAccRatio() const;

    void setTerminalMaxJerkRatio(double terminalMaxJerkRatio);
    void getTerminalMaxJerkRatio(double& terminalMaxJerkRatio) const;
    double getTerminalMaxJerkRatio() const;

    // 获取最大速度已乘上比例系数
    void getTerminalMaxVel(Terminal& terminalMaxVel) const;
    Terminal getTerminalMaxVel() const;

    // 获取最大加速度已乘上比例系数
    void getTerminalMaxAcc(Terminal& terminalMaxAcc) const;
    Terminal getTerminalMaxAcc() const;

    // 获取最大冲击已乘上比例系数
    void getTerminalMaxJerk(Terminal& terminalMaxJerk) const;
    Terminal getTerminalMaxJerk() const;

    void setModelZeroOffset(const Joints& modelZeroOffset);
    void getModelZeroOffset(Joints& modelZeroOffset) const;
    Joints getModelZeroOffset() const;

    void setDHParameter(const Joints& DHParameterAlpha, const Joints& DHParameterA, const Joints& DHParameterD, const Joints& DHParameterTheta);
    void setDHParameterAlpha(const Joints& DHParameterAlpha);
    void setDHParameterA(const Joints& DHParameterA);
    void setDHParameterD(const Joints& DHParameterD);
    void setDHParameterTheta(const Joints& DHParameterTheta);
    void getDHParameter(Joints& DHParameterAlpha, Joints& DHParameterA, Joints& DHParameterD, Joints& DHParameterTheta) const;
    Joints getDHParameterAlpha() const;
    Joints getDHParameterA() const;
    Joints getDHParameterD() const;
    Joints getDHParameterTheta() const;

    void setDELTAPara(double delta_r, double delta_R, double delta_L1, double delta_L2, double delta_Z);
    double getDELTAPara_r() const;
    double getDELTAPara_R() const;
    double getDELTAPara_L1() const;
    double getDELTAPara_L2() const;
    double getDELTAPara_Z() const;

    void setBaseFrame(const Terminal& baseFrame);
    void getBaseFrame(Terminal& baseFrame) const;
    Terminal getBaseFrame() const;

    void setEndFrame(const Terminal& endFrame);
    void getEndFrame(Terminal& endFrame) const;
    Terminal getEndFrame() const;

    void setLink(const std::vector<Link>& links);
    void getLink(std::vector<Link>& links) const;
    std::vector<Link> getLink() const;

    // 关节摩擦力参数
    void setSlope(const Joints& slope);
    Joints getSlope() const;

    void setIntercept(const Joints& intercept);
    Joints getIntercept() const;

    void setMaxFrictionTorque(const Joints& maxFrictionTorque);
    Joints getMaxFrictionTorque() const;

    void setTorqueCompensationRatio(const Joints& torqueCompensationRatio);
    Joints getTorqueCompensationRatio() const;

protected:   
    double m_samplePeriod;                  // 指令周期，(秒)
    
    int m_robotDOF;                         // 机器人自由度
    int m_externDOF;                        // 外部轴自由度

    ROBOTTYPE m_robotType;                  // 机器人类型
    EXTERNAL_AXIS_TYPE m_externalAxisType;  // 外部轴类型

    Joints m_rateTorque;                    // 额定转矩，(Nm)
    Joints m_posResolution;                 // 位置分辨率
    Joints m_velResolution;                 // 速度分辨率
    Joints m_torResolution;                 // 扭矩分辨率
    Joints m_reduceRatio;                   // 减速器减速比  输入/输出
    Joints m_pitch;                         // 螺距, (mm/turn)
    Joints m_couplingRatio;                 // 耦合比

    Joints m_jointRangeMinus;               // 负向关节范围，（度）或（mm）
    Joints m_jointRangePlus;                // 正向关节范围，（度）或（mm）
    Joints m_jointMaxVel;                   // 关节最大速度，（度/s）或（mm/s）
    Joints m_jointMaxAcc;                   // 关节最大加速度，（度/s^2）或（mm/s^2）
    Joints m_jointMaxJerk;                  // 关节最大冲击，（度/s^3）或（mm/s^3）
    double m_jointMaxVelRatio;              // 关节最大速度比例，
    double m_jointMaxAccRatio;              // 关节最大加速度比例
    double m_jointMaxJerkRatio;             // 关节最大冲击比例

    Terminal m_terminalRangeMinus;          // 负向末端位置范围，（mm）
    Terminal m_terminalRangePlus;           // 正向末端位置范围，（mm）
    Terminal m_terminalMaxVel;              // 末端最大速度，（mm/s）
    Terminal m_terminalMaxAcc;              // 末端最大加速度，（mm/s^2）
    Terminal m_terminalMaxJerk;             // 末端最大冲击，（mm/s^3）
    double m_terminalMaxVelRatio;           // 末端最大速度比例
    double m_terminalMaxAccRatio;           // 末端最大加速度比例
    double m_terminalMaxJerkRatio;          // 末端最大冲击比例

    Joints m_modelZeroOffset;               // 实际零位相对模型偏移，（度）或（mm）
    Joints m_DHParameterAlpha;              // DH参数, alpha，（度）
    Joints m_DHParameterA;                  // DH参数, a，（mm）
    Joints m_DHParameterD;                  // DH参数, d，（mm）
    Joints m_DHParameterTheta;              // DH参数, theta，（度）
    double m_delta_r, m_delta_R, m_delta_L1, m_delta_L2, m_delta_Z; // DELTA参数，（mm）

    Terminal m_baseFrame;                   // 基坐标系
    Terminal m_endFrame;                    // 末端坐标系

    std::vector<Link> m_links;              // 连杆属性
    Joints m_slope;                         // 摩擦力斜率
    Joints m_intercept;                     // 摩擦力截距
    Joints m_maxFrictionTorque;             // 最大摩擦力矩
    Joints m_torqueCompensationRatio;       // 力矩补偿比例
};

}

#endif
