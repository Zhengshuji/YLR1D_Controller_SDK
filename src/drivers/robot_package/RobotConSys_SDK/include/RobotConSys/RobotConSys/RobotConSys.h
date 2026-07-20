#ifndef ROBOTCONSYS_H
#define ROBOTCONSYS_H

#include <string>
#include <vector>
#include "TCPDev/TCPDev_TypeDef.h"
#include "RobotDev/RobotDev_TypeDef.h"
#include "VehicleMotionDev/VehicleMotionDev_TypeDef.h"
#include "CRobotParameter.hpp"
#include "RobotConSys_TypeDef.h"
#include "RobotConSys_Struct.h"
#include "sentenceStruct.h"

namespace rclib{

class DECLSPEC_DLLEXPORT RobotConSys{
public:
    virtual int init(const char* ip = "127.0.0.1", int port = 8080, const char* configDir = ".", int modbusPort = -1) = 0;
    virtual int init(int port = 8080, const char* configDir = ".", int modbusPort = -1) = 0;
    virtual void close() = 0;

    virtual bool isEstablished() = 0;
    virtual DevLayer::AUTHORITY_TYPE getAuthority() = 0;
    virtual void setAuthority(DevLayer::AUTHORITY_TYPE authority) = 0;
    virtual bool online() = 0;

    //获取/设置机器人控制系统配置参数
    virtual void writeRobotConSysPreference(const RobotConSysPreference& robotConSysPreference) = 0;
    virtual int setRobotConSysPreference(const RobotConSysPreference& robotConSysPreference) = 0;
    virtual RobotConSysPreference readRobotConSysPreference() = 0;
    virtual RobotConSysPreference getRobotConSysPreference() = 0;
    
    // 获取/设置机器人参数
    virtual void writetRobotParameter(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::RobotParameter& rParam) = 0;
    virtual int setRobotParameter(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::RobotParameter& rParam) = 0;
    virtual robsoft::RobotParameter readRobotParameter(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual robsoft::RobotParameter getRobotParameter(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    
    // 获取/设置机器人配置
    virtual void writeRobotPreference(ROBOTCONSYS_ARM_INDEX armIndex, const RobotPreference &rPrefer) = 0;
    virtual int setRobotPreference(ROBOTCONSYS_ARM_INDEX armIndex, const RobotPreference &rPrefer) = 0;
    virtual RobotPreference readRobotPreference(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual RobotPreference getRobotPreference(ROBOTCONSYS_ARM_INDEX armIndex) = 0;

    // 获取/设置坐标系配置
    virtual void writeRobotFrame(ROBOTCONSYS_ARM_INDEX armIndex, const RobotFrame &rFrame) = 0;
    virtual void setRobotFrame(ROBOTCONSYS_ARM_INDEX armIndex, const RobotFrame &rFrame) = 0;
    virtual RobotFrame readRobotFrame(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual RobotFrame getRobotFrame(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    
    // 修改坐标系
    virtual int modifyTool(ROBOTCONSYS_ARM_INDEX armIndex, const std::string& name) = 0;                    // 修改工具坐标系
    virtual int modifyToolFrame(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& toolFrame) = 0;    // 修改工具坐标系
    virtual int modifyWorkFrame(ROBOTCONSYS_ARM_INDEX armIndex, const std::string& name) = 0;               // 修改工件坐标系
    virtual int modifyWorkFrame(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& workFrame) = 0;    // 修改工具坐标系

    // 坐标系标定
    virtual int calibrateTCP(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::JointsList& js, robsoft::Terminal& t, double& pre) = 0;
    virtual int calibrateTCFZ(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::JointsList& js, const robsoft::Joints& jo, const robsoft::Joints& jz, robsoft::Terminal& t, double& pre) = 0;
    virtual int calibrateTCFX(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::JointsList& js, const robsoft::Joints& jo, const robsoft::Joints& jx, const robsoft::Joints& jz, robsoft::Terminal& t, double& pre) = 0;
    virtual int calibrateUSRF(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Point& po, const robsoft::Point& px, const robsoft::Point& py, robsoft::Terminal& t) = 0;

    // 运动指令
    virtual int searchZero(ROBOTCONSYS_ARM_INDEX armIndex, double vel = 0.05) = 0;  // 传感器回零
    virtual int returnZero(ROBOTCONSYS_ARM_INDEX armIndex, double vel = 0.1) = 0;   // 回到零位
    virtual int returnHome(ROBOTCONSYS_ARM_INDEX armIndex, double vel = 0.1) = 0;   // 回到初始位置

    // 单关节或单方向点动
    virtual int jointJOG(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::JOINTINDEX index, MOVEDIRECTION dir, double vel) = 0;  // 关节点动
    virtual int terminalJOG(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::TERMINALINDEX index, MOVEDIRECTION dir, double vel, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;   // 末端点动
    virtual int armAngleJOG(ROBOTCONSYS_ARM_INDEX armIndex, MOVEDIRECTION dir, double vel) = 0;    // 臂形角点动
    virtual int stopJOG(ROBOTCONSYS_ARM_INDEX armIndex) = 0; // 停止点动

    // 单关节或单方向速度点动
    virtual int startJointsJOGVel(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual int startTerminalJOGVel(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int setJOGVel(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Joints& j) = 0;
    virtual int setJOGVel(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& t) = 0;
    virtual int stopJOGVel(ROBOTCONSYS_ARM_INDEX armIndex) = 0;

    // 单关节或单方向步进
    virtual int jointStep(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::JOINTINDEX index, MOVEDIRECTION dir, double step, double vel) = 0;  // 关节步进
    virtual int terminalStep(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::TERMINALINDEX index, MOVEDIRECTION dir, double step, double vel, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;   // 末端步进
    virtual int armAngleStep(ROBOTCONSYS_ARM_INDEX armIndex, MOVEDIRECTION dir, double step, double vel) = 0;  // 臂形角步进

    // 关节空间规划关节点列
    virtual int moveABSJoint(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Joints& j, double vel) = 0;
    virtual int moveABSJoint(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::JointsList& js, double vel) = 0;
    virtual int moveABSJoint(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::JointsList& js, std::vector<double>& tm) = 0;
    virtual int moveABSJointR(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Joints& j, double vel) = 0;
    virtual int moveABSJointR(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::JointsList &js, double vel) = 0;
    virtual int moveABSJointR(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::JointsList &js, std::vector<double> &tm) = 0;

    // 关节空间规划末端点列
    virtual int moveJoint(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& p, double vel, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveJoint(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::TerminalList& ps, double vel, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveJointR(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& p, double vel, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveJointR(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::TerminalList& ps, double vel, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;

    // 直线圆弧规划
    virtual int moveLine(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::HomogeneousMatrix& m, double vel, double acc = 0.8, double jerk = 0.8, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveLine(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveLineR(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveCircle(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::CIRCLETYPE cir, const robsoft::Terminal& p1, const robsoft::Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveCircleR(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::CIRCLETYPE cir, const robsoft::Terminal& p1, const robsoft::Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveCircleSeg(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::CIRCLETYPE cir, const robsoft::Terminal& p1, const robsoft::Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveCircleRSeg(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::CIRCLETYPE cir, const robsoft::Terminal& p1, const robsoft::Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveCircleSegRun(ROBOTCONSYS_ARM_INDEX armIndex, int seg_index) = 0;

    // 臂形角规划
    virtual int moveArmAngle(ROBOTCONSYS_ARM_INDEX armIndex, double theta, double vel) = 0;

    // B样条规划
    virtual int moveCurve(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::CURVETYPE curve, const robsoft::TerminalList& ps, double vel, double acc, double jerk, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveCurveSeg(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::CURVETYPE curve, const robsoft::TerminalList& ps, double vel, double acc, double jerk, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveCurveSegRun(ROBOTCONSYS_ARM_INDEX armIndex, int seg_index) = 0;

    // 门运动
    virtual int moveJump(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::HomogeneousMatrix& m, double height, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveJump(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& p, double height, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;

    // 已规划点列运动
	virtual int moveJointSequence(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::JointsList& js) = 0;

    // 连续轨迹
    virtual int moveStartCon(ROBOTCONSYS_ARM_INDEX armIndex, int delay = 2) = 0;
    virtual int moveEndCon(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual int moveLineCon(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::HomogeneousMatrix& m, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveLineCon(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveLineRCon(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveCircleCon(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::CIRCLETYPE cir, const robsoft::Terminal& p1, const robsoft::Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;
    virtual int moveCircleRCon(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::CIRCLETYPE cir, const robsoft::Terminal& p1, const robsoft::Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, robsoft::COORDINATESYSTEM frame = robsoft::COORDINATE_BASE) = 0;

    // 基于力矩模式的零力拖动示教模式
    virtual int dragModeStart(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual int dragModeEnd(ROBOTCONSYS_ARM_INDEX armIndex) = 0;

    // 基于末端力传感器的柔顺控制模式
    virtual int compliantControlModeStart(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual int compliantControlModeEnd(ROBOTCONSYS_ARM_INDEX armIndex) = 0;

    // 视觉伺服模式
    virtual int visionServoModeStart(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual int visionServoModeEnd(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual int visionServoModeSetJointVel(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Joints& jointVel) = 0;
    virtual int visionServoModeSetTerminalVel(ROBOTCONSYS_ARM_INDEX armIndex, const robsoft::Terminal& terVel) = 0;    // 视觉伺服设置末端速度，相对于末端坐标系

    virtual int moveMotion(ROBOTCONSYS_ARM_INDEX armIndex, const MotionNode& motion) = 0;  // 运动指令统一处理函数
    virtual int getMotionCMDState(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual int waitMotionCMDFinish(ROBOTCONSYS_ARM_INDEX armIndex) = 0;  // 等待运动命令执行完成
    
    virtual int setJointZero(ROBOTCONSYS_ARM_INDEX armIndex, robsoft::JOINTINDEX index) = 0; // 置零
    virtual int setErrorClear(ROBOTCONSYS_ARM_INDEX armIndex) = 0;    // 清除错误
    virtual int setErrorClear() = 0;
    
    virtual void updateRobotMotion(ROBOTCONSYS_ARM_INDEX armIndex) = 0;   // 更新当前运动状态
    virtual robsoft::RobotMotion getRobotMotion(ROBOTCONSYS_ARM_INDEX armIndex) = 0;    // 获取当前运动状态

    virtual void updateArmAngle(ROBOTCONSYS_ARM_INDEX armIndex) = 0;                          // 更新臂形角
    virtual double getArmAngle(ROBOTCONSYS_ARM_INDEX armIndex) = 0;                           // 获取臂形角

    virtual void updateMotorServoState(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual void getMotorServoState(ROBOTCONSYS_ARM_INDEX armIndex, std::vector<SWITCHSTATE>& servoState) = 0;   // 获取伺服状态
    virtual void updateMotorMode(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual void getMotorControlMode(ROBOTCONSYS_ARM_INDEX armIndex, std::vector<DevLayer::ROBOTDEV_MOTOR_CONTROL_LOOP>& mode) = 0;    // 获取控制模式
    virtual void getMotorInputMode(ROBOTCONSYS_ARM_INDEX armIndex, std::vector<DevLayer::ROBOTDEV_MOTOR_INPUT_MODE>& mode) = 0;   // 获取输入模式

    // 夹爪
    virtual void setClawState(ROBOTCONSYS_ARM_INDEX armIndex, SWITCHSTATE state) = 0;  // 设置夹爪状态
    virtual void updateClawState(ROBOTCONSYS_ARM_INDEX armIndex) = 0;                         // 更新夹爪状态
    virtual SWITCHSTATE getClawState(ROBOTCONSYS_ARM_INDEX armIndex) = 0;            // 获取夹爪状态

    // 机器人状态
    virtual int setServoState(ROBOTCONSYS_ARM_INDEX armIndex, SWITCHSTATE state) = 0;
    virtual int setServoState(SWITCHSTATE state) = 0;
    virtual void updateRobotState(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual RobotState getRobotState(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual SWITCHSTATE getServoState(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual SWITCHSTATE getErrorState(ROBOTCONSYS_ARM_INDEX armIndex) = 0;
    virtual void getErrorCode(ROBOTCONSYS_ARM_INDEX armIndex, std::vector<uint64_t>& errCode) = 0;
    virtual int getMotionFinishErrorCode(ROBOTCONSYS_ARM_INDEX armIndex) = 0;

    // 运行状态
    virtual void setRunState(ROBOTCONSYS_ARM_INDEX armIndex, SYSRUNSTATE state) = 0;
    virtual void setRunState(SYSRUNSTATE state) = 0;
    virtual void setProgramRunState(SYSRUNSTATE state) = 0;

    // 机器人控制系统状态
    virtual void setPlayState(SYSPLAYSTATE state) = 0;
    virtual void setEStopState(SWITCHSTATE state) = 0;
    virtual void setPorgramFinishErrorCode(int errorCode) = 0;
    virtual void updateRobotConSysState() = 0;
    virtual RobotConSysState getRobotConSysState() = 0;
    virtual SWITCHSTATE getSystemState() = 0;
    virtual SYSPLAYSTATE getPlayState() = 0;
    virtual SWITCHSTATE getEStopState() = 0;
    virtual int getPorgramFinishErrorCode() = 0;

    // 移动机器人接口
    virtual void setVehicleConfig(const DevLayer::Vehicle_Config_Data& config) = 0;
    virtual DevLayer::Vehicle_Config_Data getVehicleConfig() = 0;
    virtual void updateVehicleState() = 0;
    virtual DevLayer::Vehicle_State_Data getVehicleState() = 0;
    virtual void updateVehicleIMUData() = 0;
    virtual DevLayer::Vehicle_IMU_Data getVehicleIMUData() = 0;

    virtual void setVehicleControlType(DevLayer::VEHICLE_CONTROL_TYPE controlType) = 0;
    virtual void setAutoCharge(SWITCHSTATE autoCharge) = 0;
    virtual void setVehicleServoState(SWITCHSTATE state) = 0;
    virtual void setMotionControlRocker(DevLayer::VEHICLE_MODE mode, float x_ratio, float y_ratio, float z_ratio) = 0;
    virtual void setMotionControl(DevLayer::VEHICLE_MODE mode, float x_vel, float y_vel, float z_vel) = 0;

    virtual void setDefaultParameter() = 0;
    virtual void readFlashParameter() = 0;
    virtual void saveParameter() = 0;
    virtual void saveServoMotorOffset(DevLayer::VEHICLE_SERVO_MOTOR_OFFSET index) = 0;
    virtual void setServoMotorOffsetAutoCalibrate(SWITCHSTATE state) = 0;

    // 本体IO设备
    virtual void setMotionIO(const RobotIO& rIO) = 0;
	virtual void setMotionDigitalInput(const DigitalState& state) = 0;              // 设置数字输入状态，仿真有效
    virtual void setMotionDigitalInput(PORTINDEX index, SWITCHSTATE state) = 0;     // 设置数字输入状态，仿真有效
	virtual void setMotionAnalogInput(const AnalogState& state) = 0;                // 设置模拟输入状态，仿真有效
    virtual void setMotionAnalogInput(PORTINDEX index, double state) = 0;           // 设置模拟输入状态，仿真有效
    virtual void setMotionDigitalOutput(const DigitalState& state) = 0;             // 设置数字输出状态
    virtual void setMotionDigitalOutput(PORTINDEX index, SWITCHSTATE state) = 0;    // 设置数字输出状态
    virtual void setMotionAnalogOutput(const AnalogState& state) = 0;               // 设置模拟输出状态
    virtual void setMotionAnalogOutput(PORTINDEX index, double state) = 0;          // 设置模拟输出状态
    virtual void updateMotionIO() = 0;                                              // 更新IO状态
    virtual RobotIO getMotionIO() = 0;                                              // 获取IO状态
    virtual SWITCHSTATE getMotionIOConnect() = 0;                                   // IO模块连接状态
    virtual DigitalState getMotionDigitalInput() = 0;                               // 获取数字输入状态
    virtual SWITCHSTATE getMotionDigitalInput(PORTINDEX index) = 0;                 // 获取数字输入状态
    virtual DigitalState getMotionDigitalOutput() = 0;                              // 获取数字输出状态
    virtual SWITCHSTATE getMotionDigitalOutput(PORTINDEX index) = 0;                // 获取数字输出状态
    virtual AnalogState getMotionAnalogInput() = 0;                                 // 获取模拟输入状态
    virtual double getMotionAnalogInput(PORTINDEX index) = 0;                       // 获取模拟输入状态
    virtual AnalogState getMotionAnalogOutput() = 0;                                // 获取模拟输出状态
    virtual double getMotionAnalogOutput(PORTINDEX index) = 0;                      // 获取模拟输出状态

    // 外部扩展设备
	// 外部IO设备
    virtual void setRobotIO(const RobotIO& rIO) = 0;
	virtual void setDigitalInput(const DigitalState& state) = 0;            // 设置数字输入状态，仿真有效
    virtual void setDigitalInput(PORTINDEX index, SWITCHSTATE state) = 0;   // 设置数字输入状态，仿真有效
	virtual void setAnalogInput(const AnalogState& state) = 0;              // 设置模拟输入状态，仿真有效
    virtual void setAnalogInput(PORTINDEX index, double state) = 0;         // 设置模拟输入状态，仿真有效
    virtual void setDigitalOutput(const DigitalState& state) = 0;           // 设置数字输出状态
    virtual void setDigitalOutput(PORTINDEX index, SWITCHSTATE state) = 0;  // 设置数字输出状态
    virtual void setAnalogOutput(const AnalogState& state) = 0;             // 设置模拟输出状态
    virtual void setAnalogOutput(PORTINDEX index, double state) = 0;        // 设置模拟输出状态
    virtual void updateRobotIO() = 0;                                       // 更新IO状态
    virtual RobotIO getRobotIO() = 0;                                       // 获取IO状态
    virtual SWITCHSTATE getIOConnect() = 0;                                 // IO模块连接状态
    virtual DigitalState getDigitalInput() = 0;                             // 获取数字输入状态
    virtual SWITCHSTATE getDigitalInput(PORTINDEX index) = 0;               // 获取数字输入状态
    virtual DigitalState getDigitalOutput() = 0;                            // 获取数字输出状态
    virtual SWITCHSTATE getDigitalOutput(PORTINDEX index) = 0;              // 获取数字输出状态
    virtual AnalogState getAnalogInput() = 0;                               // 获取模拟输入状态
    virtual double getAnalogInput(PORTINDEX index) = 0;                     // 获取模拟输入状态
    virtual AnalogState getAnalogOutput() = 0;                              // 获取模拟输出状态
    virtual double getAnalogOutput(PORTINDEX index) = 0;                    // 获取模拟输出状态

    // modbus
    virtual void setModbusData(MODBUS_DATA_TYPE type, uint16_t addr, uint16_t value) = 0;   // 设置单个Modbus数据
    virtual void updateModbusData() = 0;                                                    // 更新Modbus数据
    virtual ModbusData getModbusData() = 0;                                                 // 获取Modbus数据
    virtual uint16_t getModbusData(MODBUS_DATA_TYPE type, uint16_t addr) = 0;               // 获取单个Modbus数据

    // 外部TCP设备
    virtual void setPeripheralDevData_Joints(ROBOTCONSYS_PERIPHERAL_DATA_INDEX index, const robsoft::Joints& j) = 0;     // 设置外部设备Joints类型，index = 0,1,2,...,9
    virtual void setPeripheralDevData_Terminal(ROBOTCONSYS_PERIPHERAL_DATA_INDEX index, const robsoft::Terminal& t) = 0; // 设置外部设备Terminal类型，index = 0,1,2,...,9
    virtual robsoft::Joints getPeripheralDevData_Joints(ROBOTCONSYS_PERIPHERAL_DATA_INDEX index) = 0;                    // 获取外部设备Joints类型数据，index = 0,1,2,...,9
    virtual robsoft::Terminal getPeripheralDevData_Terminal(ROBOTCONSYS_PERIPHERAL_DATA_INDEX index) = 0;                // 获取外部设备Terminal类型数据，index = 0,1,2,...,9

    // 程序执行
    virtual void programLoad(const char* filePath) = 0;                 // 载入程序文件
    virtual void programSave(const char* filePath) = 0;                 // 保存程序文件
    virtual void programSetVel(double vel = 1) = 0;                     // 设置程序速度
    virtual void programSetDebugMode(bool mode) = 0;                    // 设置程序Debug模式
    virtual void programSetState(rclib::PROGRAM_STATE state) = 0;       // 设置程序状态
    virtual void programSetPointer(const ProgramPointer& pointer) = 0;  // 设置程序指针
    virtual ProgramPointer programGetPointer() = 0;                     // 获取程序指针
    virtual ParserErrorInfo programGetStopState() = 0;                  // 获取程序停止状态    
};
 
}

#endif