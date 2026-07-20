#ifndef ROBOTCONSYS_STRUCT_H
#define ROBOTCONSYS_STRUCT_H

#include "CDataStructure.hpp"
#include "RobotDev/RobotDev_TypeDef.h"

namespace rclib{

enum SWITCHSTATE {SWITCHOFF, SWITCHON};
enum PORTINDEX {
    PORT_1, PORT_2, PORT_3, PORT_4, PORT_5, PORT_6, PORT_7, PORT_8,
    PORT_9, PORT_10, PORT_11, PORT_12, PORT_13, PORT_14, PORT_15, PORT_16
};

struct DECLSPEC_DLLEXPORT DigitalState{
    void init(int size);
    int size() const;

    SWITCHSTATE& operator[](PORTINDEX index);
    const SWITCHSTATE& operator[](PORTINDEX index) const;

    bool operator ==(const DigitalState &state) const;
    bool operator !=(const DigitalState &state) const;

    std::vector<SWITCHSTATE> m_state;
};

struct DECLSPEC_DLLEXPORT AnalogState{
    void init(int size);
    int size() const;

    double& operator[](PORTINDEX index);
    const double& operator[](PORTINDEX index) const;

    std::vector<double> m_state;
};

class DECLSPEC_DLLEXPORT RobotIO{
public:
    RobotIO();
    RobotIO(int DISize, int DOSize, int AISize, int AOSize);
    ~RobotIO();

    void setSize(int DISize, int DOSize, int AISize, int AOSize);

    SWITCHSTATE getIOConnectState() const;
    void setIOConnectState(SWITCHSTATE state);

    DigitalState getDigitalInputState() const;
    SWITCHSTATE getDigitalInputState(PORTINDEX index) const;
    void setDigitalInputState(const DigitalState& state);
    void setDigitalInputState(PORTINDEX index, SWITCHSTATE state);

	DigitalState getDigitalOutputState() const ;
	SWITCHSTATE getDigitalOutputState(PORTINDEX index) const ;
    void setDigitalOutputState(const DigitalState& state);
    void setDigitalOutputState(PORTINDEX index, SWITCHSTATE state);

    AnalogState getAnalogInputState() const;
    double getAnalogInputState(PORTINDEX index) const;
    void setAnalogInputState(const AnalogState& state);
    void setAnalogInputState(PORTINDEX index, double state);

    AnalogState getAnalogOutputState() const;
    double getAnalogOutputState(PORTINDEX index) const;
    void setAnalogOutputState(const AnalogState& state);
    void setAnalogOutputState(PORTINDEX index, double state);

private:
    SWITCHSTATE m_IOState;      // io连接状态
    DigitalState m_DIState;     // 数字输入状态
	DigitalState m_DOState;     // 数字输出状态
    AnalogState m_AIState;      // 模拟输入状态
    AnalogState m_AOState;      // 模拟输出状态
};

enum MODBUS_DATA_TYPE{
    MODBUS_DATA_BIT,
    MODBUS_DATA_INPUT_BIT,
    MODBUS_DATA_REG,
    MODBUS_DATA_INPUT_REG,
};
struct DECLSPEC_DLLEXPORT ModbusData{
    ModbusData() {
        m_bitVector.clear(); 
        m_bitInVector.clear();
        m_RegistVector.clear();
        m_RegistInVector.clear();
    }
    std::vector<uint8_t> m_bitVector;
    std::vector<uint8_t> m_bitInVector;
    std::vector<uint16_t> m_RegistVector;
    std::vector<uint16_t> m_RegistInVector;
};

enum SYSRUNSTATE {SYSRUN_RUN, SYSRUN_PAUSE, SYSRUN_STOP};
enum SYSPLAYSTATE {SYSPLAY_TEACH, SYSPLAY_PLAY, SYSPLAY_REMOTE};
enum SYSREMOTESTATE {SYSREMOTE_NONE = -1, SYSREMOTE_RUN, SYSREMOTE_STOP, SYSREMOTE_HOME};

class DECLSPEC_DLLEXPORT RobotState{
public:
    RobotState();
    ~RobotState();

    void setServoState(SWITCHSTATE state);
    SWITCHSTATE getServoState() const;
    
    void setErrorState(SWITCHSTATE state);
    SWITCHSTATE getErrorState() const;

    void setErrorCode(const std::vector<uint64_t>& errorCode);
    std::vector<uint64_t> getErrorCode() const;

    void setMotionFinishErrorCode(int errorCode);
    int getMotionFinishErrorCode() const;
    
private:
    SWITCHSTATE m_servoStateFlag;       // 伺服状态

    SWITCHSTATE m_errorStateFlag;       // 驱动器错误状态
    std::vector<uint64_t> m_errorCode;  // 驱动器错误码

    int m_motionFinishErrorCode;   // 运动终止错误码
};

class DECLSPEC_DLLEXPORT RobotConSysState{
public:
    RobotConSysState();
    ~RobotConSysState();

    void setSystemState(SWITCHSTATE state);
    SWITCHSTATE getSystemState() const;
    
    void setPlayState(SYSPLAYSTATE state);
    SYSPLAYSTATE getPlayState() const;
    
    void setEstopState(SWITCHSTATE state);
    SWITCHSTATE getEStopState() const;

    void setPorgramFinishErrorCode(int errorCode);
    int getPorgramFinishErrorCode() const;
    
private:
    SWITCHSTATE m_sysStateFlag;     // 系统状态
    SYSPLAYSTATE m_playStateFlag;   // 运行模式
    SWITCHSTATE m_estopStateFlag;   // 急停状态
    int m_programFinishErrorCode;   // 程序终止错误码
};

class DECLSPEC_DLLEXPORT RobotFrame{
public:
    RobotFrame();
    ~RobotFrame();
    void readRobotFrame(const char* path);
    void writeRobotFrame(const char* path);

    std::string getDefaultName() const;
    std::string getTempName() const;

    std::map<std::string, robsoft::Tool> getToolList() const;
    std::string getCurrentTool() const;
    robsoft::Tool getTool(const std::string& name);
    int addTool(const std::string& name, const robsoft::Tool& tool);
    int modifyTool(const std::string& name, const robsoft::Tool& tool);
    int deleteTool(const std::string& name);
    void clearTool();
    int setCurrentTool(const std::string& name);
    void setDefaultTool();

    std::map<std::string, robsoft::Terminal> getWorkFrameList() const;
    std::string getCurrentWorkFrame() const;
    robsoft::Terminal getWorkFrame(const std::string& name);
    int addWorkFrame(const std::string& name, const robsoft::Terminal& frame);
    int modifyWorkFrame(const std::string& name, const robsoft::Terminal& frame);
    int deleteWorkFrame(const std::string& name);
    void clearWorkFrame();
    int setCurrentWorkFrame(const std::string& name);
    void setDefaultWorkFrame();

    bool operator ==(RobotFrame& frame);
    bool operator !=(RobotFrame& frame);

private:
    std::string m_currentTool;
    std::string m_currentWorkFrame;
    std::map<std::string, robsoft::Tool> m_toolList;
    std::map<std::string, robsoft::Terminal> m_workFrameList;

    std::string m_defaultName = "Default";
    std::string m_tempName = "Temp";
};

class DECLSPEC_DLLEXPORT RobotPreference{
public:
    RobotPreference();
    ~RobotPreference();

    void alignDOF(int dof);

    void readRobotPreference(const char* path);
    void writeRobotPreference(const char* path) const;

    void setJointInitPosition(const robsoft::Joints& joint);
    robsoft::Joints getJointInitPosition() const;

    void setJointCompensation(const robsoft::Joints& joint);
    robsoft::Joints getJointCompensation() const;
    
    void setJointSearchSequence(const robsoft::Joints& joint);
    robsoft::Joints getJointSearchSequence() const;

    void setChannelIndex(DevLayer::ROBOTDEV_MOTOR_CHANNEL_INDEX index);
    DevLayer::ROBOTDEV_MOTOR_CHANNEL_INDEX getChannelIndex() const;

    void setMasterIndex(DevLayer::ROBOTDEV_MOTOR_MASTER_INDEX index);
    DevLayer::ROBOTDEV_MOTOR_MASTER_INDEX getMasterIndex() const;

    void setMasterIDStart(uint8_t id);
    uint8_t getMasterIDStart() const;

    void setClawEnableState(SWITCHSTATE state);
    SWITCHSTATE getClawEnableState() const;

    void setClawType(DevLayer::ROBOTDEV_CLAW_TYPE type);
    DevLayer::ROBOTDEV_CLAW_TYPE getClawType() const;

    void setClawMasterIndex(DevLayer::ROBOTDEV_MOTOR_MASTER_INDEX index);
    DevLayer::ROBOTDEV_MOTOR_MASTER_INDEX getClawMasterIndex() const;

    void setClawID(uint8_t id);
    uint8_t getClawID() const;

    void setClawVelOutput(double clawVelOutput);
    double getClawVelOutput() const;

    void setClawVelRamp(double clawVelRamp);
    double getClawVelRamp() const;

    void setClawTorLimit(double clawTorLimit);
    double getClawTorLimit() const;

    void setClawPosResolution(double resolution);
    double getClawPosResolution() const;

    void setClawVelResolution(double resolution);
    double getClawVelResolution() const;

    void setClawTorResolution(double resolution);
    double getClawTorResolution() const;

    void setClawReduceRatio(double reduceRatio);
    double getClawReduceRatio() const;

    void setClawPitch(double pitch);
    double getClawPitch() const;

    bool isDevSame(const RobotPreference& robotPreference) const;

private:
    robsoft::Joints m_jointInitPosition;     // 关节初始位置
    robsoft::Joints m_jointCompensation;     // 关节补偿
    robsoft::Joints m_jointSearchSequence;   // 关节零位搜索顺序

    DevLayer::ROBOTDEV_MOTOR_CHANNEL_INDEX m_channelIndex;  // 通讯通道序号
    DevLayer::ROBOTDEV_MOTOR_MASTER_INDEX m_masterIndex;    // 通讯主站序号
    uint8_t m_masterIDStart;    // 通讯主站开始ID

    SWITCHSTATE m_clawEnable;
    DevLayer::ROBOTDEV_CLAW_TYPE m_clawType;                    // 夹爪类型
    DevLayer::ROBOTDEV_MOTOR_MASTER_INDEX m_clawMasterIndex;    // 夹爪通讯主站序号
    uint8_t m_clawID;                                           // 夹爪主站ID
    double m_clawVelOutput;         // 夹爪速度输出，m/s
    double m_clawVelRamp;           // 夹爪速度攀升，m/s^2
    double m_clawTorLimit;          // 夹爪力矩限制，Nm
    double m_clawPosResolution;     // 夹爪位置分辨率
    double m_clawVelResolution;     // 夹爪速度分辨率
    double m_clawTorResolution;     // 夹爪力矩分辨率
    double m_clawReduceRatio;       // 夹爪减速比
    double m_clawPitch;             // 夹爪螺距
};

enum ROBOTMODEL {   ROBOTMODEL_HI_ARM_3 = 0x00, ROBOTMODEL_HI_ARM_4, ROBOTMODEL_HI_ARM_6, ROBOTMODEL_HI_ARM_7,
                    ROBOTMODEL_HI_ARM_DUAL_17 = 0x10,
                    ROBOTMODEL_HI_VEHICLE_DIFF_TWO = 0x20, ROBOTMODEL_HI_VEHICLE_ACKERMANN, ROBOTMODEL_HI_VEHICLE_QUATTRO, 
                    ROBOTMODEL_HI_VEHICLE_OMNI_QUATTRO, ROBOTMODEL_HI_VEHICLE_MECANUM_QUATTRO, ROBOTMODEL_HI_VEHICLE_TANK,
                    ROBOTMODEL_HI_ROBOT_SINGLE_13 = 0x30, 
                    ROBOTMODEL_HI_ROBOT_DUAL_26 = 0x40,
                    ROBOTMODEL_ABB_IRB120 = 0x50,
                    ROBOTMODEL_FANUC_LRMATE200ID4S = 0x60, 
                    ROBOTMODEL_KUKA_KR3R540 = 0x70, ROBOTMODEL_KUKA_KR4R600 = 0x71, 
                    ROBOTMODEL_YASKAWA_GP4 = 0x80,
                    ROBOTMODEL_ARM_TEST = 0x100};
enum ROBOTDEV_TYPE{
    ROBOTDEV_SIM = 0X00,    // 仿真  
    ROBOTDEV_AXI = 0X10,    // ZYNQ控制器
    ROBOTDEV_CAN = 0X20,    // CAN通讯
    ROBOTDEV_ECAT = 0X30,   // EtherCAT通讯
};
enum VEHICLEDEV_TYPE{
    VEHICLEDEV_SIM = 0x00,  // 仿真
    VEHICLEDEV_UART = 0x10, // UART
    VEHICLEDEV_UDP = 0x20,  // UDP
};
class DECLSPEC_DLLEXPORT RobotConSysPreference{
public:
    RobotConSysPreference();
    ~RobotConSysPreference();

    void readRobotConSysPreference(const char* path);
    void writeRobotConSysPreference(const char* path) const;

    void setRobotName(const std::string& name);
    std::string getRobotName() const;

    void setRobotModel(ROBOTMODEL model);
    ROBOTMODEL getRobotModel() const;

    void setRobotDevType(ROBOTDEV_TYPE type);
    ROBOTDEV_TYPE getRobotDevType() const;

    void setRobotDevPeriod(uint32_t period_us);
    uint32_t getRobotDevPeriod() const;

    void setRobotDevChannelNum(uint8_t num);
    uint8_t getRobotDevChannelNum() const;

    void setIOEnableState(SWITCHSTATE state);
    SWITCHSTATE getIOEnableState() const;

    void setDINum(uint8_t num);
    uint8_t getDINum() const;

    void setDONum(uint8_t num);
    uint8_t getDONum() const;

    void setAINum(uint8_t num);
    uint8_t getAINum() const;

    void setAONum(uint8_t num);
    uint8_t getAONum() const;

    void setMotionIOEnableState(SWITCHSTATE state);
    SWITCHSTATE getMotionIOEnableState() const;

    void setMotionDINum(uint8_t num);
    uint8_t getMotionDINum() const;

    void setMotionDONum(uint8_t num);
    uint8_t getMotionDONum() const;

    void setMotionAINum(uint8_t num);
    uint8_t getMotionAINum() const;

    void setMotionAONum(uint8_t num);
    uint8_t getMotionAONum() const;

    void setEStopEnableState(SWITCHSTATE state);
    SWITCHSTATE getEStopEnableState() const;

    void setModbusSlavePort(int port);
    int getModbusSlavePort() const;

    void setVehicleDevChannelNum(uint8_t num);
    uint8_t getVehicleDevChannelNum() const;

    void setVehicleDevType(VEHICLEDEV_TYPE type);
    VEHICLEDEV_TYPE getVehicleDevType() const;

    void setVehicleDevPeriod(uint32_t period_us);
    uint32_t getVehicleDevPeriod() const;

    void setVehicleDevIP(const std::string& ip);
    std::string getVehicleDevIP() const;

    void setVehicleDevPort(int port);
    int getVehicleDevPort() const;

    void setVehicleDevIMUOnBoardEnable(SWITCHSTATE state);
    SWITCHSTATE getVehicleDevIMUOnBoardEnable() const;

    void setInitPlayState(SYSPLAYSTATE state);
    SYSPLAYSTATE getInitPlayState() const;

    bool isDevSame(const RobotConSysPreference& robotConSysPreference) const;

private:
    std::string m_robotName;    // 机器人名称
    ROBOTMODEL m_robotModel;    // 机器人构型

    ROBOTDEV_TYPE m_robotDevType;   // 机器人设备类型
    uint32_t m_robotDevPeriod_us;   // 手臂控制周期
    uint8_t m_robotDevChannelNum;   // 手臂通讯缓存通道数量
    
    SWITCHSTATE m_ioEnable;     // io模块使能状态
    uint8_t m_nDI;              // DI数量
    uint8_t m_nDO;              // DO数量
    uint8_t m_nAI;              // AI数量
    uint8_t m_nAO;              // AO数量

    SWITCHSTATE m_motionIOEnable;   // io模块使能状态
    uint8_t m_nMotionDI;            // DI数量
    uint8_t m_nMotionDO;            // DO数量
    uint8_t m_nMotionAI;            // AI数量
    uint8_t m_nMotionAO;            // AO数量

    SWITCHSTATE m_estopEnable;  // estop使能状态

    int m_modbusSlavePort;              // modbus Slave端口号
    
    uint8_t m_vehicleDevChannelNum;             // 移动机器人通道数
    VEHICLEDEV_TYPE m_vehicleDevType;           // 移动机器人设备类型
    uint32_t m_vehicleDevPeriod_us;             // 移动机器人控制周期
    std::string m_vehicleDevIP;                 // 移动机器人IP
    int m_vehicleDevPort;                       // 移动机器人端口号
    SWITCHSTATE m_vehicleDevIMUOnBoardEnable;   // 移动机器人板载IMU使能

    SYSPLAYSTATE m_initPlayState;           // 初始化运行状态
};

enum MOVEDIRECTION {MOVE_NEGATIVE, MOVE_POSITIVE};
enum MOTIONTYPE {
    MOTION_SEARCH_ZERO, MOTION_RETURN_ZERO, MOTION_RETURN_HOME,
    MOTION_JOG_J, MOTION_JOG_T, MOTION_JOG_A, MOTION_JOG_STOP,
    MOTION_JOG_VEL_START_J, MOTION_JOG_VEL_START_T, MOTION_JOG_VEL_START_A,
    MOTION_JOG_VEL_J, MOTION_JOG_VEL_T, MOTION_JOG_VEL_A, MOTION_JOG_VEL_STOP,
    MOTION_STEP_J, MOTION_STEP_T, MOTION_STEP_A,
    MOTION_MOVEABSJ, MOTION_MOVEABSJR, MOTION_MOVEABSJ_TIME, MOTION_MOVEABSJR_TIME,
    MOTION_MOVEJ, MOTION_MOVEJR,
    MOTION_MOVEL, MOTION_MOVELR, 
    MOTION_MOVEC, MOTION_MOVECR,
    MOTION_MOVEC_SEG, MOTION_MOVECR_SEG, MOTION_MOVEC_SEG_RUN,
    MOTION_MOVE_ARM_ANGLE,
    MOTION_MOVEB, MOTION_MOVEB_SEG, MOTION_MOVEB_SEG_RUN,
    MOTION_MOVEJUMP, MOTION_MOVE_JSEQUENCE,
    MOTION_START_CON, MOTION_END_CON, MOTION_MOVEL_CON, MOTION_MOVELR_CON, MOTION_MOVEC_CON, MOTION_MOVECR_CON, 

    MOTION_DRAG_MODE_START = 100, MOTION_DRAG_MODE_END,
    MOTION_COMPLIANT_CONTROL_MODE_START = 110, MOTION_COMPLIANT_CONTROL_MODE_END,
    MOTION_VISION_SERVO_MODE_START = 120, MOTION_VISION_SERVO_MODE_END, MOTION_VISION_SERVO_MODE_SET_JOINTS_VEL, MOTION_VISION_SERVO_MODE_SET_TERMINAL_VEL,
};
class DECLSPEC_DLLEXPORT MotionNode{
public:
    MotionNode();
    ~MotionNode();
    void print(const char* str = "MotionNode") const;

    MOTIONTYPE motionType;
    robsoft::JOINTINDEX jIndex;
    robsoft::TERMINALINDEX tIndex;
    double step;                    // 步进距离或者门运动高度
    double accRatio;
    double jerkRatio;
    double angleRatio;
    double precision;               // 连续轨迹转弯半径，B样条精度
    robsoft::COORDINATESYSTEM referFrame;
    robsoft::CIRCLETYPE circleType;
    robsoft::CURVETYPE curveType;
    int delayNum;                   // 连续轨迹延迟段数
    std::vector<double> velList;
    robsoft::JointsList jointList;
    robsoft::TerminalList terminalList;
};

}

#endif
