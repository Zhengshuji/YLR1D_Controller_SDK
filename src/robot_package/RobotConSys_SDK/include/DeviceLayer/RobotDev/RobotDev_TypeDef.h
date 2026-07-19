#ifndef ROBOTDEV_TYPEDEF_H
#define ROBOTDEV_TYPEDEF_H

#include <stdint.h>

namespace DevLayer{

#define ROBOTDEV_CACHE_NUM 3

#define ROBOTDEV_MASTER_MAX_INDEX 4  // 最大的主站ID
enum ROBOTDEV_MOTOR_MASTER_INDEX{ROBOTDEV_MOTOR_MASTER_1, ROBOTDEV_MOTOR_MASTER_2, ROBOTDEV_MOTOR_MASTER_3, ROBOTDEV_MOTOR_MASTER_4};   // 通信主站ID
enum ROBOTDEV_MOTOR_CHANNEL_INDEX{ROBOTDEV_MOTOR_CHANNEL_1, ROBOTDEV_MOTOR_CHANNEL_2, ROBOTDEV_MOTOR_CHANNEL_3, ROBOTDEV_MOTOR_CHANNEL_4};  // 数据缓存通道序号

struct RobotDev_Config{
    uint32_t period_us;
    uint8_t nChannel;
};


enum ROBOTDEV_MOTOR_SERVO_STATE{ROBOTDEV_MOTOR_SERVO_STATE_IDLE, ROBOTDEV_MOTOR_SERVO_STATE_CLOSELOOP};
enum ROBOTDEV_MOTOR_CONTROL_LOOP{ROBOTDEV_MOTOR_CONTROL_LOOP_UNKNOWN, ROBOTDEV_MOTOR_CONTROL_LOOP_POSTION, ROBOTDEV_MOTOR_CONTROL_LOOP_VELOCITY, ROBOTDEV_MOTOR_CONTROL_LOOP_TORQUE};
enum ROBOTDEV_MOTOR_INPUT_MODE{ROBOTDEV_MOTOR_INPUT_MODE_UNKNOWN, ROBOTDEV_MOTOR_PERIODIC_MODE, ROBOTDEV_MOTOR_CONTOUR_MODE};
struct RobotDev_Motor_struct{
    int32_t pos = 0;
    int32_t vel = 0;
    int16_t tor = 0;
};
#define MAX_MOTOR_NUM 14    // 单通道最大电机数
struct RobotDev_Motor_Config{
    uint8_t nMotor;
    ROBOTDEV_MOTOR_MASTER_INDEX master;
    uint8_t idStart;    // 起始ID，从0开始
};
struct RobotDev_Motor_ServoState{
    ROBOTDEV_MOTOR_SERVO_STATE servoState[MAX_MOTOR_NUM];

    RobotDev_Motor_ServoState& operator=(const RobotDev_Motor_ServoState& state){
        for(int i=0; i<MAX_MOTOR_NUM; i++){
            servoState[i] = state.servoState[i];
        }
        return *this;
    }
};
struct RobotDev_Motor_Mode{
    ROBOTDEV_MOTOR_CONTROL_LOOP controlMode[MAX_MOTOR_NUM];
    ROBOTDEV_MOTOR_INPUT_MODE inputMode[MAX_MOTOR_NUM];
    
    RobotDev_Motor_Mode& operator=(const RobotDev_Motor_Mode& mode){
        for(int i=0; i<MAX_MOTOR_NUM; i++){
            controlMode[i] = mode.controlMode[i];
            inputMode[i] = mode.inputMode[i];
        }
        return *this;
    }
};
struct RobotDev_Motor_Data{
    RobotDev_Motor_struct data[MAX_MOTOR_NUM];

    RobotDev_Motor_Data& operator=(const RobotDev_Motor_Data& val){
        for(int i=0; i<MAX_MOTOR_NUM; i++){
            data[i] = val.data[i];
        }
        return *this;
    }
};
struct RobotDev_Motor_Error{
    bool errorClear[MAX_MOTOR_NUM];
    uint64_t errorCode[MAX_MOTOR_NUM];

    RobotDev_Motor_Error& operator=(const RobotDev_Motor_Error& error){
        for(int i=0; i<MAX_MOTOR_NUM; i++){
            errorClear[i] = error.errorClear[i];
            errorCode[i] = error.errorCode[i];
        }
        return *this;
    }
};


enum ROBOTDEV_CLAW_TYPE{
    ROBOTDEV_CLAW_SIM_TRC12 = 0x00,
    ROBOTDEV_CLAW_SIM_RH56 = 0x01,
    ROBOTDEV_CLAW_CAN_TRC12 = 0x10,
    ROBOTDEV_CLAW_RS485_RH56 = 0x20,
};
struct RobotDev_Claw_Config{
    bool enable;                        // 夹爪使能
    ROBOTDEV_CLAW_TYPE type;            // 夹爪类型
    ROBOTDEV_MOTOR_MASTER_INDEX master; // 夹爪通信主站
    uint8_t id;                         // 夹爪id
    short velOutput;                    // 夹爪速度输出
    short velRamp;                      // 夹爪速度爬升
    short torLimit;                     // 夹爪力矩限制
};
struct RobotDev_Claw_Data{
    bool state;
};


#define MAX_AO_NUM 4
#define MAX_AI_NUM 4
struct RobotDev_IO_Config{
    bool enable;    // 设备IO使能状态
    uint8_t nDI;    // 设备数字输入端口数
    uint8_t nDO;    // 设备数字输出端口数
    uint8_t nAI;    // 设备模拟输入端口数
    uint8_t nAO;    // 设备模拟输出端口数
};
struct RobotDev_IO_Data{
    bool ioConnect;
    uint32_t DI;
    uint32_t DO;
    float AI[MAX_AI_NUM];
    float AO[MAX_AO_NUM];

    RobotDev_IO_Data& operator=(const RobotDev_IO_Data& val){
        ioConnect = val.ioConnect;
        DI = val.DI;
        DO = val.DO;
        for(int i=0; i<MAX_AI_NUM; i++){
            AI[i] = val.AI[i];
        }
        for(int i=0; i<MAX_AO_NUM; i++){
            AO[i] = val.AO[i];
        }
        return *this;
    }
};


struct RobotDev_StateIO_Config{
    bool eStopEnable;   // 急停使能
};
struct RobotDev_StateIO_Data{
    uint8_t stateInput;     // 获取急停和远程状态，bit 4-0: EMR1(急停) EMR(急停) RST(重置) STO(停止) STA(运行)
    uint16_t stateOutput;   // 设置系统状态，点亮状态指示灯，bit 8-0: SVO(伺服) SYS(系统) REM(远程) PLY(再现) TEA(示教) SSA(停止) PSA(暂停) RSA(运行) ERR(错误)
};

}

#endif