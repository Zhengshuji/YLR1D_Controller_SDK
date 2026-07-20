#ifndef VEHICLEMOTIONDEV_TYPEDEF_H
#define VEHICLEMOTIONDEV_TYPEDEF_H

#include <stdint.h>
#include "Common_Define.h"

namespace DevLayer{

enum VEHICLE_SERVO_MOTOR_OFFSET{VEHICLE_SERVO_MOTOR_OFFSET_1, VEHICLE_SERVO_MOTOR_OFFSET_2, VEHICLE_SERVO_MOTOR_OFFSET_3, VEHICLE_SERVO_MOTOR_OFFSET_4, VEHICLE_SERVO_MOTOR_OFFSET_ALL};

#define VEHICLE_PARAMETER_NUM 44
enum VEHICLE_PARAMETER_ADDR{ADDR_CONTROL_TYPE = 1, ADDR_VEHICLE_TYPE, ADDR_WHEEL_DISTANCE, ADDR_AXIS_DISTANCE, 
                            ADDR_TARGET_X_MAX, ADDR_TARGET_Y_MAX, ADDR_TARGET_Z_MAX, 
                            ADDR_TARGET_X_MAX_ACC, ADDR_TARGET_Y_MAX_ACC, ADDR_TARGET_Z_MAX_ACC,
                            ADDR_WHEEL_INPUT_MODE, 
                            ADDR_WHEEL_VEL_MAX, ADDR_WHEEL_ACC_MAX, ADDR_WHEEL_VEL_RES, ADDR_WHEEL_ACC_RES, 
                            ADDR_WHEEL_DIAMETER, ADDR_WHEEL_REDUCE_RATIO,
                            ADDR_WHEEL_DIR_1, ADDR_WHEEL_DIR_2, ADDR_WHEEL_DIR_3, ADDR_WHEEL_DIR_4,
                            ADDR_SERVO_INPUT_MODE, 
                            ADDR_SERVO_POS_MIN, ADDR_SERVO_POS_MAX, ADDR_SERVO_VEL_MAX, ADDR_SERVO_ACC_MAX, 
                            ADDR_SERVO_POS_RES, ADDR_SERVO_VEL_RES, ADDR_SERVO_ACC_RES, 
                            ADDR_SERVO_REDUCE_RATIO,
                            ADDR_SERVO_DIR_1, ADDR_SERVO_DIR_2, ADDR_SERVO_DIR_3, ADDR_SERVO_DIR_4,
                            ADDR_SERVO_OFFSET_1, ADDR_SERVO_OFFSET_2, ADDR_SERVO_OFFSET_3, ADDR_SERVO_OFFSET_4,
                            ADDR_SERVO_OFFSET_ANGLE_1, ADDR_SERVO_OFFSET_ANGLE_2, ADDR_SERVO_OFFSET_ANGLE_3, ADDR_SERVO_OFFSET_ANGLE_4,
                            ADDR_SERVO_CALIBRATE_TORQUE,
                            ADDR_CAN_MOTOR_ENABLE};

enum VEHICLE_TEST_PERIPHERAL{   VEHICLE_TEST_PERIPHERAL_IMU = 1, VEHICLE_TEST_PERIPHERAL_UART_CMD, VEHICLE_TEST_PERIPHERAL_SONAR, 
                                VEHICLE_TEST_PERIPHERAL_BLUETOOTH, VEHICLE_TEST_PERIPHERAL_UART_DEBUG, VEHICLE_TEST_PERIPHERAL_BATTERY,   // UART
                                VEHICLE_TEST_PERIPHERAL_LM75B,                                  // IIC
                                VEHICLE_TEST_PERIPHERAL_CAN_MOTOR, VEHICLE_TEST_PERIPHERAL_CAN_CMD,     // CAN
                                VEHICLE_TEST_PERIPHERAL_IA6B, VEHICLE_TEST_PERIPHERAL_IR, VEHICLE_TEST_PERIPHERAL_LED,      // PWM
                                VEHICLE_TEST_PERIPHERAL_EMC, VEHICLE_TEST_PERIPHERAL_AUTOCHARGE,        // IO
                                VEHICLE_TEST_PERIPHERAL_PRINT_PARAMETER, VEHICLE_TEST_PERIPHERAL_MOTION};

enum VEHICLE_CONTROL_TYPE{VEHICLE_CONTROL_REMOTE, VEHICLE_CONTROL_UART, VEHICLE_CONTROL_BLUETOOTH, VEHICLE_CONTROL_CAN, VEHICLE_CONTROL_ETH};
enum VEHICLE_TYPE{VEHICLE_DIFF_TWO, VEHICLE_ACKERMANN, VEHICLE_QUATTRO, VEHICLE_OMNI_QUATTRO, VEHICLE_MECANUM_QUATTRO, VEHICLE_TANK};
enum VEHICLE_MOTOR_INPUT_MODE{VEHICLE_MOTOR_INPUT_MODE_UNKNOWN, VEHICLE_MOTOR_PERIODIC_MODE, VEHICLE_MOTOR_CONTOUR_MODE};
enum VEHICLE_MODE{VEHICLE_MODE_NORMAL, VEHICLE_MODE_MOVE_XY, VEHICLE_MODE_MOVE_Y, VEHICLE_MODE_ROTATE, VEHICLE_MODE_PARKING};
enum VEHICLE_MOTOR_SERVO_STATE{VEHICLE_MOTOR_SERVO_STATE_OFF, VEHICLE_MOTOR_SERVO_STATE_ON};

#define VEHICLE_STATE_TYPE_NUM 8
enum VEHICLE_STATE_TYPE{VEHICLE_STATE_CONTROL = 0x11, VEHICLE_STATE_IMU_X, VEHICLE_STATE_IMU_Y, VEHICLE_STATE_IMU_Z, 
                        VEHICLE_STATE_SONAR, VEHICLE_STATE_WHEEL, VEHICLE_STATE_SERVO, VEHICLE_STATE_IR, 
                        VEHICLE_IMU_DATA = 0x1f};

#define VEHICLE_PI 3.14159265358979323846

struct DECLSPEC_DLLEXPORT Vehicle_Config_Data{
    enum VEHICLE_CONTROL_TYPE controlType;
    enum VEHICLE_TYPE vehicleType;

    float wheelDistance;                // mm
    float axisDistance;                 // mm

    float targetx_max;	                // mm/s
    float targety_max;	                // mm/s
    float targetz_max;	                // rad/s

    float targetx_max_acc;	            // mm/s^2
    float targety_max_acc;	            // mm/s^2
    float targetz_max_acc;	            // rad/s^2

    enum VEHICLE_MOTOR_INPUT_MODE wheelMotorInputMode;
    float wheelMotorVelMax;	            // mm/s
    float wheelMotorAccMax;	            // mm/s^2
    int16_t wheelMotorVelResolution;
    int16_t wheelMotorAccResolution;
    float wheelMotorDiameter;	        // mm
    float wheelMotorReduceRatio;
    int wheelMotorDirection[4];

    enum VEHICLE_MOTOR_INPUT_MODE servoMotorInputMode;
    float servoMotorPosMin;	            // rad
    float servoMotorPosMax;	            // rad
    float servoMotorVelMax;	            // rad/s
    float servoMotorAccMax;	            // rad/s^2
    int32_t servoMotorPosResolution;
    int16_t servoMotorVelResolution;
    int16_t servoMotorAccResolution;
    float servoMotorReduceRatio;
    int servoMotorDirection[4];
    int32_t servoMotorOffset[4];	    // pulse
    float servoMotorOffsetAngle[4];     // rad
    int16_t servoMotorCalibrateTorque;  // 0.1%

    uint8_t canMotorEnable;
};

struct DECLSPEC_DLLEXPORT Vehicle_State_Data{
    uint8_t batterySOC;                             // 电池电量，0.1%
    VEHICLE_MODE vehicleMode;                       // 运动模式
    VEHICLE_MOTOR_SERVO_STATE vehicleServoState;    // 使能状态
    float vehicle_x_vel;                            // mm/s
    float vehicle_y_vel;                            // mm/s
    float vehicle_z_vel;                            // rad/s

    float wheelVel[4];                              // rad/s
    float servoPos[4];                              // rad

    uint8_t sonar[8];                               // cm
    uint8_t irDutyCycle[2];                         // %
};

struct DECLSPEC_DLLEXPORT Vehicle_IMU_Data{
    int16_t ppsSyncStamp;
    int8_t temprature;
    float pressure;
    uint32_t timestamp;
    float acc[3];
    float gyr[3];
    float mag[3];
    float eul[3];
    float quat[4];
};

struct imu_data_t{
    uint8_t tag; 			/* item tag: 0x91 */
    uint32_t id; 			/* user define ID */
    int8_t temprature;	    /* imu temprature (℃) */
    float pressure;		    /* air pressure (Pa) */
    uint32_t timestamp;     /* time stamp (ms) */
    float acc[3]; 			/* acceleration xyz (1g) */
    float gyr[3]; 			/* angular velocity xyz (deg/s) */
    float mag[3]; 			/* magnetic field xyz (uT) */
    float eul[3]; 			/* attitude: eular angle xyz (deg) */
    float quat[4]; 			/* attitude: quaternion wxyz */
};

#define VEHICLE_SONAR_CHANNEL_NUM 8
#define VEHICLE_IA6B_CHANNEL_NUM 6
#define VEHICLE_IR_CHANNEL_NUM 2
struct Vehicle_Sensor_Data{
    imu_data_t imu_data;

    uint8_t sonar_dis[VEHICLE_SONAR_CHANNEL_NUM];

    uint16_t batteryCumulativeTotalVoltage;
    uint16_t batteryGatherTotalVoltage;
    uint16_t batteryCurrent;
    uint16_t batterySOC;

    int PWM_CONTROL_VAL[VEHICLE_IA6B_CHANNEL_NUM];
    uint8_t PWM_DUTY_CYCLE[VEHICLE_IR_CHANNEL_NUM];

    uint8_t emcStopState1, emcStopState2;
};

struct Vehicle_Process_Config_Data{
    uint8_t motorNum;

    float targetx_max_acc_period;
    float targety_max_acc_period;
    float targetz_max_acc_period;
    
    float wheelPerimeter;
    
    int16_t wheelMotorAccMaxPulse;
    int16_t servoMotorVelMaxPulse;
    int16_t servoMotorAccMaxPulse;
};

struct Vehicle_Motor{
    float wheelMotorVel[4];	// mm/s
    float servoMotorPos[4];	// rad
};

struct Vehicle_Motion{
    float vx = 0;   // mm/s
    float vy = 0;	// mm/s
    float wz = 0;	// rad/s
};

struct DECLSPEC_DLLEXPORT Vehicle_Control_Data{
    enum VEHICLE_MODE targetVehicleMode = VEHICLE_MODE_NORMAL;
    enum VEHICLE_MOTOR_SERVO_STATE targetServoState = VEHICLE_MOTOR_SERVO_STATE_OFF;
    struct Vehicle_Motion targetVehicleMotion;
};

}

#endif