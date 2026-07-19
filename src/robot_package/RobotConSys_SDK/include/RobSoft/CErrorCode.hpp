#ifndef CERRORCODE_HPP
#define CERRORCODE_HPP

#include "RMacroDefine.hpp"
#include <string>

enum {
    ERROR_NONE                          = 0,    // 无错误
    ERROR_NO_INVERSE_KINEMATICS,                // 无逆解
    ERROR_INVERVE_KINEMATICS_OVER_ITERATION,    // 求解超出最大迭代次数
    ERROR_OVERRANGE,                            // 超出限制位置
    ERROR_OVERVELOCITY,                         // 超出最大速度
    ERROR_OVERACCELERATION,                     // 超出最大加速度
    ERROR_PARAMETER_DRIVER_CHANGE,              // 驱动器参数变更，需重启
    ERROR_TOOL_NAME_INVALID,                    // 非法的工具名称
    ERROR_TOOL_EXIST,                           // 工具已存在
    ERROR_TOOL_NOT_EXIST,                       // 工具不存在
    ERROR_MODIFY_DEFAULT_TOOL,                  // 默认工具不可修改
    ERROR_DELETE_DEFAULT_TOOL,                  // 默认工具不可删除
    ERROR_WORKFRAME_NAME_INVALID,               // 非法的工件名称 
    ERROR_WORKFRAME_EXIST,                      // 工件已存在
    ERROR_WORKFRAME_NOT_EXIST,                  // 工件不存在
    ERROR_MODIFY_DEFAULT_WORKFRAME,             // 默认工件不可修改
    ERROR_DELETE_DEFAULT_WORKFRAME,             // 默认工件不可删除
    ERROR_CALIBRATE_SAMPLE_NUM,                 // 错误的工具标定采样点数
    ERROR_SERVO_OFF,                            // 伺服未使能
    ERROR_SERVO_ON,                             // 伺服已使能
    ERROR_ROBOT_DEV,                            // 机器人设备报错
    ERROR_EMERGENCY_STOP,                       // 急停已打开
    ERROR_CMD_CONFLICT,                         // 指令冲突
    ERROR_CON_TRAJ_NOT_START,                   // 连续轨迹未开始
    ERROR_CON_TRAJ_NOT_END,                     // 连续轨迹未结束
    ERROR_MOTION_STOP,                          // 运动中止
    ERROR_NO_SUCH_FILE,                         // 没有这个文件
    ERROR_OPEN_FILE,                            // 打开文件失败

    ERROR_ROBOT_DEV_CONNECTION_INCOMPLETE = 80, // 机器人设备未连接

    ERROR_INVALID_KEYWORD               = 100,  // 无效的关键字
    ERROR_CAMERA_CONNECT,                       // 相机未连接
    ERROR_IO_CONNECT,                           // IO未连接
    ERROR_NO_FUNCTION,                          // 函数不存在
    ERROR_NO_MATCH_FUNCTION,                    // 函数不匹配
    ERROR_SERVER_POINT_NULL,                    // 服务器指针错误

    ERROR_UNDEFINE_VARIABLE             = 200,  // 未定义变量
    ERROR_VARIABLE_ASSIGNMENT,                  // 非法的变量赋值
    ERROR_POINTER_ACCESS_VIOLATION,             // 函数指针访问越界
    ERROR_SYNTAX_ERROR,                         // 语法错误
    ERROR_OPERATION_ASSIGNMENT,                 // 不同类型的变量
    ERROR_NULL_SENTENCE,                        // 空语句
    ERROR_VARIABLE_DEFINITION,                  // 变量定义错误
    ERROR_VARIABLE_REDEFINITION,                // 变量重复定义
    ERROR_FUNCTION_REDEFINITION,                // 函数重复定义

    ERROR_INVALID_DEV_CHANNEL = 1900,

    ERROR_PERMISSION_DENIED             = 2000, // 无权限
};
 
DECLSPEC_DLLEXPORT std::string get_error_string(int robErrno, int g_intLanguageType = 0);

#endif
