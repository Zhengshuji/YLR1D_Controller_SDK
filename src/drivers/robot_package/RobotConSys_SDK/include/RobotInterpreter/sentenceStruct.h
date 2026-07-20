#ifndef SENTENCE_StrUCT_H
#define SENTENCE_StrUCT_H
#include <string>
#include <vector>
namespace rclib {

enum TeachType{HR,ABB,FANUC};

enum PROGRAM_STATE{PROGRAM_RUN, PROGRAM_STOP, PROGRAM_PAUSE, PROGRAM_CONTINUE, PROGRAM_STEP};

enum PROGRAM_KEY_WORD: unsigned int;
/// @brief 暂时不用
// struct ProgramVarible{
//     std::string Name;
//     PROGRAM_KEY_WORD Type;
//     union _Value
//     {
//         /* data */
//         std::string BOOL;
//         std::vector<double> Vector_Doble;
//         std::vector<std::string> Vector_String;
//         public:
//             _Value(){
//                 //new (&BOOL) std::string();
//                 new (&Vector_Doble) std::vector<double>();
//                 //new (&Vector_String) std::vector<std::string>();
//             }
//             ~_Value(){
//                 //BOOL.~basic_string();
//                 Vector_Doble.~vector();
//                 // Vector_String.~vector();
//             }
//     }Value;
    
//};    
// 程序语句
struct ProgramSentence{ 
    std::string strSentence;
    ProgramSentence(){}
    ProgramSentence(std::string str)
        :strSentence(str)
    {}

};

typedef std::vector<ProgramSentence> VECTOR_SENTENCE;

// 程序指针
struct ProgramPointer{  
    int nFunction;      //函数序号
    int nSentence;      //语句序号
};

/// 程序解析错误 
struct ParserErrorInfo{
    int nErrno;              //错误号
    std::string strInfo;    //错误信息
};

/// @brief 关键字
enum PROGRAM_KEY_WORD : unsigned int{
    VALUE, BOOL_,INT_, DOUBLE_, JOINT, TERMINAL, 
	BOOL_VECTOR, INT_VECTOR, DOUBLE_VECTOR, JOINT_VECTOR, TERMINAL_VECTOR, 
	CALCU, MAIN, FUNC, ENDMAIN, ENDFUNC, RETURN, CALL,
    FOR, ENDFOR, WHILE, ENDWHILE, CONTINUE, BREAK,
    IF, ELSEIF, ELSE, ENDIF, GOTO, LABLE, 
    WAITDIN, DOUT, AOUT, AIN, DIN,
    DELAY, COM,
    FRAME, TOOLFRAME, USERFRAME,
    MOVEABSJ, MOVEABSJR, MOVEJ, MOVEJR, MOVEL, MOVELR, MOVEC, MOVECR,
    MOVEB, MOVEJT,
    TOOL,WORK,VLOCATE, ROTATE180X,
    //sb3
    SET_JOINTS,SET_TERMINAL, SET_JOINT_BY_CURR, SET_TERMINAL_BY_CURR,
    //ABB
    VAR,LOCAL,TASK,NUM,STRING,PERS,
    ORIENT,POS,POSE,CONFDATA,ROBTARGET,SIGNAKXX,TOOLDATA,WOBJDATA
    ,PROC_,TRAP
    ,ACCSET, ADD, COMMENT, PULSEDO, SET, RESET, SETDO,TEST,VELSET, WAITDI,WAITDO,WAITTIME
    //fanuc
    ,J ,L, C, A, PR, R, P, OPERATOR, DO, DI, RO, RI, AO, AI, WAIT, JMP, SKIP, OFFSET
    , UTOOL, UFRAME, UTOOL_NUM, UFRAME_NUM, SELECT,
    //vision && modbus
    VJOINT,VTERMINAL,
    MB_BIT_R, MB_REG_R, MB_IN_BIT_R,MB_IN_REG_R , 
    MB_BIT_W, MB_REG_W, MB_IN_BIT_W, MB_IN_REG_W, MB_WAIT_BIT,
    TOOL_CLAW,SET_VEL,WAIT_MOVE_FINISH,
};

/// @brief 运算符
enum PROGRAM_OPERATOR{
    OPERATOR_ASS,   // =
    OPERATOR_ASS1,  // :=
    OPERATOR_ADD,   // +
    OPERATOR_SUB,   // -
    OPERATOR_MUL,   // *
    OPERATOR_DIV,   // /
    OPERATOR_MOD,   // %
    OPERATOR_EQU,   // ==
    OPERATOR_NEQ,   // !=
    OPERATOR_LAG,   // >
    OPERATOR_LES,   // <
    OPERATOR_GEQ,   // >=
    OPERATOR_LEQ    // <=
};

/// @brief 语言
enum ERROR_INIF_LANGUAGE{
    CHINESE,
    ENGLISH
};

/// @brief For语句结构
struct ForCount
{
    int nTotal=0;
    int nCompletedTime=0;

    int nBegin=0;     //开始
    int nEnd=0;       //结束
    int nStep=1;      //步长
    int nWalker=0;    //行者，当前值

	std::string name;	//变量名
    bool    step()
    {
		nWalker += nStep; 
		return (nStep > 0) ? (nWalker > nEnd) : (nWalker < nEnd);
    }
};

}
#endif