#ifndef CDATASTRUCTURE_HPP
#define CDATASTRUCTURE_HPP

#include <vector>
#include <list>
#include <map>
#include <string>

#include "REigen.hpp"
#include "RMacroDefine.hpp"

namespace robsoft{

enum JOINTINDEX{JOINT_NONE=-2, JOINT_WHOLE=-1, JOINT_1, JOINT_2, JOINT_3, JOINT_4, JOINT_5, JOINT_6, JOINT_7, JOINT_8, JOINT_9, JOINT_10, JOINT_11, JOINT_12, JOINT_13, JOINT_14, JOINT_15};   // 关节序号
enum TERMINALINDEX{TERMINAL_X, TERMINAL_Y, TERMINAL_Z, TERMINAL_A, TERMINAL_B, TERMINAL_C}; // 末端位姿序号
enum POINTINDEX{POINT_X, POINT_Y, POINT_Z}; // 点的序号
enum ATTITUDEINDEX{ATTITUDE_A, ATTITUDE_B, ATTITUDE_C}; // 姿态角序号
enum QUATERNIONINDEX{QUATERNION_W, QUATERNION_X, QUATERNION_Y, QUATERNION_Z};   // 四元素序号
enum COORDINATESYSTEM{COORDINATE_BASE, COORDINATE_WORK, COORDINATE_TOOL};    // 坐标系
enum CIRCLETYPE{PART_CIRCLE, WHOLE_CIRCLE};    // 部分圆，整圆
enum CURVETYPE{OPEN_CURVE, CLOSE_CURVE};    // 部分圆，整圆

class DECLSPEC_DLLEXPORT Joints{   // 关节
public:
    Joints();                                           // 初始化为6自由度关节，默认角度为0
    Joints(int dof);                                    // 初始化为指定自由度的关节，默认角度为0
    Joints(int dof, double value);                      // 指定自由度，通过常数初始化角度值
    Joints(int dof, double* value);                     // 指定自由度，通过数组初始化角度值
    Joints(int dof, const std::vector<double>& value);  // 指定自由度，通过向量初始化角度值
    Joints(const std::vector<double>& value);           // 通过向量初始化角度值，向量长度即为自由度
    Joints(const VectorXd& value);                      // 通过VectorXd初始化角度值
    ~Joints();

    void setValue(int dof);                                     // 初始化为指定自由度的关节，默认角度为0
    void setValue(int dof, double value);                       // 指定自由度，通过常数初始化角度值
    void setValue(int dof, double* value);                      // 指定自由度，通过数组初始化角度值
    void setValue(int dof, const std::vector<double>& value);   // 指定自由度，通过向量初始化角度值
    void setValue(const std::vector<double>& value);            // 通过向量初始化角度值，向量长度即为自由度
    void setValue(const VectorXd& value);                       // 通过VectorXd初始化角度值

    void append(const Joints& joint);   // 扩展角度值
    Joints interceptDOF(int dof) const; // 返回保留前dof个数据
    void alignDOF(int dof);             // 对齐自由度

    int getJointsDOF() const;                           // 返回自由度
    double getValue(JOINTINDEX index) const;            // 返回指定关节角度值
    VectorXd getValue() const;                          // 返回所有关节角度
    void getValue(double* value) const;                 // 通过数组获取所有关节角度
    void getValue(std::vector<double>& value) const;    // 通过向量获取所有关节角度
    void getValue(VectorXd& value) const;               // 通过VectorXd获取所有关节角度
    double norm() const;                                // 返回关节空间二范数
    double norm(const Joints& joint) const;             // 返回两组关节的空间距离

    double& operator [](JOINTINDEX index);              // 返回指定关节角度值
    const double& operator [](JOINTINDEX index) const;  // 返回指定关节角度值
    Joints operator +(const Joints& joint) const;       // 关节相加
    Joints operator -(const Joints& joint) const;       // 关节相减
    Joints operator *(const Joints& joint) const;       // 关节一对一相乘
    Joints operator /(const Joints& joint) const;       // 关节一对一相除
    Joints operator *(double value) const;              // 关节所有值乘常数
    Joints operator /(double value) const;              // 关节所有值除以常数

    bool operator ==(const Joints& joint) const;    // 判断两组关节是否相等
    bool operator !=(const Joints& joint) const;    // 判断两组关节是否不同

    bool judgeOverMinimum(const Joints& joint) const;       // 判断是否存在角度小于参考值
    bool judgeOverMaximum(const Joints& joint) const;       // 判断是否存在角度大于参考值
    bool judgeABSOverMaximum(const Joints& joint) const;    // 判断是否存在角度绝对值大于参考值

    void print(const char* str = "Joints") const;   // 打印所有关节值

protected:
    VectorXd m_value;
};

class DECLSPEC_DLLEXPORT JointsList{   // 关节列表
public:
    JointsList();
    JointsList(const std::vector<Joints> &js);
    ~JointsList();

    void clear();
    void push_back(const Joints& element);
    void push_front(const Joints& element);
    void pop_back();
    void erase(int index);
    Joints& operator [](int index);
    const Joints& operator [](int index) const;

    VectorXd getCertainJointList(JOINTINDEX index) const;    // 获取关节空间点列中的某一关节的点列

    bool empty() const;
    int size() const;

private:
    std::vector<Joints> m_jVec;
};

class DECLSPEC_DLLEXPORT JointsMotionState{
public:
    JointsMotionState();
    JointsMotionState(int dof);
    ~JointsMotionState();

    void setValue(int dof);
    void setPosValue(const Joints& pos);
    void setVelValue(const Joints& vel);
    void setAccValue(const Joints& acc);
    void setTorValue(const Joints& tor);

    int getJointsDOF() const;
    Joints getPosValue() const;
    Joints getVelValue() const;
    Joints getAccValue() const;
    Joints getTorValue() const;

    void append(const JointsMotionState& jMotion);
    void calVelAcc(const JointsMotionState& jMotion, double time);

private:
    Joints m_pos;
    Joints m_vel;
    Joints m_acc;
    Joints m_tor;
};

class DECLSPEC_DLLEXPORT JointsMotionStateList{   // 关节状态列表
public:
    JointsMotionStateList();
    ~JointsMotionStateList();

    void clear();
    void push_back(const JointsMotionState& element);
    void pop_back();
    void pop_front();
    void erase(int index);
    void append(const JointsMotionStateList& jMotionList);
    JointsMotionState front() const;
    JointsMotionState back() const;
    JointsMotionState& operator [](int index);
    const JointsMotionState& operator [](int index) const;

    bool empty() const;
    int size() const;

    void print() const;

private:
    std::vector<JointsMotionState> m_jmsVec;
};

class DECLSPEC_DLLEXPORT Point{    // 点的位置
public:
    Point();                                            // 生成默认点位，(0,0,0)
    Point(double valueX, double valueY, double valueZ); // 生成点位
    Point(const Vector3d& value);                       // 通过Vector3d生成点位
    ~Point();

    void setValue(double valueX, double valueY, double valueZ); // 生成点位
    void setValue(const Vector3d& value);                       // 通过Vector3d生成点位

    double getValue(POINTINDEX index) const;                                // 返回指定的点位值
    Vector3d getValue() const;                                              // 返回Vector3d类型的点位
    void getValue(double& valueX, double& valueY, double& valueZ) const;    // 获取点位x，y，z的值
    void getValue(Vector3d& value) const;                                   // 获取点位，存储为Vector3d

    double& operator [](POINTINDEX index);              // 返回指定的点位值
    const double& operator [](POINTINDEX index) const;  // 返回指定的点位值
    Point operator +(const Point& point) const;         // 点位相加
    Point operator -(const Point& point) const;         // 点位相减
    Point operator *(double value) const;               // 点位相乘

    double dot(const Point& point) const;   // 点积
    Point cross(const Point& point) const;  // 叉乘
    double norm() const;                    // 欧氏距离
    void normalization();                   // 归一化

    bool operator ==(const Point& point) const; // 判断点位是否相同
    bool operator !=(const Point& point) const; // 判断点位是否不同

    void print(const char* str = "Point") const;    // 打印点位

protected:
    Vector3d m_value;
};

class Point;
class DECLSPEC_DLLEXPORT UnitVector:public Point{  // 单位向量
public:
    UnitVector();
    UnitVector(double valueX, double valueY, double valueZ);
    UnitVector(const Vector3d& value);
    UnitVector(const Point& point);
    ~UnitVector();

    void setValue(double valueX, double valueY, double valueZ);
    void setValue(const Vector3d& value);
    void setValue(const Point& point);

    void operator=(const Point& point);

    void print(const char* str = "UnitVector") const;
};

class RotateMatrix;
class DualVector;
class Quaternion;
class RotateAxis;
class DECLSPEC_DLLEXPORT AttitudeAngle{    // X-Y-Z固定角，先绕X轴gamma，再绕Y轴beta，最后绕Z轴alpha，等价于Z-Y-X欧拉角，先绕Z轴alpha，再绕Y轴beta，再绕X轴gamma
public:
    AttitudeAngle();                                            // 生成默认姿态，(0,0,0)
    AttitudeAngle(double valueA, double valueB, double valueC); // 生成姿态
    AttitudeAngle(const Vector3d& value);                       // 通过Vector3d生成姿态
    ~AttitudeAngle();

    void setValue(double valueA, double valueB, double valueC); // 生成姿态
    void setValue(const Vector3d& value);                       // 通过Vector3d生成姿态

    double getValue(ATTITUDEINDEX index) const;                             // 返回指定的姿态值
    Vector3d getValue() const;                                              // 返回Vector3d类型的姿态
    void getValue(double& valueA, double& valueB, double& valueC) const;    // 获取姿态A，B，C的值

    RotateMatrix getRotateMatrix() const;   // 生成旋转矩阵
    DualVector getDualVector() const;       // 生成双向量
    Quaternion getQuaternion() const;       // 生成四元数

    double& operator [](ATTITUDEINDEX index);                   // 返回指定的姿态值
    const double& operator [](ATTITUDEINDEX index) const;       // 返回指定的姿态值
    AttitudeAngle operator +(const AttitudeAngle& angle) const; // 姿态值相加
    AttitudeAngle operator -(const AttitudeAngle& angle) const; // 姿态值相减
    AttitudeAngle operator *(double value) const;               // 姿态值相乘

    void print(const char* str = "AttitudeAngle") const;        // 打印姿态值

protected:
    Vector3d m_value;
};

class DECLSPEC_DLLEXPORT Mat3x3{
public:
    Mat3x3();
    Mat3x3(const Matrix3d& value);
    ~Mat3x3();

    void setValue(const Matrix3d& value);
    void setValue(int i, int j, double value);

    Matrix3d getValue() const;
    double getValue(int i, int j);

    Point operator *(const Point& point) const;
    Mat3x3 trans() const;
    Mat3x3 inv() const;

    bool operator ==(const Mat3x3& mat) const;
    bool operator !=(const Mat3x3& mat) const;

    void print(const char* str = "Mat3x3") const;

protected:
    Matrix3d m_value;
};

class AttitudeAngle;
class DualVector;
class Quaternion;
class RotateAxis;
class DECLSPEC_DLLEXPORT RotateMatrix{ // 旋转矩阵
public:
    RotateMatrix();                         // 生成默认旋转矩阵，为单位阵
    RotateMatrix(const Matrix3d& value);    // 通过Matrix3d设定旋转矩阵
    ~RotateMatrix();

    void setValue(const Matrix3d& value);   // 通过Matrix3d设定旋转矩阵

    double getValue(int i, int j) const;    // 返回指定旋转矩阵值
    Matrix3d getValue() const;              // 返回Matrix3d类型的姿态

    void setFromZYZ(double valueA, double valueB, double valueC);       // 通过ZYZ姿态角生成旋转矩阵
    void transToZYZ(double& valueA, double& valueB, double& valueC);    // 转换为ZYZ姿态角

    AttitudeAngle getAttitudeAngle() const; // 生成欧拉角
    DualVector getDualVector() const;       // 生成双向量
    Quaternion getQuaternion() const;       // 生成四元数
    RotateAxis getRotateAxis() const;       // 生成轴角度

    RotateMatrix operator *(double value) const;                // 旋转矩阵乘常数
    RotateMatrix operator +(const RotateMatrix& rotate) const;  // 旋转矩阵相加
    RotateMatrix operator -(const RotateMatrix& rotate) const;  // 旋转矩阵相减
    RotateMatrix operator *(const RotateMatrix& rotate) const;  // 旋转矩阵相乘
    Point operator *(const Point& point) const;                 // 旋转矩阵乘点位向量
    RotateMatrix trans() const;                                 // 旋转矩阵转置
    RotateMatrix inv() const;                                   // 旋转矩阵求逆

    void print(const char* str = "RotateMatrix") const; // 打印旋转矩阵

protected:
    Matrix3d m_value;
};

class AttitudeAngle;
class RotateMatrix;
class Quaternion;
class RotateAxis;
class DECLSPEC_DLLEXPORT DualVector{   // 双向量
public:
    DualVector();
    DualVector(const UnitVector& dirVector, const UnitVector& normVector);
    ~DualVector();

    void setValue(const UnitVector& dirVector, const UnitVector& normVector);
    void setDirVector(const UnitVector &dirVector);
    void setNormVector(const UnitVector& normVector);

    UnitVector getDirVector() const;
    UnitVector getNormVector() const;

    RotateMatrix getRotateMatrix() const;
    AttitudeAngle getAttitudeAngle() const;
    Quaternion getQuaternion() const;

    void print(const char* str = "DualVector") const;

protected:
    UnitVector m_dirVector;
    UnitVector m_normVector;
};

class AttitudeAngle;
class RotateMatrix;
class DualVector;
class RotateAxis;
class DECLSPEC_DLLEXPORT Quaternion{   // 四元数
public:
    Quaternion();
    Quaternion(const Vector4d& value);
    Quaternion(double valueW, double valueX, double valueY, double valueZ);
    ~Quaternion();

    void setValue(const Vector4d& value);
    void setValue(double valueW, double valueX, double valueY, double valueZ);

    double getValue(QUATERNIONINDEX index) const;

    RotateMatrix getRotateMatrix() const;
    AttitudeAngle getAttitudeAngle() const;
    DualVector getDualVector() const;

    double& operator [](QUATERNIONINDEX index);
    const double& operator [](QUATERNIONINDEX index) const;
    Quaternion operator *(double value) const;
    double operator *(const Quaternion &q) const;
    Quaternion operator +(const Quaternion &q) const;
    double dot(const Quaternion &q) const;
    Quaternion cross(const Quaternion &q) const;
    double norm() const;
    Quaternion conj() const;    // 共轭
    Quaternion reverse() const; // 取反
    Quaternion log() const; // log(q)
    Quaternion exp() const; // exp(q)

    void print(const char* str = "Quaternion") const;

protected:
    Vector4d m_value;   // 0:w 1:x 2:y 3:z
};

class AttitudeAngle;
class RotateMatrix;
class DualVector;
class Quaternion;
class DECLSPEC_DLLEXPORT RotateAxis{   // 绕固定轴旋转的姿态表示
public:
    RotateAxis();
    RotateAxis(const UnitVector& axis, double angle);
    ~RotateAxis();

    void setValue(const UnitVector& axis, double angle);

    UnitVector getRotateAxis() const;
    double getRotateAngle() const;
    RotateMatrix getRotateMatrix() const;

    void print(const char* str = "RotateAxis") const;

protected:
    UnitVector m_axis;
    double m_angle;
};

class Point;
class AttitudeAngle;
class RotateMatrix;
class HomogeneousMatrix;
class DECLSPEC_DLLEXPORT Terminal{ // 末端位姿
public:
    Terminal();                                                     // 设置默认位姿，(0,0,0,0,0,0)
    Terminal(const Vector6d& value);                                // 通过Vector6d设置位置姿态值
    Terminal(const std::vector<double>& value);                     // 通过向量设置位置姿态值
    Terminal(double valueX, double valueY, double valueZ, double valueA, double valueB, double valueC); // 设置位置姿态值
    Terminal(const Point& point, const AttitudeAngle& attitude);    // 通过点和欧拉角设置位置姿态值
    Terminal(const Point& point, const RotateMatrix& rotate);       // 通过点和旋转矩阵设置位置姿态值
    Terminal(const HomogeneousMatrix& homo);                        // 通过齐次矩阵设置位置姿态值
    Terminal(const Point& po, const Point& px, const Point& py);    // 通过坐标系设置位置姿态值
    ~Terminal();

    void setValue(const Vector6d& value);                               // 通过Vector6d设置位置姿态值
    void setValue(const std::vector<double>& value);                    // 通过向量设置位置姿态值
    void setValue(double valueX, double valueY, double valueZ, double valueA, double valueB, double valueC); // 设置位置姿态值
    void setValue(const Point& point, const AttitudeAngle& attitude);   // 通过点和欧拉角设置位置姿态值
    void setValue(const Point& point, const RotateMatrix& rotate);      // 通过点和旋转矩阵设置位置姿态值
    void setValue(const HomogeneousMatrix& homo);                       // 通过齐次矩阵设置位置姿态值
    void setValue(const Point& po, const Point& px, const Point& py);   // 通过坐标系设置位置姿态值

    Point getPoint() const;                         // 返回点位
    AttitudeAngle getAttitudeAngle() const;         // 返回欧拉角
    RotateMatrix getRotateMatrix() const;           // 返回旋转矩阵
    HomogeneousMatrix getHomogeneousMatrix() const; // 生成其次矩阵

    double getValue(TERMINALINDEX index) const; // 返回指定的位置或姿态值
    Vector6d getValue() const;                  // 返回Vector6类型的位置姿态值
    double norm() const;                            // 返回位姿空间二范数
    double norm(const Terminal& terminal) const;    // 返回两组位姿的空间距离

    Terminal getTerminalInWorkFrame(Terminal workframe) const;      // 已知基坐标系下值，求工件坐标系下的值
    Terminal getTerminalFromWorkFrame(Terminal workframe) const;    // 已知工件坐标系下值，求基坐标系下值
    Terminal getContraryTerminal() const;   // 当Terminal表示工件表面向上的坐标系时，获取工具端抓取的Terminal

    double& operator [](TERMINALINDEX index);               // 返回指定的位置或姿态值
    const double& operator [](TERMINALINDEX index) const;   // 返回指定的位置或姿态值
    Terminal operator +(const Terminal& terminal) const;    // 位姿相加
    Terminal operator -(const Terminal& terminal) const;    // 位姿相减
    Terminal operator *(const Terminal& terminal) const;    // 位姿相乘
    Terminal operator *(double value) const;                // 位姿乘常数

    bool operator ==(const Terminal& terminal) const;   // 判断位姿是否相同
    bool operator !=(const Terminal& terminal) const;   // 判断位姿是否不同

    bool judgeOverMinimum(const Terminal& terminal) const;      // 判断是否存在位姿小于参考值
    bool judgeOverMaximum(const Terminal& terminal) const;      // 判断是否存在位姿大于参考值
    bool judgeABSOverMaximum(const Terminal& terminal) const;   // 判断是否存在位姿绝对值大于参考值

    void print(const char* str = "Terminal") const;     // 打印位置姿态

protected:
    Point m_point;
    AttitudeAngle m_attitude;
};

class DECLSPEC_DLLEXPORT TerminalList{   // 关节状态列表
public:
    TerminalList();
    TerminalList(const std::vector<Terminal> &ts);
    ~TerminalList();

    void clear();
    void push_back(const Terminal& element);
    void push_front(const Terminal& element);
    void pop_back();
    void erase(int index);
    Terminal& operator [](int index);
    const Terminal& operator [](int index) const;

    bool empty() const;
    int size() const;

private:
    std::vector<Terminal> m_tVec;
};

class Point;
class AttitudeAngle;
class RotateMatrix;
class Terminal;
class DECLSPEC_DLLEXPORT HomogeneousMatrix{    // 齐次矩阵
public:
    HomogeneousMatrix();                                                    // 设置默认齐次矩阵，为单位阵
    HomogeneousMatrix(const Matrix4d& value);                               // 通过Matrix4d设置齐次矩阵
    HomogeneousMatrix(const std::vector<double>& value);                    // 通过向量设置齐次矩阵
    HomogeneousMatrix(const Point& point, const AttitudeAngle& attitude);   // 通过点和欧拉角设置齐次矩阵
    HomogeneousMatrix(const Point& point, const RotateMatrix& rotate);      // 通过点和旋转矩阵设置齐次矩阵
    ~HomogeneousMatrix();

    void setValue(const Matrix4d& value);                               // 通过Matrix4d设置齐次矩阵
    void setValue(const std::vector<double>& value);                    // 通过向量设置齐次矩阵
    void setValue(const Point& point, const AttitudeAngle& attitude);   // 通过点和欧拉角设置齐次矩阵
    void setValue(const Point& point, const RotateMatrix& rotate);      // 通过点和旋转矩阵设置齐次矩阵

    double getValue(int i, int j) const;    // 返回指定齐次矩阵值
    Matrix4d getValue() const;              // 返回Matrix4d类型的齐次矩阵

    Point getPoint() const;                 // 返回点位
    AttitudeAngle getAttitudeAngle() const; // 返回欧拉角
    RotateMatrix getRotateMatrix() const;   // 返回旋转矩阵
    Terminal getTerminal() const;           // 返回位置姿态

    HomogeneousMatrix operator *(const HomogeneousMatrix& homo) const;  // 齐次矩阵相乘
    HomogeneousMatrix inv() const;                                      // 齐次矩阵求逆

    HomogeneousMatrix getHomogeneousMatrixInWorkFrame(HomogeneousMatrix workframe) const;   // 已知基坐标系下值，求工件坐标系下的值
    HomogeneousMatrix getHomogeneousMatrixFromWorkFrame(HomogeneousMatrix workframe) const; // 已知工件坐标系下值，求基坐标系下值

    void print(const char* str = "HomogeneousMatrix") const;    // 打印齐次矩阵

protected:
    Matrix4d m_value;
};

class DECLSPEC_DLLEXPORT RobotMotion{
public:
    RobotMotion();
    ~RobotMotion();

    void setCurrentJointPosition(const Joints& joint);  // 设置当前关节角度
    Joints getCurrentJointPosition() const;             // 获取当前关节角度

    void setCurrentJointVelocity(const Joints& joint);  // 设置当前关节速度
    Joints getCurrentJointVelocity() const;             // 获取当前关节速度

    void setCurrentJointAcceleration(const Joints& joint);  // 设置当前关节加速度
    Joints getCurrentJointAcceleration() const;             // 获取当前关节加速度

    void setCurrentJointTorque(const Joints& joint);    // 设置当前关节力矩
    Joints getCurrentJointTorque() const;               // 获取当前关节力矩

    void setCurrentTerminal(const Terminal& terminal);  // 设置当前末端位姿
    Terminal getCurrentTerminal() const;                // 获取当前末端位姿

    void setCurrentWorkTerminal(const Terminal& terminal);  // 设置当前工件坐标系下末端位姿
    Terminal getCurrentWorkTerminal() const;                // 获取当前工件坐标系下末端位姿

private:
    Joints m_currentJointPosition;      //当前关节角度
    Joints m_currentJointVelocity;      //当前关节速度
    Joints m_currentJointAcceleration;  //当前关节加速度
    Joints m_currentJointTorque;        //当前关节力矩
    Terminal m_currentTerminal;         //当前末端位姿
    Terminal m_currentWorkTerminal;     //当前工件坐标系下姿态
};

class DECLSPEC_DLLEXPORT Link{
public:
    Link();
    ~Link();

    void setQuality(double quality);
    void setCentroid(const Point& centroid);
    void setInertialTensor(const Mat3x3& inertialTensor);

    double getQuality() const;
    Point getCentroid() const;
    Mat3x3 getInertialTensor() const;

    bool operator ==(const Link& link) const;
    bool operator !=(const Link& link) const;

    void print(const char* str = "Link") const;

private:
    double m_quality;           // 质量
    Point m_centroid;           // 质心
    Mat3x3 m_inertialTensor;    // 惯性张量
};

class DECLSPEC_DLLEXPORT Tool{
public:
    Tool();
    ~Tool();

    void setFrame(const Terminal& frame);
    void setQuality(double quality);
    void setCentroid(const Point& centroid);
    void setInertialTensor(const Mat3x3& inertialTensor);

    Terminal getFrame() const;
    double getQuality() const;
    Point getCentroid() const;
    Mat3x3 getInertialTensor() const;

    bool operator ==(const Tool& tool) const;
    bool operator !=(const Tool& tool) const;

    void print(const char* str = "Tool") const;

private:
    Terminal m_frame;   // 坐标系
    double m_quality;   // 质量
    Point m_centroid;   // 质心
    Mat3x3 m_inertialTensor;    // 惯性张量
};

class DECLSPEC_DLLEXPORT ForceSensor{
public:
    ForceSensor();
    ~ForceSensor();

    void setZeroPoint(const Terminal& zeroPoint);
    Terminal getZeroPoint() const;

    void setInertialCoefficient(const Terminal& inertialCoefficient);
    Terminal getInertialCoefficient() const;

    void setDampingCoefficient(const Terminal& dampingCoefficient);
    Terminal getDampingCoefficient() const;

    void setStiffnessCoefficient(const Terminal& stiffnessCoefficient);
    Terminal getStiffnessCoefficient() const;

private:
    Terminal m_zeroPoint;               // 末端六维力传感器零点
    Terminal m_inertialCoefficient;     // 末端阻抗惯性系数
    Terminal m_dampingCoefficient;      // 末端阻抗阻尼系数
    Terminal m_stiffnessCoefficient;    // 末端阻抗刚度系数
};

}

#endif
