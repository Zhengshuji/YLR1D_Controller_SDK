// robot_sdk.hpp — RobotConSys SDK 薄封装（dlopen 工厂模式同 demo，库路径由可执行文件位置推算，
// 不依赖 cwd）。所有 SDK 调用收口在此类，驱动节点只依赖本类。
#ifndef ROBOT_DRIVER__ROBOT_SDK_HPP_
#define ROBOT_DRIVER__ROBOT_SDK_HPP_

#include <string>
#include <vector>

#include "RobotConSys/RobotConSys.h"
#include "SystemOperation.h"

namespace robot_driver {

// SDK 单位约定（实测）：臂关节角 = 度；末端位置 = mm、姿态角 = 度；
// 底盘 vx/vy = mm/s、wz = rad/s（与 Demo 一致）。框架/moveit 用弧度+米，
// 在本类（SDK 边界）统一换算。
constexpr double kDeg2Rad = 0.017453292519943295;  // pi/180
constexpr double kRad2Deg = 57.29577951308232;     // 180/pi
constexpr double kMm2M = 0.001;
constexpr double kM2Mm = 1000.0;

class RobotSdk
{
public:
  RobotSdk() = default;
  ~RobotSdk() { close(); }

  /// lib_dir 含 libRobotConSys_Client.so（如 install/robot_driver/lib）
  bool connect(const std::string & ip, int port, const std::string & lib_dir)
  {
    if (sys_) { return true; }
    if (lib_.open(lib_dir.c_str(), "RobotConSys_Client") != 0) { return false; }
    auto create = reinterpret_cast<rclib::RobotConSys * (*)()>(
        lib_.loadFunc("createRobotConSys_Client"));
    free_ = reinterpret_cast<void (*)(rclib::RobotConSys *)>(
        lib_.loadFunc("freeRobotConSys_Client"));
    if (!create || !free_) { lib_.close(); return false; }
    sys_ = create();
    if (!sys_) { lib_.close(); return false; }
    if (sys_->init(ip.c_str(), port) != 0) { close(); return false; }
    return true;
  }

  void close()
  {
    if (sys_ && free_) { free_(sys_); }
    sys_ = nullptr;
    free_ = nullptr;
    lib_.close();
  }

  bool online() const { return sys_ && sys_->isEstablished(); }
  void setAuthority(DevLayer::AUTHORITY_TYPE a) { if (sys_) { sys_->setAuthority(a); } }
  // 系统运行/示教状态（外部客户端控制需 REMOTE+RUN；实验性，Demo 未显式设置）
  void setPlayState(int s) { if (sys_) { sys_->setPlayState(rclib::SYSPLAYSTATE(s)); } }
  void setRunState(int s) { if (sys_) { sys_->setRunState(rclib::SYSRUNSTATE(s)); } }
  int getErrorState(int arm) const
  {
    if (!sys_) { return 0; }
    sys_->updateRobotState(rclib::ROBOTCONSYS_ARM_INDEX(arm));
    return static_cast<int>(sys_->getErrorState(rclib::ROBOTCONSYS_ARM_INDEX(arm)));
  }

  // ── 臂 ──
  int setServoState(int arm, bool on)
  {
    return sys_ ? sys_->setServoState(rclib::ROBOTCONSYS_ARM_INDEX(arm),
                                      on ? rclib::SWITCHON : rclib::SWITCHOFF)
                : -1;
  }
  int getDOF(int arm) const
  {
    return sys_ ? sys_->getRobotParameter(rclib::ROBOTCONSYS_ARM_INDEX(arm)).getWholeDOF()
                : 0;
  }
  int moveABSJoint(int arm, const std::vector<double> & joints_rad, double vel)
  {
    if (!sys_) { return -1; }
    std::vector<double> deg(joints_rad.size());
    for (size_t i = 0; i < joints_rad.size(); ++i) { deg[i] = joints_rad[i] * kRad2Deg; }
    robsoft::Joints j(deg);
    return sys_->moveABSJoint(rclib::ROBOTCONSYS_ARM_INDEX(arm), j, vel);
  }
  /// 阻塞等待该臂运动执行完成（Demo 序列的关键：moveABSJoint 后必须 waitMotionCMDFinish
  /// 才会真实执行；在独立线程调用避免阻塞 executor）
  int waitMotionCMDFinish(int arm) const
  {
    return sys_ ? sys_->waitMotionCMDFinish(rclib::ROBOTCONSYS_ARM_INDEX(arm)) : -1;
  }
  /// 轮询当前关节位置（内部先 updateRobotMotion 刷新缓存），printState 模式
  std::vector<double> getJointPos(int arm) const
  {
    std::vector<double> out;
    if (!sys_) { return out; }
    sys_->updateRobotMotion(rclib::ROBOTCONSYS_ARM_INDEX(arm));
    robsoft::Joints j = sys_->getRobotMotion(rclib::ROBOTCONSYS_ARM_INDEX(arm))
                            .getCurrentJointPosition();
    j.getValue(out);
    for (double & v : out) { v *= kDeg2Rad; }   // SDK 报度 -> 框架弧度
    return out;
  }
  std::vector<double> getTerminal(int arm) const
  {
    std::vector<double> out;
    if (!sys_) { return out; }
    sys_->updateRobotMotion(rclib::ROBOTCONSYS_ARM_INDEX(arm));
    robsoft::Terminal t = sys_->getRobotMotion(rclib::ROBOTCONSYS_ARM_INDEX(arm))
                              .getCurrentTerminal();
    for (int i = 0; i < 6; ++i) {   // xyz=mm->m，ABC=deg->rad
      const double v = t[robsoft::TERMINALINDEX(i)];
      out.push_back(i < 3 ? v * kMm2M : v * kDeg2Rad);
    }
    return out;
  }
  bool getServoState(int arm) const
  {
    if (!sys_) { return false; }
    sys_->updateRobotState(rclib::ROBOTCONSYS_ARM_INDEX(arm));
    return sys_->getServoState(rclib::ROBOTCONSYS_ARM_INDEX(arm)) == rclib::SWITCHON;
  }
  void setClawState(int arm, bool on)
  {
    if (sys_) { sys_->setClawState(rclib::ROBOTCONSYS_ARM_INDEX(arm),
                                   on ? rclib::SWITCHON : rclib::SWITCHOFF); }
  }
  bool getClawState(int arm) const
  {
    if (!sys_) { return false; }
    sys_->updateClawState(rclib::ROBOTCONSYS_ARM_INDEX(arm));
    return sys_->getClawState(rclib::ROBOTCONSYS_ARM_INDEX(arm)) == rclib::SWITCHON;
  }

  // ── 底盘 ──
  void setVehicleControlType(int type)
  {
    if (sys_) { sys_->setVehicleControlType(DevLayer::VEHICLE_CONTROL_TYPE(type)); }
  }
  void setVehicleServoState(bool on)
  {
    if (sys_) { sys_->setVehicleServoState(on ? rclib::SWITCHON : rclib::SWITCHOFF); }
  }
  /// mode 见 DevLayer::VEHICLE_MODE；x,y 单位 mm/s；z 单位 rad/s
  void setMotionControl(int mode, float x, float y, float z)
  {
    if (sys_) { sys_->setMotionControl(DevLayer::VEHICLE_MODE(mode), x, y, z); }
  }
  DevLayer::Vehicle_State_Data getVehicleState() const
  {
    DevLayer::Vehicle_State_Data out{};
    if (!sys_) { return out; }
    sys_->updateVehicleState();
    return sys_->getVehicleState();
  }

private:
  SysLayer::CLoadLibrary lib_;
  rclib::RobotConSys * sys_ = nullptr;
  void (*free_)(rclib::RobotConSys *) = nullptr;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__ROBOT_SDK_HPP_