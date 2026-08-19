// 驱动层 robot_driver：平替 ylr1d_translate 及以下（translate/control/algorithm/plant）。
// 对外 3 个 action（复用 ylr1d_translate 类型，规划/HMI 零改动）：
//   /chassis_move  (mode+direction+speed+duration) -> setMotionControl(vx,vy,wz)
//   /arm_move      (part+positions)                -> moveABSJoint + 轮询到位
//   /gripper_move  (part+open)                     -> setClawState
// 同时轮询 SDK 状态发布 /sensors/arm_raw + /sensors/vehicle_raw（供传感器层），/health 自维护。
// 动作状态机用 20Hz 定时器推进（同转译层 translate_node 模式），不阻塞 executor。
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_interfaces/msg/tr_arm_msg.hpp"
#include "robot_interfaces/msg/tr_vehicle_msg.hpp"
#include "ylr1d_translate/action/chassis_move.hpp"
#include "ylr1d_translate/action/arm_move.hpp"
#include "ylr1d_translate/action/gripper_move.hpp"
#include "robot_driver/robot_sdk.hpp"

#include <unistd.h>  // readlink
#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace robot_driver {

using ChassisMove = ylr1d_translate::action::ChassisMove;
using ArmMove = ylr1d_translate::action::ArmMove;
using GripperMove = ylr1d_translate::action::GripperMove;
using ChassisGoalHandle = rclcpp_action::ServerGoalHandle<ChassisMove>;
using ArmGoalHandle = rclcpp_action::ServerGoalHandle<ArmMove>;
using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperMove>;

// 与 ylr1d_translate constants.hpp 对齐的常量
namespace cst {
constexpr int8_t kModeTranslate = 0, kModeRotate = 1, kModeStop = 2;
constexpr double kArmTol = 0.02;      // 臂到位容差 rad
constexpr double kArmTimeout = 30.0;  // 臂超时 s
constexpr double kMmPerM = 1000.0;    // m/s -> mm/s（SDK 底盘速度单位）
}  // namespace cst

class RobotDriverNode : public rclcpp::Node
{
public:
  RobotDriverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
      : Node("robot_driver", opts)
  {
    declare_parameter("server_ip", "172.22.224.1");
    declare_parameter("server_port", 8109);
    declare_parameter("default_arm_vel", 0.2);  // rad/s，ArmMove 无速度字段
    declare_parameter("servo_on_start", true);
    declare_parameter("home_on_start", false);   // 默认不回零（与 armDemo 对齐，排查臂运动）
    declare_parameter("home_joints", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter("vehicle_control_type", static_cast<int>(DevLayer::VEHICLE_CONTROL_UART));
    declare_parameter("poll_rate", 20.0);

    ip_ = get_parameter("server_ip").as_string();
    port_ = get_parameter("server_port").as_int();
    arm_vel_ = get_parameter("default_arm_vel").as_double();
    servo_on_start_ = get_parameter("servo_on_start").as_bool();
    home_on_start_ = get_parameter("home_on_start").as_bool();
    home_joints_ = get_parameter("home_joints").as_double_array();
    vehicle_ctrl_type_ = get_parameter("vehicle_control_type").as_int();
    const int poll_hz = static_cast<int>(get_parameter("poll_rate").as_double());

    // ── action server（话题名与转译层一致）──
    chassis_server_ = rclcpp_action::create_server<ChassisMove>(
        this, "chassis_move",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ChassisMove::Goal> g) {
          return chassis_goal_cb(*g); },
        [this](std::shared_ptr<ChassisGoalHandle> h) { return chassis_cancel_cb(h); },
        [this](std::shared_ptr<ChassisGoalHandle> h) { chassis_execute_cb(h); });
    arm_server_ = rclcpp_action::create_server<ArmMove>(
        this, "arm_move",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmMove::Goal> g) {
          return arm_goal_cb(*g); },
        [this](std::shared_ptr<ArmGoalHandle> h) { return arm_cancel_cb(h); },
        [this](std::shared_ptr<ArmGoalHandle> h) { arm_execute_cb(h); });
    gripper_server_ = rclcpp_action::create_server<GripperMove>(
        this, "gripper_move",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperMove::Goal> g) {
          return gripper_goal_cb(*g); },
        [this](std::shared_ptr<GripperGoalHandle> h) { return gripper_cancel_cb(h); },
        [this](std::shared_ptr<GripperGoalHandle> h) { gripper_execute_cb(h); });

    // ── 话题 ──
    arm_raw_pub_ = create_publisher<robot_interfaces::msg::TRArmMsg>("/sensors/arm_raw", 10);
    vehicle_raw_pub_ =
        create_publisher<robot_interfaces::msg::TRVehicleMsg>("/sensors/vehicle_raw", 10);
    health_pub_ = create_publisher<std_msgs::msg::String>("/health", 10);

    // ── 定时器：动作 tick + 状态轮询 ──
    tick_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / poll_hz),
                                    [this]() { tick(); });
    health_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_health(); });
    connect_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { try_connect(); });
    try_connect();
  }

  ~RobotDriverNode()
  {
    if (connected_) {
      sdk_.setMotionControl(DevLayer::VEHICLE_MODE_NORMAL, 0.0f, 0.0f, 0.0f);
    }
    sdk_.close();
  }

private:
  // ═══════════════════════ 连接生命周期 ═══════════════════════
  void try_connect()
  {
    if (connected_) { return; }
    const std::string lib_dir = sdk_lib_dir();
    if (!sdk_.connect(ip_, port_, lib_dir)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "RobotConSys connect failed (%s:%d), retrying...", ip_.c_str(), port_);
      return;
    }
    connected_ = true;
    // 权限与 armDemo 一致用 OPERATOR（实测 ROOT 下 armDemo 序列臂不动；底盘 demo 才用 ROOT）
    sdk_.setAuthority(DevLayer::AUTHORITY_OPERATOR);
    RCLCPP_INFO(get_logger(), "RobotConSys connected: %s:%d (lib %s)", ip_.c_str(), port_,
                lib_dir.c_str());
    for (int arm = 0; arm < 3; ++arm) {   // 探测 ARM_1..3（ARM_4 探测会卡死服务端，勿加回）
      RCLCPP_INFO(get_logger(), "ARM_INDEX[%d] DOF=%d servo=%d err=%d",
                  arm, sdk_.getDOF(arm),
                  sdk_.getServoState(arm) ? 1 : 0, sdk_.getErrorState(arm));
    }
    if (servo_on_start_) {
      sdk_.setVehicleServoState(true);
      sdk_.setServoState(0, true);
      sdk_.setServoState(1, true);
      sdk_.setServoState(2, true);   // 躯干 ARM_3
      RCLCPP_INFO(get_logger(), "servo enabled (vehicle + arm0 + arm1 + torso)");
    }
    if (home_on_start_) {
      for (int arm = 0; arm < 2; ++arm) {
        const int dof = sdk_.getDOF(arm);
        if (dof <= 0) { continue; }
        std::vector<double> target(home_joints_.begin(),
                                   home_joints_.begin() + std::min<size_t>(home_joints_.size(), static_cast<size_t>(dof)));
        while (target.size() < static_cast<size_t>(dof)) { target.push_back(0.0); }
        RCLCPP_INFO(get_logger(), "homing arm%d (%zu joints) (inline, main thread)", arm, target.size());
        if (sdk_.moveABSJoint(arm, target, arm_vel_) == 0) {
          sdk_.waitMotionCMDFinish(arm);
        }
        home_[arm].active = true;
        home_[arm].target = target;
        RCLCPP_INFO(get_logger(), "arm%d homed", arm);
      }
    }
  }

  /// armDemo 对齐：moveABSJoint + waitMotionCMDFinish 在同一 worker 线程执行（回零/臂运动共用；
  /// 实测拆线程 wait 会卡死/不执行）
  void run_arm_motion(int arm, const std::vector<double> & targets, double vel)
  {
    auto & w = arm_worker_[arm];
    ++w.gen;
    const uint64_t g = w.gen;
    w.running = true;
    w.ret.store(-999);
    w.th = std::thread([this, arm, targets, vel, g]() {
      int r = sdk_.moveABSJoint(arm, targets, vel);
      if (r != 0) {
        RCLCPP_WARN(get_logger(), "arm%d moveABSJoint error ret=%d", arm, r);
      } else {
        r = sdk_.waitMotionCMDFinish(arm);
      }
      auto & ww = arm_worker_[arm];
      if (ww.gen == g) { ww.ret.store(r); ww.running = false; }
    });
    w.th.detach();
  }

  /// 由可执行文件位置推算 SDK 库目录：<prefix>/lib/robot_driver/robot_driver -> <prefix>/lib
  static std::string sdk_lib_dir()
  {
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) { return "install/robot_driver/lib"; }
    buf[n] = '\0';
    std::string exe(buf);
    auto slash = exe.find_last_of('/');
    if (slash == std::string::npos) { return "install/robot_driver/lib"; }
    exe = exe.substr(0, slash);                       // <prefix>/lib/robot_driver
    slash = exe.find_last_of('/');
    if (slash == std::string::npos) { return "install/robot_driver/lib"; }
    return exe.substr(0, slash);                      // <prefix>/lib
  }

  // ═══════════════════════ 定时器主 tick ═══════════════════════
  void tick()
  {
    tick_chassis();
    tick_home();
    tick_arm();
    publish_state();
  }

  // ═══════════════════════ 底盘 ═══════════════════════
  /// 下发底盘速度并记录命令值（里程计按命令死推）；未连接则只记录
  void send_chassis(DevLayer::VEHICLE_MODE mode, float vx, float vy, float wz)
  {
    last_cmd_vx_ = vx; last_cmd_vy_ = vy; last_cmd_wz_ = wz;
    if (connected_) { sdk_.setMotionControl(mode, vx, vy, wz); }
  }

  rclcpp_action::GoalResponse chassis_goal_cb(const ChassisMove::Goal & g)
  {
    if (g.mode != cst::kModeTranslate && g.mode != cst::kModeRotate && g.mode != cst::kModeStop) {
      RCLCPP_WARN(get_logger(), "chassis_move rejected: invalid mode=%d", g.mode);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse chassis_cancel_cb(std::shared_ptr<ChassisGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void chassis_execute_cb(std::shared_ptr<ChassisGoalHandle> h)
  {
    if (!h->is_executing()) { h->execute(); }
    if (chassis_.handle && chassis_.handle != h) {
      set_result(chassis_.handle, false, "superseded by new goal");
    }
    const auto & g = h->get_goal();
    chassis_.handle = h;
    chassis_.mode = g->mode;
    chassis_.direction = g->direction;
    chassis_.speed = g->speed;
    chassis_.duration = g->duration;
    chassis_.start = now();

    if (!vehicle_ctrl_set_) {   // 惰性设置（与 armDemo 对齐：vehicle 相关配置不在启动时设）
      sdk_.setVehicleControlType(vehicle_ctrl_type_);
      vehicle_ctrl_set_ = true;
    }
    if (g->mode == cst::kModeStop) {
      send_chassis(DevLayer::VEHICLE_MODE_NORMAL, 0.0f, 0.0f, 0.0f);
      set_result(h, true, "stopped");
      return;
    }
    // TRANSLATE：direction 为车体运动方向角，speed 为线速度(m/s) -> vx/vy(mm/s)
    // ROTATE：speed 为角速度(rad/s) -> wz
    double vx = 0.0, vy = 0.0, wz = 0.0;
    if (g->mode == cst::kModeTranslate) {
      vx = g->speed * std::cos(g->direction) * cst::kMmPerM;
      vy = g->speed * std::sin(g->direction) * cst::kMmPerM;
    } else {
      wz = g->speed;
    }
    // 平移用 MOVE_XY（全向 vx/vy）：实测 NORMAL 只驱动 vx/wz，纯横移（vy）不动；
    // 旋转 ROTATE；停车 NORMAL(0,0,0)（与 Demo 一致）
    const auto mode = (g->mode == cst::kModeRotate) ? DevLayer::VEHICLE_MODE_ROTATE
                                                    : DevLayer::VEHICLE_MODE_MOVE_XY;
    send_chassis(mode, static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(wz));
    RCLCPP_INFO(get_logger(), "chassis_move: mode=%d vx=%.1f vy=%.1f wz=%.3f dur=%.1f",
                g->mode, vx, vy, wz, g->duration);
  }

  void tick_chassis()
  {
    if (!chassis_.handle) { return; }
    auto h = chassis_.handle;
    if (h->is_canceling()) {
      send_chassis(DevLayer::VEHICLE_MODE_NORMAL, 0.0f, 0.0f, 0.0f);
      set_result(h, false, "canceled");
      return;
    }
    if (chassis_.duration > 0.0 &&
        (now() - chassis_.start).seconds() >= chassis_.duration) {
      send_chassis(DevLayer::VEHICLE_MODE_NORMAL, 0.0f, 0.0f, 0.0f);
      set_result(h, true, "move done");
      return;
    }
    auto fb = std::make_shared<ChassisMove::Feedback>();
    fb->phase = "moving";
    h->publish_feedback(fb);
  }

  // ═══════════════════════ 机械臂（关节空间） ═══════════════════════
  rclcpp_action::GoalResponse arm_goal_cb(const ArmMove::Goal & g)
  {
    // 真机仅双臂（无独立躯干）：part 1=左臂 2=右臂；part 0（躯干）拒绝
    const int home_idx = (g.part == 0) ? 2 : g.part - 1;
    if (home_[home_idx].active) {
      RCLCPP_WARN(get_logger(), "arm_move rejected: arm still homing");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (g.part < 0 || g.part > 2) {
      RCLCPP_WARN(get_logger(), "arm_move rejected: invalid part=%d", g.part);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse arm_cancel_cb(std::shared_ptr<ArmGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void arm_execute_cb(std::shared_ptr<ArmGoalHandle> h)
  {
    if (!h->is_executing()) { h->execute(); }
    if (arm_.handle && arm_.handle != h) { set_result(arm_.handle, false, "superseded by new goal"); }
    const auto & g = h->get_goal();
    arm_.handle = h;
    arm_.part = g->part;
    arm_.targets = g->positions;
    arm_.start = now();
    arm_.arm = (g->part == 0) ? 2 : g->part - 1;   // 0->2(躯干/ARM_3) 1->0(左) 2->1(右)
    arm_.dof = connected_ ? sdk_.getDOF(arm_.arm) : 0;
    if (arm_.dof == 0 || arm_.targets.size() != static_cast<size_t>(arm_.dof)) {
      RCLCPP_WARN(get_logger(), "arm_move positions size mismatch: got %zu, dof=%d",
                  arm_.targets.size(), arm_.dof);
      set_result(h, false, "positions size mismatch");
      return;
    }
    if (connected_) {
      // 目标与当前位置一致（容差内）-> 直接成功不调 SDK：moveit 轨迹起点=当前位形时
      // moveABSJoint 下发零位移会被 SDK 拒绝（-1），幂等目标应视为已到位
      const auto cur = sdk_.getJointPos(arm_.arm);
      bool already_at = true;
      for (size_t i = 0; i < arm_.targets.size() && i < cur.size(); ++i) {
        if (std::abs(cur[i] - arm_.targets[i]) > cst::kArmTol) { already_at = false; break; }
      }
      if (already_at) {
        RCLCPP_INFO(get_logger(), "arm_move: arm%d already at target (no motion)", arm_.arm);
        set_result(h, true, "arm already at target");
        return;
      }
      // 与 armDemo 逐行一致：moveABSJoint + waitMotionCMDFinish 在发起连接的线程（executor/main）
      // 内联执行——实测 worker 线程发指令会阻塞且不执行（SDK 线程亲和）
      RCLCPP_INFO(get_logger(), "arm_move: arm%d -> %zu joints @%.2f rad/s (inline, main thread)",
                  arm_.arm, arm_.targets.size(), arm_vel_);
      const int ret = sdk_.moveABSJoint(arm_.arm, arm_.targets, arm_vel_);
      if (ret != 0) {
        RCLCPP_WARN(get_logger(), "arm_move moveABSJoint error ret=%d", ret);
        set_result(h, false, "moveABSJoint error " + std::to_string(ret));
        return;
      }
      const int wret = sdk_.waitMotionCMDFinish(arm_.arm);
      set_result(h, wret == 0, wret == 0 ? "arm reached target"
                                         : "waitMotionCMDFinish error " + std::to_string(wret));
    } else {
      set_result(h, false, "not connected");
      return;
    }
  }

  void tick_arm()
  {
    if (!arm_.handle) { return; }
    auto h = arm_.handle;
    if (h->is_canceling()) {
      ++arm_worker_[arm_.arm].gen;   // 旧 worker 结果作废
      set_result(h, false, "canceled");
      return;
    }
    auto & w = arm_worker_[arm_.arm];
    if (w.running) {
      // 执行中：进度反馈（当前位 vs 目标，近似）
      const auto cur = connected_ ? sdk_.getJointPos(arm_.arm) : std::vector<double>();
      size_t reached = 0;
      for (size_t i = 0; i < arm_.targets.size() && i < cur.size(); ++i) {
        if (std::abs(cur[i] - arm_.targets[i]) <= cst::kArmTol) { ++reached; }
      }
      auto fb = std::make_shared<ArmMove::Feedback>();
      fb->progress = arm_.targets.empty() ? 1.0
                                          : static_cast<double>(reached) / arm_.targets.size();
      h->publish_feedback(fb);
      if ((now() - arm_.start).seconds() > cst::kArmTimeout) {
        ++w.gen;
        set_result(h, false, "timeout");
      }
      return;
    }
    const int r = w.ret.load();
    if (r != -999) {
      w.ret.store(-999);
      set_result(h, r == 0, r == 0 ? "arm reached target"
                                   : "waitMotionCMDFinish error " + std::to_string(r));
    }
  }

  void tick_home()
  {
    for (int arm = 0; arm < 2; ++arm) {
      if (!home_[arm].active) { continue; }
      auto & w = arm_worker_[arm];
      if (w.running) { continue; }
      const int r = w.ret.load();
      if (r != -999) {
        w.ret.store(-999);
        home_[arm].active = false;
        RCLCPP_INFO(get_logger(), "arm%d homed (waitMotionCMDFinish ret=%d)", arm, r);
      }
    }
  }

  // ═══════════════════════ 夹爪 ═══════════════════════
  rclcpp_action::GoalResponse gripper_goal_cb(const GripperMove::Goal & g)
  {
    if (g.part != 0 && g.part != 1) {
      RCLCPP_WARN(get_logger(), "gripper_move rejected: invalid part=%d", g.part);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse gripper_cancel_cb(std::shared_ptr<GripperGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void gripper_execute_cb(std::shared_ptr<GripperGoalHandle> h)
  {
    if (!h->is_executing()) { h->execute(); }
    if (gripper_.handle && gripper_.handle != h) {
      set_result(gripper_.handle, false, "superseded by new goal");
    }
    gripper_.handle = h;
    const auto & g = h->get_goal();
    const int arm = g->part;  // 0=左夹爪->arm0, 1=右夹爪->arm1
    if (connected_) { sdk_.setClawState(arm, g->open); }
    auto fb = std::make_shared<GripperMove::Feedback>();
    fb->progress = 1.0;
    h->publish_feedback(fb);
    RCLCPP_INFO(get_logger(), "gripper_move: arm%d open=%d", arm, g->open ? 1 : 0);
    set_result(h, true, "gripper set");
  }

  // ═══════════════════════ 状态发布 ═══════════════════════
  void publish_state()
  {
    if (!connected_) { return; }
    // 双臂 + 躯干（arm2 = ARM_3）
    const char * arm_names[3] = {"arm0", "arm1", "arm2"};
    for (int arm = 0; arm < 3; ++arm) {
      const auto joints = sdk_.getJointPos(arm);
      if (joints.empty()) { continue; }
      robot_interfaces::msg::TRArmMsg m;
      m.name = arm_names[arm];
      m.dof = static_cast<int32_t>(joints.size());
      m.servo_state = sdk_.getServoState(arm);
      m.claw_state = sdk_.getClawState(arm);
      for (double v : joints) { m.joints.push_back(static_cast<float>(v)); }
      const auto term = sdk_.getTerminal(arm);
      for (double v : term) { m.terminal.push_back(static_cast<float>(v)); }
      arm_raw_pub_->publish(m);
    }
    // 底盘：SDK getVehicleState 不回报实际速度（恒 0）——发布最后命令速度（死推，与速度控制对齐）
    const auto vs = sdk_.getVehicleState();
    robot_interfaces::msg::TRVehicleMsg vm;
    vm.vehicle_servo_state = static_cast<int32_t>(vs.vehicleServoState);
    vm.vehicle_x_vel = last_cmd_vx_;
    vm.vehicle_y_vel = last_cmd_vy_;
    vm.vehicle_z_vel = last_cmd_wz_;
    for (int i = 0; i < 4; ++i) {
      vm.wheel_vel.push_back(vs.wheelVel[i]);
      vm.servo_pos.push_back(vs.servoPos[i]);
    }
    for (int i = 0; i < 8; ++i) { vm.sonar.push_back(static_cast<int8_t>(vs.sonar[i])); }
    vehicle_raw_pub_->publish(vm);
  }

  void publish_health()
  {
    std_msgs::msg::String msg;
    if (connected_ && sdk_.online()) { msg.data = "OK"; }
    else if (connected_) { msg.data = "LOST_CONNECTION"; }
    else { msg.data = "NOT_CONNECTED"; }
    health_pub_->publish(msg);
  }

  // ═══════════════════════ 结果辅助 ═══════════════════════
  void set_result(std::shared_ptr<ChassisGoalHandle> h, bool ok, const std::string & msg)
  {
    if (!h) { return; }
    auto r = std::make_shared<ChassisMove::Result>();
    r->success = ok;
    r->message = msg;
    ok ? h->succeed(r) : h->abort(r);
    if (chassis_.handle == h) { chassis_.handle.reset(); }
    RCLCPP_INFO(get_logger(), "chassis_move %s: %s", ok ? "succeeded" : "aborted", msg.c_str());
  }
  void set_result(std::shared_ptr<ArmGoalHandle> h, bool ok, const std::string & msg)
  {
    if (!h) { return; }
    auto r = std::make_shared<ArmMove::Result>();
    r->success = ok;
    r->message = msg;
    ok ? h->succeed(r) : h->abort(r);
    if (arm_.handle == h) { arm_.handle.reset(); }
    RCLCPP_INFO(get_logger(), "arm_move %s: %s", ok ? "succeeded" : "aborted", msg.c_str());
  }
  void set_result(std::shared_ptr<GripperGoalHandle> h, bool ok, const std::string & msg)
  {
    if (!h) { return; }
    auto r = std::make_shared<GripperMove::Result>();
    r->success = ok;
    r->message = msg;
    ok ? h->succeed(r) : h->abort(r);
    if (gripper_.handle == h) { gripper_.handle.reset(); }
    RCLCPP_INFO(get_logger(), "gripper_move %s: %s", ok ? "succeeded" : "aborted", msg.c_str());
  }

  // ── 成员 ──
  RobotSdk sdk_;
  bool connected_ = false;
  std::string ip_;
  int port_ = 8109;
  double arm_vel_ = 0.2;
  bool servo_on_start_ = true;
  int vehicle_ctrl_type_ = DevLayer::VEHICLE_CONTROL_UART;
  bool vehicle_ctrl_set_ = false;   // vehicle 控制类型首次底盘指令时设置（armDemo 对齐）
  // SDK getVehicleState 不回报底盘实际速度（恒 0）：里程计按最后命令速度死推（与速度控制对齐）
  float last_cmd_vx_ = 0.0f, last_cmd_vy_ = 0.0f, last_cmd_wz_ = 0.0f;
  bool home_on_start_ = false;
  std::vector<double> home_joints_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  struct { bool active = false; std::vector<double> target; } home_[2];
  // 臂运动 worker：waitMotionCMDFinish 独立线程阻塞（Demo 序列），结果经原子变量回传；
  // gen=目标代数：新目标/取消自增，旧 worker 结果丢弃
  struct ArmWorker {
    std::thread th;
    std::atomic<bool> running{false};
    std::atomic<int> ret{-999};   // -999 = 未完成
    uint64_t gen = 0;
  } arm_worker_[2];
  struct {
    std::shared_ptr<ChassisGoalHandle> handle;
    int8_t mode = 0;
    double direction = 0.0, speed = 0.0, duration = 0.0;
    rclcpp::Time start;
  } chassis_;
  struct {
    std::shared_ptr<ArmGoalHandle> handle;
    int8_t part = 0;
    std::vector<double> targets;
    int arm = 0, dof = 0;
    rclcpp::Time start;
  } arm_;
  struct {
    std::shared_ptr<GripperGoalHandle> handle;
  } gripper_;

  rclcpp_action::Server<ChassisMove>::SharedPtr chassis_server_;
  rclcpp_action::Server<ArmMove>::SharedPtr arm_server_;
  rclcpp_action::Server<GripperMove>::SharedPtr gripper_server_;
  rclcpp::Publisher<robot_interfaces::msg::TRArmMsg>::SharedPtr arm_raw_pub_;
  rclcpp::Publisher<robot_interfaces::msg::TRVehicleMsg>::SharedPtr vehicle_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_, health_timer_, connect_timer_;
};

}  // namespace robot_driver


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_driver::RobotDriverNode>());
  rclcpp::shutdown();
  return 0;
}