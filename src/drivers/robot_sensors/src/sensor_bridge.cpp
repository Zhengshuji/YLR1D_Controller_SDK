// 传感器层（真机）：订阅 /sensors/arm_raw（TRArmMsg，按 printState 模式取好的关节+末端）
// 与 /sensors/vehicle_raw（TRVehicleMsg，SDK 上报的底盘速度/轮速/舵角），组装成框架
// 30 关节名的 /joint_states —— ylr1d_perception 的 joint_state_receiver 输入源（原为 Gazebo）。
// 关节名单一来源：ylr1d_description/config/joint_config.hpp（kSteering/kWheels/kTorso/kLeftArm/kRightArm）。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robot_interfaces/msg/tr_arm_msg.hpp"
#include "robot_interfaces/msg/tr_vehicle_msg.hpp"
#include "ylr1d_description/config/joint_config.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace {

// 夹指位置语义（与 ylr1d_translate constants.hpp finger_target 对齐）
constexpr double kLeftFingerOpen = -0.014, kLeftFingerClose = 0.0;
constexpr double kRightFingerOpen = 0.0, kRightFingerClose = 0.014;

// 带越界保护的取值
inline double at(const std::vector<float> & v, size_t i)
{
  return i < v.size() ? static_cast<double>(v[i]) : 0.0;
}

}  // namespace

class SensorBridgeNode : public rclcpp::Node
{
public:
  SensorBridgeNode() : Node("sensor_bridge")
  {
    arm_sub_ = create_subscription<robot_interfaces::msg::TRArmMsg>(
        "/sensors/arm_raw", 10,
        [this](const robot_interfaces::msg::TRArmMsg::SharedPtr m) {
          // 按 msg.name（"arm0"/"arm1"）归档；未知名忽略
          if (m->name == "arm0") { arms_[0] = m; }
          else if (m->name == "arm1") { arms_[1] = m; }
          else if (m->name == "arm2") { torso_ = m; }   // 躯干（ARM_3，4 关节）
        });
    vehicle_sub_ = create_subscription<robot_interfaces::msg::TRVehicleMsg>(
        "/sensors/vehicle_raw", 10,
        [this](const robot_interfaces::msg::TRVehicleMsg::SharedPtr m) { vehicle_ = m; });
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               [this]() { tick(); });
  }

private:
  /// 填一组关节：<7 为臂关节（取 SDK 关节角），>=7 为夹指（取夹爪开合）
  void fill_group(sensor_msgs::msg::JointState & js,
                  const char * const * names, size_t count, size_t arm_idx)
  {
    const auto & arm = arms_[arm_idx];
    const bool claw = arm ? arm->claw_state : false;
    for (size_t i = 0; i < count; ++i) {
      double pos = 0.0;
      if (i < 7) {
        pos = arm ? at(arm->joints, i) : 0.0;
      } else {
        pos = arm_idx == 0 ? (claw ? kLeftFingerOpen : kLeftFingerClose)
                           : (claw ? kRightFingerOpen : kRightFingerClose);
      }
      js.name.push_back(names[i]);
      js.position.push_back(pos);
      js.velocity.push_back(0.0);
      js.effort.push_back(0.0);
    }
  }

  void tick()
  {
    using namespace ylr1d_description;
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();

    // 4 转向：位置 = SDK servo_pos（rad）
    for (size_t i = 0; i < kSteeringJointCount; ++i) {
      js.name.push_back(kSteeringJoints[i]);
      js.position.push_back(vehicle_ ? at(vehicle_->servo_pos, i) : 0.0);
      js.velocity.push_back(0.0);
      js.effort.push_back(0.0);
    }
    // 4 轮：速度 = SDK wheel_vel（rad/s）
    for (size_t i = 0; i < kWheelJointCount; ++i) {
      js.name.push_back(kWheelJoints[i]);
      js.position.push_back(0.0);
      js.velocity.push_back(vehicle_ ? at(vehicle_->wheel_vel, i) : 0.0);
      js.effort.push_back(0.0);
    }
    // 4 躯干：来自 arm2（ARM_3，驱动已换算弧度）；无数据填 0
    for (size_t i = 0; i < kTorsoJointCount; ++i) {
      js.name.push_back(kTorsoJoints[i]);
      js.position.push_back(torso_ ? at(torso_->joints, i) : 0.0);
      js.velocity.push_back(0.0);
      js.effort.push_back(0.0);
    }
    // 左臂 9（7 关节 + 2 夹指）<- arm0；右臂 9 <- arm1
    fill_group(js, kLeftArmJoints, kLeftArmJointCount, 0);
    fill_group(js, kRightArmJoints, kRightArmJointCount, 1);

    if (js.name.size() != 30u) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "joint count %zu != 30, skip frame", js.name.size());
      return;
    }
    js_pub_->publish(js);
  }

  rclcpp::Subscription<robot_interfaces::msg::TRArmMsg>::SharedPtr arm_sub_;
  rclcpp::Subscription<robot_interfaces::msg::TRVehicleMsg>::SharedPtr vehicle_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<robot_interfaces::msg::TRArmMsg::SharedPtr, 2> arms_;
  robot_interfaces::msg::TRArmMsg::SharedPtr torso_;
  robot_interfaces::msg::TRVehicleMsg::SharedPtr vehicle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorBridgeNode>());
  rclcpp::shutdown();
  return 0;
}