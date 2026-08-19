// odom_pub：真机里程计适配节点——订阅 /sensors/vehicle_raw（SDK 上报的底盘实际速度，
// vx/vy=mm/s、wz=rad/s，与 setMotionControl 完全对齐），积分出 /odom + odom->Link_Base TF。
// 框架 wheel_odometry 只发 twist 不积分（仿真由 EKF 做），真机无 EKF，本节点补齐。
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "robot_interfaces/msg/tr_vehicle_msg.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kMmToM = 1.0 / 1000.0;
constexpr double kMaxDt = 0.2;

}  // namespace

class OdomPubNode : public rclcpp::Node
{
public:
  OdomPubNode() : Node("odom_pub")
  {
    sub_ = create_subscription<robot_interfaces::msg::TRVehicleMsg>(
        "/sensors/vehicle_raw", 10,
        [this](const robot_interfaces::msg::TRVehicleMsg::SharedPtr m) { on_vehicle(m); });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    last_stamp_ = now();
  }

private:
  void on_vehicle(const robot_interfaces::msg::TRVehicleMsg::SharedPtr m)
  {
    const rclcpp::Time stamp = now();
    double dt = (stamp - last_stamp_).seconds();
    last_stamp_ = stamp;
    if (dt <= 0.0) { dt = 0.0; }
    dt = std::min(dt, kMaxDt);

    const double vx = static_cast<double>(m->vehicle_x_vel) * kMmToM;  // m/s
    const double vy = static_cast<double>(m->vehicle_y_vel) * kMmToM;
    const double wz = static_cast<double>(m->vehicle_z_vel);           // rad/s

    yaw_ += wz * dt;
    x_ += (vx * std::cos(yaw_) - vy * std::sin(yaw_)) * dt;
    y_ += (vx * std::sin(yaw_) + vy * std::cos(yaw_)) * dt;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "Link_Base";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "Link_Base";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<robot_interfaces::msg::TRVehicleMsg>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_stamp_;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPubNode>());
  rclcpp::shutdown();
  return 0;
}