#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/tr_vehicle_msg.hpp"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("test_node")
  {
    // 创建发布者，话题 "test_topic"，队列大小 10
    publisher_ = this->create_publisher<robot_interfaces::msg::TRVehicleMsg>("TRVehicleMsg", 10);

    // 创建订阅者，订阅同一话题，回调函数为 topic_callback
    subscriber_ = this->create_subscription<robot_interfaces::msg::TRVehicleMsg>(
      "TRVehicleMsg", 10, std::bind(&TestNode::topic_callback, this, std::placeholders::_1));

    // 创建定时器，每 1 秒触发一次 timer_callback
    timer_ = this->create_wall_timer(1s, std::bind(&TestNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = robot_interfaces::msg::TRVehicleMsg();
    message.vehicle_servo_state = 1;  // 设置消息内容
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.vehicle_servo_state);
    publisher_->publish(message);
  }

  void topic_callback(const robot_interfaces::msg::TRVehicleMsg::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->vehicle_servo_state);
  }

  rclcpp::Publisher<robot_interfaces::msg::TRVehicleMsg>::SharedPtr publisher_;
  rclcpp::Subscription<robot_interfaces::msg::TRVehicleMsg>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}