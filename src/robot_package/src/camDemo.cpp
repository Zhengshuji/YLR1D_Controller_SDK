#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "CamDev/CamDev_Client.h"

#include <vector>
#include <string>

using namespace DevLayer;

class CamDevNode : public rclcpp::Node
{
public:
    CamDevNode() : Node("camdev_node")
    {
        // 声明参数（允许运行时修改）
        this->declare_parameter<std::string>("server_ip", "127.0.0.1");
        this->declare_parameter<int>("server_port", 8201);
        this->declare_parameter<bool>("color_mode", true);  // true: 彩色, false: 深度

        // 获取参数
        std::string ip = this->get_parameter("server_ip").as_string();
        int port = this->get_parameter("server_port").as_int();
        color_mode_ = this->get_parameter("color_mode").as_bool();

        // 初始化相机客户端
        if (cam_.init(ip.c_str(), port) != 0) {
            RCLCPP_ERROR(this->get_logger(), "CamDev init failed!");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CamDev init success");

        // 创建图像发布者（话题名可自定义）
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

        // 使用定时器周期性捕获图像（频率可调，此处设为 20Hz）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CamDevNode::timer_callback, this)
        );
    }

    ~CamDevNode()
    {
        cam_.close();
    }

private:
    void timer_callback()
    {
        // 捕获图像数据
        uint8_t imgBuf[IMG_BUF_LEN];  // 需提前定义 IMG_BUF_LEN，或动态分配
        int len;
        if (color_mode_) {
            len = cam_.capture(imgBuf, CAM_DEV_IMG_TYPE_COLOR);
        } else {
            len = cam_.capture(imgBuf, CAM_DEV_IMG_TYPE_DEPTH);
        }

        if (len < 0) {
            RCLCPP_ERROR(this->get_logger(), "Capture failed, len=%d", len);
            return;
        }

        // 跳过图像头（假设 IMGHEAD 是字符串常量，例如 "#IMAGE#"）
        std::string header = IMGHEAD;  // 需定义
        if (len < static_cast<int>(header.length())) {
            RCLCPP_WARN(this->get_logger(), "Data too short, skip");
            return;
        }
        std::vector<uint8_t> img_bytes(imgBuf + header.length(), 
                                       imgBuf + header.length() + len);
        
        RCLCPP_INFO(this->get_logger(), "Received %d bytes (header: %d bytes, image: %d bytes)", 
            len, header.length(), img_bytes.size());

        // 解码为 cv::Mat
        cv::Mat img;
        if (color_mode_) {
            img = cv::imdecode(img_bytes, cv::IMREAD_COLOR);
        } else {
            img = cv::imdecode(img_bytes, cv::IMREAD_ANYDEPTH);
        }

        if (img.empty()) {
            RCLCPP_WARN(this->get_logger(), "Decoded image is empty");
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Captured image %dx%d", img.rows, img.cols);

        // 转换为 ROS 图像消息并发布
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), 
                                      color_mode_ ? "bgr8" : "mono16", 
                                      img).toImageMsg();
        msg->header.stamp = this->now();
        image_pub_->publish(*msg);
    }

    CamDev_Client cam_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool color_mode_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamDevNode>());
    rclcpp::shutdown();
    return 0;
}