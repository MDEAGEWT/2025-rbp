#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class InfoSender : public rclcpp::Node
{
public:
  InfoSender()
  : Node("info_sender")
  {
    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("data_reciever/msg", 10);

    RCLCPP_INFO(this->get_logger(), "Info sender started, publishing student info");

    // Create timer to publish data every 1 second
    timer_ = this->create_wall_timer(1000ms, std::bind(&InfoSender::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "2024298016 김민규";

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InfoSender>());
  rclcpp::shutdown();
  return 0;
}
