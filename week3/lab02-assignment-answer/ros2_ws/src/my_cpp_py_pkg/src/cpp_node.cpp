#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cpp_node");
  RCLCPP_INFO(node->get_logger(), "Hello from CPP!");
  rclcpp::shutdown();
  return 0;
}