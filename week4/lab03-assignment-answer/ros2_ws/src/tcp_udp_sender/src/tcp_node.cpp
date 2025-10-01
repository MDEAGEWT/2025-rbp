#include <cstdio>
#include <chrono>
#include <memory>
#include <random>
#include <limits>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TcpNode : public rclcpp::Node
{
public:
  TcpNode()
  : Node("tcp_node"), gen_(rd_()), dis_(0, 255)
  {
    // Create TCP socket
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
      return;
    }

    // Setup server address
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(5005);
    server_addr_.sin_addr.s_addr = inet_addr("127.0.0.1");

    // Connect to server
    if (connect(sockfd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to connect to server on port 5005");
    } else {
      RCLCPP_INFO(this->get_logger(), "Connected to TCP server on port 5005");
    }

    // Create timer to send data every 0.5 seconds
    auto timer_callback = [this]() -> void {
      uint8_t random_number = dis_(gen_);

      ssize_t bytes_sent = send(sockfd_, &random_number, sizeof(random_number), 0);
      if (bytes_sent > 0) {
        RCLCPP_INFO(this->get_logger(), "Sent: %u", random_number);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to send data");
      }
    };

    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

  ~TcpNode()
  {
    if (sockfd_ >= 0) {
      close(sockfd_);
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  int sockfd_;
  struct sockaddr_in server_addr_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_int_distribution<uint8_t> dis_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpNode>());
  rclcpp::shutdown();
  return 0;
}
