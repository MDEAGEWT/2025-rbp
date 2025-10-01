#include <cstdio>
#include <chrono>
#include <memory>
#include <thread>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

class DataReceiver : public rclcpp::Node
{
public:
  DataReceiver()
  : Node("data_receiver"), latest_tcp_value_(0), latest_udp_value_(0),
    tcp_received_(false), udp_received_(false), previous_winner_value_(0), has_previous_winner_(false)
  {
    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::UInt8>("topic_generator/msg", 10);

    // Setup TCP server (port 5005)
    tcp_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create TCP socket");
      return;
    }

    int opt = 1;
    setsockopt(tcp_sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Set socket timeout to avoid blocking forever
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(tcp_sockfd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in tcp_addr;
    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_addr.s_addr = INADDR_ANY;
    tcp_addr.sin_port = htons(5005);

    if (bind(tcp_sockfd_, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind TCP socket");
      return;
    }

    if (listen(tcp_sockfd_, 1) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to listen on TCP socket");
      return;
    }

    // Setup UDP server (port 5006)
    udp_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
      return;
    }

    struct sockaddr_in udp_addr;
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_addr.s_addr = INADDR_ANY;
    udp_addr.sin_port = htons(5006);

    if (bind(udp_sockfd_, (struct sockaddr*)&udp_addr, sizeof(udp_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket");
      return;
    }

    // Set UDP socket timeout
    setsockopt(udp_sockfd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    RCLCPP_INFO(this->get_logger(), "TCP server listening on port 5005");
    RCLCPP_INFO(this->get_logger(), "UDP server listening on port 5006");

    // Start receiver threads
    tcp_thread_ = std::thread(&DataReceiver::tcp_receiver_thread, this);
    udp_thread_ = std::thread(&DataReceiver::udp_receiver_thread, this);

    // Create timer to compare values every 0.3 seconds
    timer_ = this->create_wall_timer(300ms, std::bind(&DataReceiver::compare_callback, this));
  }

  ~DataReceiver()
  {
    running_ = false;
    if (tcp_thread_.joinable()) tcp_thread_.join();
    if (udp_thread_.joinable()) udp_thread_.join();
    if (tcp_sockfd_ >= 0) close(tcp_sockfd_);
    if (udp_sockfd_ >= 0) close(udp_sockfd_);
    if (tcp_client_sockfd_ >= 0) close(tcp_client_sockfd_);
  }

private:
  void tcp_receiver_thread()
  {
    while (running_) {
      // Accept TCP connection with timeout handling
      struct sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      tcp_client_sockfd_ = accept(tcp_sockfd_, (struct sockaddr*)&client_addr, &client_len);

      if (tcp_client_sockfd_ < 0) {
        if (!running_) break;
        std::this_thread::sleep_for(100ms);
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "TCP client connected");

      while (running_) {
        uint8_t received_value;
        ssize_t bytes_received = recv(tcp_client_sockfd_, &received_value, sizeof(received_value), 0);

        if (bytes_received == sizeof(received_value)) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_tcp_value_ = received_value;
          tcp_received_ = true;
          RCLCPP_DEBUG(this->get_logger(), "TCP received: %u", latest_tcp_value_);
        } else if (bytes_received <= 0) {
          if (!running_) break;
          // Timeout or error, check if we should continue
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
          }
          break;
        }
      }

      if (tcp_client_sockfd_ >= 0) {
        close(tcp_client_sockfd_);
        tcp_client_sockfd_ = -1;
      }
    }
  }

  void udp_receiver_thread()
  {
    while (running_) {
      uint8_t received_value;
      struct sockaddr_in sender_addr;
      socklen_t sender_len = sizeof(sender_addr);

      ssize_t bytes_received = recvfrom(udp_sockfd_, &received_value, sizeof(received_value),
                                       0, (struct sockaddr*)&sender_addr, &sender_len);

      if (bytes_received == sizeof(received_value)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_udp_value_ = received_value;
        udp_received_ = true;
        RCLCPP_DEBUG(this->get_logger(), "UDP received: %u", latest_udp_value_);
      } else if (bytes_received < 0) {
        if (!running_) break;
        // Handle timeout (EAGAIN/EWOULDBLOCK) gracefully
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        RCLCPP_WARN(this->get_logger(), "UDP receive error: %s", strerror(errno));
      }
    }
  }

  void compare_callback()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!tcp_received_ || !udp_received_) {
      return;
    }

    uint8_t winner_value;

    if (latest_udp_value_ > latest_tcp_value_) {
      winner_value = latest_udp_value_;
      RCLCPP_INFO(this->get_logger(), "UDP was larger. UDP: %u, TCP: %u",
                  latest_udp_value_, latest_tcp_value_);
    } else if (latest_tcp_value_ > latest_udp_value_) {
      winner_value = latest_tcp_value_;
      RCLCPP_INFO(this->get_logger(), "TCP was larger. TCP: %u, UDP: %u",
                  latest_tcp_value_, latest_udp_value_);
    } else {
      // 같은 경우: 이전 winner 값 유지 (첫 비교시엔 0)
      winner_value = previous_winner_value_;
      RCLCPP_INFO(this->get_logger(), "Values are equal. Both: %u", latest_tcp_value_);
    }

    // winner_value 저장 및 퍼블리시
    previous_winner_value_ = winner_value;
    has_previous_winner_ = true;

    auto message = std_msgs::msg::UInt8();
    message.data = winner_value;
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int tcp_sockfd_;
  int udp_sockfd_;
  int tcp_client_sockfd_ = -1;

  std::thread tcp_thread_;
  std::thread udp_thread_;
  std::atomic<bool> running_{true};

  std::mutex data_mutex_;
  uint8_t latest_tcp_value_;
  uint8_t latest_udp_value_;
  bool tcp_received_;
  bool udp_received_;
  uint8_t previous_winner_value_;
  bool has_previous_winner_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataReceiver>());
  rclcpp::shutdown();
  return 0;
}
