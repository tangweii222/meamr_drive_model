#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>

class SerialInterface : public rclcpp::Node
{
public:
  SerialInterface();

  // 發送速度命令給 UART
  void sendMotorCommand(double lin_vel, double theta_vel);

  // 取得上一次收到的原始資料
  std::vector<uint8_t> getLatestRx();

private:
  void serialReadCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_write_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_read_sub_;

  std::mutex rx_mutex_;
  std::vector<uint8_t> last_rx_data_;
};
