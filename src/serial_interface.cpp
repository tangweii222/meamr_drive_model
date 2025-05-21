#include "meamr_drive_model/serial_interface.hpp"

SerialInterface::SerialInterface()
: Node("serial_interface")
{
  serial_write_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/serial_write", 10);
  serial_read_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/serial_read", 10,
    std::bind(&SerialInterface::serialReadCallback, this, std::placeholders::_1)
  );
}

void SerialInterface::sendMotorCommand(double lin_vel, double theta_vel)
{
    std_msgs::msg::UInt8MultiArray msg;

    std::ostringstream ss;
    ss << "$" << lin_vel << "$" << theta_vel;
    std::string data_str = ss.str();

    msg.data.assign(data_str.begin(), data_str.end());
    serial_write_pub_->publish(msg);
}

void SerialInterface::serialReadCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    
    std::lock_guard<std::mutex> lock(rx_mutex_);
    last_rx_data_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Received serial_read, size: %ld", msg->data.size());

}

std::vector<uint8_t> SerialInterface::getLatestRx()
{
  std::lock_guard<std::mutex> lock(rx_mutex_);
  
  return last_rx_data_;
}
