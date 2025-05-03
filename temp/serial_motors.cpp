#include "meamr_drive_model/serial_motors.hpp"
#include <sstream>

SerialMotor::SerialMotor(rclcpp::Node *node, const std::string &topic_name, double wheel_radius)
    : wheel_radius_(wheel_radius)
{
    serial_pub_ = node->create_publisher<std_msgs::msg::String>(topic_name, 10);
}

SerialMotor::~SerialMotor() = default;

void SerialMotor::setVelRPM(double left_rpm, double right_rpm)
{
    std::ostringstream ss;
    ss << "L" << left_rpm << "R" << right_rpm;
    std_msgs::msg::String msg;
    msg.data = ss.str();
    serial_pub_->publish(msg);
}

void SerialMotor::setVel(double left_vel_mps, double right_vel_mps, double wheel_radius)
{
    wheel_radius_ = wheel_radius;
    double left_rpm = (left_vel_mps / wheel_radius_) * 60.0 / (2.0 * M_PI);
    double right_rpm = (right_vel_mps / wheel_radius_) * 60.0 / (2.0 * M_PI);
    setVelRPM(left_rpm, right_rpm);
}
