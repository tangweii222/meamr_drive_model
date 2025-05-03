#ifndef MEAMR_SERIAL_MOTOR_H
#define MEAMR_SERIAL_MOTOR_H

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SerialMotor
{
public:
    SerialMotor(rclcpp::Node *node, const std::string &topic_name, double wheel_radius);
    ~SerialMotor();

    void setVelRPM(double left_rpm, double right_rpm);
    void setVel(double left_vel_mps, double right_vel_mps, double wheel_radius);

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_pub_;
    double wheel_radius_;
};

#endif
