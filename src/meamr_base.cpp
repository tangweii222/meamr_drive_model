/**
 * @file meamr_base.cpp
 * @brief Implementation of the MeamrBase class, which provides the base functionality for a differential drive robot.
 * 
 * - Subscribes to `cmd_vel` topic to convert velocity commands into wheel commands for the communication layer.
 * - Calculates odometry based on wheel velocities and publishes it to the `odom_raw` topic.
 * - Updates the robot's state, including position and orientation, based on the differential drive kinematics.
 */

#include "meamr_drive_model/meamr_base.hpp"
#include "meamr_drive_model/vehicle_hardware_data.h"
#include <math.h>

MeamrBase::MeamrBase() : Node("meamr_base_node"),timeout_(rclcpp::Duration::from_seconds(0.2)) 
{
    
    // Declare parameters From Yaml
    this->declare_parameter("wheel_radius", 0.03);
    this->declare_parameter("track_width", 0.30);
    this->declare_parameter("max_vel_x", 0.6);
    this->declare_parameter("max_vel_theta", 0.9);
    this->declare_parameter("std_dev_x", 0.1);
    this->declare_parameter("std_dev_y", 0.1);
    this->declare_parameter("std_dev_z", 0.1);
    this->declare_parameter("std_dev_roll", 0.1);
    this->declare_parameter("std_dev_pitch", 0.1);
    this->declare_parameter("std_dev_yaw", 0.1);
    // Get Hardware Data
    VehicleHardwardData hardware_data;
    this->get_parameter("wheel_radius", hardware_data.wheel_radius);
    this->get_parameter("track_width", hardware_data.track_width);
    this->get_parameter("max_vel_x", hardware_data.max_linear_vel);
    this->get_parameter("max_vel_theta", hardware_data.max_angular_vel);
    // Get Odometry covariance Data
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_footprint";
    this->get_parameter("std_dev_x", odom_.pose.covariance[0]);
    this->get_parameter("std_dev_y", odom_.pose.covariance[7]);
    this->get_parameter("std_dev_z", odom_.pose.covariance[14]);
    this->get_parameter("std_dev_roll", odom_.pose.covariance[21]);
    this->get_parameter("std_dev_pitch", odom_.pose.covariance[28]);
    this->get_parameter("std_dev_yaw", odom_.pose.covariance[35]);

    last_update_time_ = this->now();

    // Publisher and Subscriber
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_raw", 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&MeamrBase::cmdVelCallback, this, std::placeholders::_1));
    // Timer
    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MeamrBase::odomCallback, this));

}

MeamrBase::~MeamrBase()
{
    Stop();
}

void MeamrBase::setSerialInterface(std::shared_ptr<SerialInterface> serial)
{
  serial_interface_ = serial;
}
// 輸出預控制的速度給STM32
void MeamrBase::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
{   
    // 將速度與角速度限制在最大值範圍內
    des_lin_vel_ = std::clamp(msg->linear.x, -max_vel_x_, max_vel_x_);              // m/s
    des_theta_vel_ = std::clamp(msg->angular.z, -max_vel_theta_, max_vel_theta_);   // rad/s

    // 發送速度指令到馬達
    if (serial_interface_) {
        serial_interface_->sendMotorCommand(des_lin_vel_, des_theta_vel_);
    } else {
        RCLCPP_WARN(this->get_logger(), "Serial interface not initialized.");
    }
    
}

int MeamrBase::Init()
{
    return 0;
}

int MeamrBase::Stop()
{
    // 停止動作，例如停止發送轉速指令
    des_lin_vel_ = 0.0;
    des_theta_vel_ = 0.0;
    // 發送停止指令到馬達
    if (serial_interface_) {
        serial_interface_->sendMotorCommand(des_lin_vel_, des_theta_vel_);
    } else {
        RCLCPP_WARN(this->get_logger(), "Serial interface not initialized.");
    }
    return 0;
}

//Odometry 
void MeamrBase::odomCallback()
{
    Update();
    Publish();
}

void MeamrBase::Update()
{
    now_ = this->now();
    dt_ = (now_ - last_update_time_).seconds();
    last_update_time_ = now_;

    double lin_vel = 0.0;
    double theta_vel = 0.0;

    // 從 serial 接收格式為 "$lin_vel$theta_vel"
    std::vector<uint8_t> rx_data = serial_interface_->getLatestRx();
    if (!rx_data.empty()) {
        std::string data_str(rx_data.begin(), rx_data.end());
        size_t pos1 = data_str.find('$');
        size_t pos2 = data_str.find('$', pos1 + 1);
        if (pos1 == 0 && pos2 != std::string::npos) {
        try {
            lin_vel = std::stof(data_str.substr(pos1 + 1, pos2 - pos1 - 1));
            theta_vel = std::stof(data_str.substr(pos2 + 1));
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Parse error: %s", e.what());
        }
        } else {
        RCLCPP_WARN(this->get_logger(), "Invalid serial format: %s", data_str.c_str());
        }
    }
    

    // Update the wheel position and orientation
    double delta_th = theta_vel * dt_;
    double mid_th = theta_ + delta_th / 2.0;
    double delta_x = lin_vel * cos(mid_th) * dt_;
    double delta_y = lin_vel * sin(mid_th) * dt_;

    // Update the robot's position and orientation
    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_th;

    // Update the odometry message
    odom_.pose.pose.position.x = x_;
    odom_.pose.pose.position.y = y_;
    odom_.pose.pose.orientation = createQuaternionFromYaw(theta_);
    odom_.twist.twist.linear.x = lin_vel;
    odom_.twist.twist.angular.z = theta_vel;
}

int MeamrBase::Publish()
{
    odom_.header.stamp = this->now();
    odom_pub_->publish(odom_);
    return 0;
}

geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw)
{
    // Create a quaternion from yaw angle because the robot is 2D
    // and we only need to represent rotation around the Z-axis
    // using the tf2 library
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
}
