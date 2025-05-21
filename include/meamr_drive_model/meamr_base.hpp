#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "dd_kinematic_model.hpp"
#include "vehicle_hardware_data.h"
#include "meamr_drive_model/serial_interface.hpp"


class MeamrBase : public rclcpp::Node
{
public:
    MeamrBase();
    ~MeamrBase();

    int Init();
    int Stop();
    int Publish();
    void Update();
    void setSerialInterface(std::shared_ptr<SerialInterface> serial);
    
private:

    void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);
    void odomCallback();
    // Robot configuration
    VehicleHardwardData hardware_data_;
    double wheel_radius_;
    double track_width_;
    double max_vel_x_;
    double max_vel_theta_;
    // Robot State
    double des_lin_vel_;        
    double des_theta_vel_;          
    double x_, y_, theta_;                    
    // Time Control
    double dt_;
    rclcpp::Time now_;              
    rclcpp::Time last_update_time_; 
    rclcpp::Duration timeout_;      

    // ROS2 Interface
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; 
    rclcpp::TimerBase::SharedPtr odom_timer_;                                   
    nav_msgs::msg::Odometry odom_;     

    // Serial Interface
    std::shared_ptr<SerialInterface> serial_interface_;    
    KinematicModel kinematic_model_;
};

