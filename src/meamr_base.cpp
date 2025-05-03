#include "meamr_drive_model/meamr_base.hpp"

MeamrBase::MeamrBase() : Node("meamr_base_node"),timeout_(rclcpp::Duration::from_seconds(0.2)) 
{
    
    // Declare and get parameters
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

    last_update_time_ = this->now();
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("track_width", track_width_);
    this->get_parameter("max_vel_x", max_vel_x_);
    this->get_parameter("max_vel_theta", max_vel_theta_);

    double std_dev_x, std_dev_y, std_dev_z, std_dev_roll, std_dev_pitch, std_dev_yaw;
    this->get_parameter("std_dev_x", std_dev_x);
    this->get_parameter("std_dev_y", std_dev_y);
    this->get_parameter("std_dev_z", std_dev_z);
    this->get_parameter("std_dev_roll", std_dev_roll);
    this->get_parameter("std_dev_pitch", std_dev_pitch);
    this->get_parameter("std_dev_yaw", std_dev_yaw);


    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_footprint";
    odom_.pose.covariance[0] = std_dev_x;
    odom_.pose.covariance[7] = std_dev_y;
    odom_.pose.covariance[14] = std_dev_z;
    odom_.pose.covariance[21] = std_dev_roll;
    odom_.pose.covariance[28] = std_dev_pitch;
    odom_.pose.covariance[35] = std_dev_yaw;

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


void MeamrBase::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
{   
    // Clamp the linear and angular velocities to the maximum limits
    des_vel_lin_ = std::clamp(msg->linear.x, -max_vel_x_, max_vel_x_);
    des_vel_theta_ = std::clamp(msg->angular.z, -max_vel_theta_, max_vel_theta_);

    // Compute wheel speeds (m/s) then convert to rad/s
    des_vel_left_ = (des_vel_lin_ - des_vel_theta_ * track_width_ * 0.5) / wheel_radius_;
    des_vel_right_ = (des_vel_lin_ + des_vel_theta_ * track_width_ * 0.5) / wheel_radius_;

    // 發布轉速指令
    // 🔧 印出左右輪速度（rad/s）
    // RCLCPP_INFO(this->get_logger(), "Left vel: %.2f rad/s | Right vel: %.2f rad/s", 
    //             des_vel_left_, des_vel_right_);
}

int MeamrBase::Init()
{
    // 建立通訊埠
    RCLCPP_INFO(this->get_logger(), "Init() skipped SerialMotor, running in test mode.");
    return 0;
}


int MeamrBase::Stop()
{
    // 停止動作，例如停止發送轉速指令
    des_vel_left_ = 0.0;
    des_vel_right_ = 0.0;
    // 發送停止指令到馬達

    return 0;
}

int MeamrBase::ResetOdom()
{
    return -1; // 目前不支援
}

int MeamrBase::ResetMotors()
{
    return -1; // 目前不支援
}

int MeamrBase::Publish()
{

    odom_.header.stamp = this->now();
    odom_pub_->publish(odom_);
    return 0;
}

void MeamrBase::odomCallback()
{
    Update();
    Publish();
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

void MeamrBase::Update()
{
    now_ = this->now();
    dt_ = (now_ - last_update_time_).seconds();
    last_update_time_ = now_;
    // Get the wheel velocities from the motor controller by UART/I2C
    
    // 模擬 wheel velocities -> m/s（暫時不讀實際馬達回報）
    double left_wheel_vel_ = des_vel_left_;
    double right_wheel_vel_ = des_vel_right_;
    

    // Wheel velocities -> m/s
    double left_vel = left_wheel_vel_ * wheel_radius_;
    double right_vel = right_wheel_vel_ * wheel_radius_;


    // Based on the kinematic model of a differential drive robot to calculate the velocity
    double lin_vel = (left_vel + right_vel) * 0.5;
    double theta_vel = (right_vel - left_vel) / track_width_;

    // Update the wheel position and orientation
    double delta_th = theta_vel * dt_;
    double mid_th = theta_ + delta_th / 2.0;
    double delta_x = lin_vel * cos(mid_th) * dt_;
    double delta_y = lin_vel * sin(mid_th) * dt_;

    // Update the robot's position and orientation
    // x_ is the x position of the robot
    // y_ is the y position of the robot
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
