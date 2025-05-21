#ifndef MEAMR_BASE_H
#define MEAMR_BASE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "dd_kinematic_model.hpp"
#include "vehicle_hardware_data.h"
#include "serial_interface.hpp"

// #include "meamr_drive_model/serial_motors.hpp"

class MeamrBase : public rclcpp::Node
/**
 * @class MeamrBase
 * @brief A base class for controlling a mobile robot with differential drive.
 * 
 * This class provides methods for initializing the controller, stopping the robot,
 * resetting odometry and motor states, publishing odometry information, and updating
 * the robot's state based on wheel velocities. It also handles ROS2 communication
 * for receiving velocity commands and publishing odometry data.
 */
{
public:
    /**
     * @brief Constructor for the MeamrBase class.
     */
    MeamrBase();

    /**
     * @brief Destructor for the MeamrBase class.
     */
    ~MeamrBase();

    /**
     * @brief Initializes the controller.
     * @return int Status code indicating success or failure.
     */
    int Init();

    /**
     * @brief Stops all robot actions.
     * @return int Status code indicating success or failure.
     */
    int Stop();

    /**
     * @brief Resets the odometry to its initial state (if supported).
     * @return int Status code indicating success or failure.
     */
    int ResetOdom();

    /**
     * @brief Resets the motor states (if supported).
     * @return int Status code indicating success or failure.
     */
    int ResetMotors();

    /**
     * @brief Publishes odometry information.
     * @return int Status code indicating success or failure.
     */
    int Publish();

    /**
     * @brief Updates the odometry information based on the left and right wheel velocities.
     */
    void Update();

private:
    /**
     * @brief Callback function to receive cmd_vel commands.
     * @param msg Shared pointer to the Twist message containing velocity commands.
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);

    /**
     * @brief Callback function to periodically update odometry information.
     */
    void odomCallback();

    KinematicModel kinematic_model_;


    // Parameters and Variables
    VehicleHardwardData hardware_data_;
    double wheel_radius_;       ///< Radius of the wheels (in meters).
    double track_width_;        ///< Distance between the left and right wheels (in meters).
    double gear_ratio_;         ///< Gear ratio of the motor.

    double wheel_acc_;          ///< Maximum wheel acceleration (in m/s^2).
    double wheel_dec_;          ///< Maximum wheel deceleration (in m/s^2).
    double max_vel_x_;          ///< Maximum linear velocity (in m/s).
    double max_vel_theta_;      ///< Maximum angular velocity (in rad/s).

    double des_lin_vel_;        ///< Desired linear velocity (in m/s).
    double des_theta_vel_;      ///< Desired angular velocity (in rad/s).
    double des_vel_left_;       ///< Desired velocity for the left wheel (in m/s).
    double des_vel_right_;      ///< Desired velocity for the right wheel (in m/s).

    double left_wheel_vel_;     ///< Current velocity of the left wheel (in m/s).
    double right_wheel_vel_;    ///< Current velocity of the right wheel (in m/s).
    double x_;                  ///< Current x position of the robot (in meters).
    double y_;                  ///< Current y position of the robot (in meters).
    double theta_;              ///< Current orientation of the robot (in radians).

    double dt_;                 ///< Time step for odometry updates (in seconds).

    double prev_x_;             ///< Previous x position of the robot (in meters).
    double prev_y_;             ///< Previous y position of the robot (in meters).
    double prev_theta_;         ///< Previous orientation of the robot (in radians).

    bool publish_tf_;           ///< Flag to indicate whether to publish TF transformations.

    // Time Control
    rclcpp::Time now_;              ///< Current time.
    rclcpp::Time last_update_time_; ///< Time of the last update.
    rclcpp::Duration timeout_;      ///< Timeout duration for velocity commands.

    // ROS2 Communication Objects
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;            ///< Publisher for odometry messages.
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;    ///< Subscriber for velocity commands.
    rclcpp::TimerBase::SharedPtr odom_timer_;                                   ///< Timer for periodic odometry updates.

    // Odometry Message
    nav_msgs::msg::Odometry odom_;                                              ///< Odometry message to be published.
    
    // Serial Interface
    std::shared_ptr<SerialInterface> serial_interface_;                         ///< Serial interface for communication with the robot.
};

#endif  // MEAMR_BASE_H
