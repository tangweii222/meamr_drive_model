#include "meamr_drive_model/dd_kinematic_model.hpp"
#include <math.h>
#include <algorithm>

KinematicModel::KinematicModel()
{
}

// Input : left and right wheel RPM
// Output : linear and angular velocity of the robot
void KinematicModel::forward_kinematics(double lMotorRadPerSec, double rMotorRadPerSec, const VehicleHardwardData &data, double &outLinearVel, double &outAngularVel)
{   
    // RPM to rad/sec
    // double lMotorRadPerSec = leftMotorRPM * (2 * M_PI) / 60;
    // double rMotorRadPerSec = rightMotorRPM * (2 * M_PI) / 60;

    double leftVel = 1 * lMotorRadPerSec * data.wheel_radius;
    double rightVel = 1 * rMotorRadPerSec * data.wheel_radius;

    outLinearVel = (leftVel + rightVel) * 0.5;
    outAngularVel = (rightVel - leftVel) / data.track_width;
}

// Input : linear and angular velocity of the robot
// Output : left and right wheel RPM
void KinematicModel::backward_kinematics(double linearVel, double angularVel, const VehicleHardwardData &data, float &outLeftMotorRPM, float &outRightMotorRPM)
{
    double trackWidth = data.track_width;
    double wheelRadius = data.wheel_radius;
    // Speed limits
    float desVelLin = std::max(-1 * data.max_linear_vel, std::min(linearVel, data.max_linear_vel));
    float desVelTheta = std::max(-1 * data.max_angular_vel, std::min(angularVel, data.max_angular_vel));

    //  Computing left and right wheels velocity (differential drive)
    float desVelLeft = desVelLin - desVelTheta * trackWidth * 0.5;
    float desVelRight = desVelLin + desVelTheta * trackWidth * 0.5;

    // linear velocity (m/s) to angular velocity (rad/s)
    desVelLeft /= 1.0 * wheelRadius;
    desVelRight /= 1.0 * wheelRadius;

    // rad/s to RPM
    outLeftMotorRPM = desVelLeft * 60 / (2 * M_PI);
    outRightMotorRPM = desVelRight * 60 / (2 * M_PI);
}