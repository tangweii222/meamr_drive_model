#ifndef _KINEMATIC_MODEL_H_
#define _KINEMATIC_MODEL_H_
#include "vehicle_hardware_data.h"

class KinematicModel
{
public:
    KinematicModel();
    void forward_kinematics(double leftMotorRPM, double rightMotorRPM, const VehicleHardwardData &data, double &outLinearVel, double &outAngularVel);
    void backward_kinematics(double linearVel, double angularVel, const VehicleHardwardData &data, float &outLeftMotorRPM, float &outRightMotorRPM);
};

#endif //_KINEMATIC_MODEL_H_