#ifndef _VEHICLE_HARDWARD_DATA_H_
#define _VEHICLE_HARDWARD_DATA_H_

struct VehicleHardwardData
{

    double max_linear_vel;  // x coordinate, unit: m/s
    double max_angular_vel; // z coordinate, unit: rad/s

    double track_width;  // Unit: meter , distance between the centers of two wheels.
    double wheel_radius; // Unit: meter
};

#endif // _VEHICLE_HARDWARD_DATA_H_