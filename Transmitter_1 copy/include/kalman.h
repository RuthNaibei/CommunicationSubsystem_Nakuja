#ifndef KALMAN_H
#define KALMAN_H

struct Filtered_Data {
    float filtered_altitude;
    float filtered_velocity;
    float filtered_acceleration;
};

struct Filtered_Data filterData(float x_acceleration, float altitude, float velocity);


#endif
