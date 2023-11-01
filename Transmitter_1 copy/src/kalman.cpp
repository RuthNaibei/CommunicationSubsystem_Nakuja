#include <BasicLinearAlgebra.h>
#include "defs.h"
#include "kalman.h"


// Define the Kalman filter variables and parameters
float q = 0.0001;
float T = 0.1;

// The system dynamics matrix
BLA::Matrix<3, 3> A = {1.0, 0.1, 0.005,
                       0, 1.0, 0.1,
                       0, 0, 1.0};

// Relationship between measurement and states
BLA::Matrix<3, 3> H = {1.0, 0, 0,
                       0, 0, 1.0};

// Initial posteriori estimate error covariance
BLA::Matrix<3, 3> P = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

// Measurement error covariance
BLA::Matrix<3, 3> R = {0.25, 0, 0,
                       0, 0.75, 0,
                       0, 0, 0.75};

// Process noise covariance
BLA::Matrix<3, 3> Q = {q, q, q,
                       q, q, q,
                       q, q, q};

// Identity Matrix
BLA::Matrix<3, 3> I = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

BLA::Matrix<3, 1> x_hat = {1500.0,
                           0.0,
                           0.0};

// Function to filter altitude, velocity, and acceleration
// struct Filtered_Data {
//     float filtered_altitude;
//     float filtered_velocity;
//     float filtered_acceleration;
// };
// struct Filtered_Data filterData(float x_acceleration, float altitude, float velocity);
 struct Filtered_Data filterData(float x_acceleration, float altitude, float velocity) {
    struct Filtered_Data filtered_values;

    // Measurement matrix
    BLA::Matrix<3, 1> Z = {altitude, velocity, x_acceleration};

    // Predicted state estimate
    BLA::Matrix<3, 1> x_hat_minus = A * x_hat;

    // Predicted estimate covariance
    BLA::Matrix<3, 3> P_minus = A * P * (~A) + Q;

    // Kalman gain
    BLA::Matrix<3, 3> con = (H * P_minus * (~H) + R);
    BLA::Matrix<3, 3> K = P_minus * (~H) * Invert(con);

    // Measurement residual
    BLA::Matrix<3, 1> Y = Z - (H * x_hat_minus);

    // Updated state estimate
    x_hat = x_hat_minus + K * Y;

    // Updated estimate covariance
    P = (I - K * H) * P_minus;

    // Store the filtered values in the struct
    filtered_values.filtered_altitude = x_hat(0);
    filtered_values.filtered_velocity = x_hat(1);
    filtered_values.filtered_acceleration = x_hat(2);

    return filtered_values;
}



