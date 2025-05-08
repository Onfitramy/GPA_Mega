#include "calibration_data.h"

// IMU1 accelerometer
const float IMU1_offset[3] = {-0.005, -0.085, 0.17}; 
const float IMU1_scale[3] = {0.9954, 0.9944, 0.9959};

// IMU2 accelerometer
const float IMU2_offset[3] = {0, 0, 0}; 
const float IMU2_scale[3] = {1, 1, 1};

// Magnetometer
const float MAG_offset[3] = {0.3082, -0.0361, -0.48925};
const float MAG_scale[3] = {1.1543, 1.1094, 1.2785};