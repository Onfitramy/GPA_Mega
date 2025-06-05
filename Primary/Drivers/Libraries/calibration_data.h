#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H

/* ### CALIBRATION AND OFFSET DATA FOR BOARDS 1 - 5 ### */
#define GPA_MEGA_4

//#define calibration_mode // enable raw sensor data

// IMU1 accelerometer
extern const float IMU1_offset[3]; 
extern const float IMU1_scale[3];

// IMU2 accelerometer
extern const float IMU2_offset[3]; 
extern const float IMU2_scale[3];

// Magnetometer
extern const float MAG_offset[3];
extern const float MAG_scale[3];

#endif // CALIBRATION_DATA_H