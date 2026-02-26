#ifndef HIL_H
#define HIL_H

#include "main.h"

#define RAIL_LENGTH     6       // m
#define WET_MASS        18.454  // kg
#define IGNITION_TIME   0       // s
#define MOTOR_BURN_TIME 5       // s
#define MOTOR_IMPULSE   5880.2  // Ns
#define PROPELLANT_MASS 3.454   // kg
#define MOTOR_THRUST    MOTOR_IMPULSE / MOTOR_BURN_TIME;

typedef struct {
    float t;
    float m;
    float r_i[3];
    float v_i[3];
    float v_b[3];
    float w_b[3];
    float a_i[3];
    float a_b[3];
    arm_matrix_instance_f32 *DCM_bi;
    arm_matrix_instance_f32 *DCM_ib;
} HIL_t;

extern HIL_t HIL;

void HILInit();
void HILupdateStates(float dt);

void HILgetIMUData(IMU_AverageData_t *HIL_imu_data);
void HILgetMagnetometerData(LIS3MDL_Data_t *HIL_mag_data);
void HILgetBarometerData(bmp390_data_t *HIL_bmp_data);
void HILgetGPSData(UBX_NAV_PVT *HIL_gps_data);

#endif