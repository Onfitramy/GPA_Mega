#ifndef HIL_H
#define HIL_H

#include "_components.h"

#define HIL_RAIL_LENGTH     6.0f     // m
#define HIL_WET_MASS        22.343f  // kg
#define HIL_IGNITION_TIME   5.0f     // s
#define HIL_MOTOR_BURN_TIME 5.0f     // s
#define HIL_MOTOR_IMPULSE   5880.2f  // Ns
#define HIL_PROPELLANT_MASS 3.454f   // kg
#define HIL_CD_S_DROGUE     0.467f   // m^2
#define HIL_CD_S_MAIN       5.77f    // m^2
#define HIL_C_ROLL          0.01f    // (rad/s)/(m/s)
#define HIL_MOTOR_THRUST    HIL_MOTOR_IMPULSE / HIL_MOTOR_BURN_TIME;

typedef enum {
    HIL_STATE_PRELAUNCH,
    HIL_STATE_BURN,
    HIL_STATE_COAST,
    HIL_STATE_DESCEND,
    HIL_STATE_DROGUE,
    HIL_STATE_MAIN,
    HIL_STATE_LANDED,
} HIL_State_t;

typedef struct {
    HIL_State_t state;
    float t;
    float m;
    float F_b[3];
    float r_i[3];
    float v_i[3];
    float v_b[3];
    float w_b[3];
    float a_i[3];
    float a_b[3];
    float euler[3];
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