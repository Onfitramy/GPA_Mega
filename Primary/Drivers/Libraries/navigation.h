#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "armMathAddon.h"

#define x_size1 6
#define z_size1 3
#define u_size1 3

#define x_size2 9
#define z_size2 6
#define u_size2 3

// standard KF variables
typedef struct {
    uint8_t x_size;
    uint8_t z_size;
    uint8_t u_size;

    float *dx;
    float *dz;

    arm_matrix_instance_f32 M1;
    arm_matrix_instance_f32 M2;
    arm_matrix_instance_f32 M3;
    arm_matrix_instance_f32 M4;
    arm_matrix_instance_f32 M5;
    arm_matrix_instance_f32 M6;
    arm_matrix_instance_f32 M7;
} Kalman_Instance;

// stores actual data for orientation KF
typedef struct {
    float dx[x_size1];
    float dz[z_size1];

    float M1[x_size1*x_size1];
    float M2[x_size1*x_size1];
    float M3[z_size1*z_size1];
    float M4[z_size1*z_size1];
    float M5[x_size1*z_size1];
    float M6[x_size1*z_size1];
    float M7[z_size1*x_size1];
} x6z3u3KalmanData;

// stores actual data for position KF
typedef struct {
    float dx[x_size2];
    float dz[z_size2];

    float M1[x_size2*x_size2];
    float M2[x_size2*x_size2];
    float M3[z_size2*z_size2];
    float M4[z_size2*z_size2];
    float M5[x_size2*z_size2];
    float M6[x_size2*z_size2];
    float M7[z_size2*x_size2];
} x9z6u3KalmanData;

void normalizeAngle(float* angle, float upper_boundary, float lower_boundary, float step_size);
void normalizeAngleVector(float* angle_vec, uint8_t pos_top, uint8_t pos_bottom, float upper_boundary, float lower_boundary, float step_size);

void normalizeAnglePair(float angle_lead, float* angle_mod, float upper_boundary, float lower_boundary, float step_size);
void normalizeAnglePairVector(float* angle_lead_vec, float* angle_mod_vec, uint8_t pos_top, uint8_t pos_bottom, float upper_boundary, float lower_boundary, float step_size);

void UBLOXtoWGS84(int32_t lat_e7, int32_t lon_e7, int32_t height_e3, double* WGS84_vec);
void WGS84toECEF(double* WGS84, double* ECEF_vec);
void ECEFtoENU(double* WGS84_ref, double* ECEF_ref, double* ECEF, double* ENU);

void OrientationFix(float* accel_vec, float* mag_vec, float* output_angles);

void RotationMatrixFromEuler(float phi, float theta, float psi, arm_matrix_instance_f32* M_out);
void DeulerMatrixFromEuler(float phi, float theta, arm_matrix_instance_f32 *mat);
void Vec3_BodyToWorld(float *vec3_body, arm_matrix_instance_f32 *mat_rot, float *vec3_world);

void KalmanFilterInit(Kalman_Instance *Kalman, x6z3u3KalmanData *data, uint8_t x_vec_size, uint8_t z_vec_size, uint8_t u_vec_size);
void KalmanFilterPredictSV(Kalman_Instance *Kalman, arm_matrix_instance_f32 *A_mat, float *x_vec, arm_matrix_instance_f32 *B_mat, float *u_vec);
void KalmanFilterPredictCM(Kalman_Instance *Kalman, arm_matrix_instance_f32 *A_mat, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *Q_mat);
void KalmanFilterUpdateGain(Kalman_Instance *Kalman, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *C_mat, arm_matrix_instance_f32 *R_mat, arm_matrix_instance_f32 *K_mat);
void KalmanFilterCorrectSV(Kalman_Instance *Kalman, arm_matrix_instance_f32 *K_mat, float *z_vec, arm_matrix_instance_f32 *C_mat, float *x_vec);
void KalmanFilterCorrectCM(Kalman_Instance *Kalman, arm_matrix_instance_f32 *K_mat, arm_matrix_instance_f32 *C_mat, arm_matrix_instance_f32 *P_mat);

#endif // NAVIGATION_H