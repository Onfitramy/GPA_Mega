#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "armMathAddon.h"

#define magnetic_dip_angle 64.0f

#define x_size1 6
#define z_size1 3
#define u_size1 3

#define x_size2 9
#define z_size2 6
#define u_size2 3

#define x_size3 4
#define z_size3 6
#define u_size3 0

typedef enum {
    DCM_bi_WorldToBody = -1,
    DCM_ib_BodyToWorld = 1
} direct_cosine_matrix_t;

// standard KF variables
typedef struct {
    uint8_t x_size;
    uint8_t z_size;
    uint8_t u_size;
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

void EulerOrientationFix(float *a_vec, float *m_vec, float *output_angles);
void RotationMatrixOrientationFix(float *a_vec, float *m_vec, arm_matrix_instance_f32 *mat, direct_cosine_matrix_t dcm_type);

void RotationMatrixFromEuler(float phi, float theta, float psi, arm_matrix_instance_f32* M_out);
void EulerFromRotationMatrix(arm_matrix_instance_f32 *mat, float *euler);
void RotationMatrixFromQuaternion(float *q, arm_matrix_instance_f32 *mat, direct_cosine_matrix_t dcm_type);
void QuaternionFromRotationMatrix(arm_matrix_instance_f32 *mat, float *q);
void DeulerMatrixFromEuler(float phi, float theta, arm_matrix_instance_f32 *mat);
void Vec3_BodyToWorld(float *vec3_body, arm_matrix_instance_f32 *mat_rot, float *vec3_world);

void EKFInitQuaternion(float *q, float *a_vec, float *m_vec);
void EKFPredictQuaternionSV(float *q, float *gyr_vec, const float dt);
void EKFPredictQuaternionCM(float *q, const float dt, const float gyro_var, arm_matrix_instance_f32 *F_mat, arm_matrix_instance_f32 *P_mat);
void EKFCorrectQuaternionSV(float *q, float *acc_vec, float *mag_vec);
void EKFGetStateTransitionJacobian(float *gyro_vec, float dt, arm_matrix_instance_f32 *F_mat);
void EKFGetObservationJacobian(float *q, arm_matrix_instance_f32 *H_mat);

void KalmanFilterInit(Kalman_Instance *Kalman, uint8_t x_vec_size, uint8_t z_vec_size, uint8_t u_vec_size);
void KalmanFilterPredictSV(Kalman_Instance *Kalman, arm_matrix_instance_f32 *F_mat, float *x_vec, arm_matrix_instance_f32 *B_mat, float *u_vec);
void KalmanFilterPredictCM(Kalman_Instance *Kalman, const arm_matrix_instance_f32 *F_mat, arm_matrix_instance_f32 *P_mat, const arm_matrix_instance_f32 *Q_mat);
void KalmanFilterUpdateGain(Kalman_Instance *Kalman, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *H_mat, arm_matrix_instance_f32 *R_mat, arm_matrix_instance_f32 *K_mat);
void KalmanFilterCorrectSV(Kalman_Instance *Kalman, arm_matrix_instance_f32 *K_mat, float *z_vec, arm_matrix_instance_f32 *H_mat, float *x_vec);
void KalmanFilterCorrectCM(Kalman_Instance *Kalman, arm_matrix_instance_f32 *K_mat, arm_matrix_instance_f32 *H_mat, arm_matrix_instance_f32 *P_mat);

    /*float H_data[6*4] = { original observation model jacobian
        2*(g_vec_enu[0]*q[0]+g_vec_enu[1]*q[3]-g_vec_enu[2]*q[2]), 2*(g_vec_enu[0]*q[1]+g_vec_enu[1]*q[2]+g_vec_enu[2]*q[3]), 2*(-g_vec_enu[0]*q[2]+g_vec_enu[1]*q[1]-g_vec_enu[2]*q[0]), 2*(-g_vec_enu[0]*q[3]+g_vec_enu[1]*q[0]+g_vec_enu[2]*q[1]),
        2*(-g_vec_enu[0]*q[3]+g_vec_enu[1]*q[0]+g_vec_enu[2]*q[1]), 2*(g_vec_enu[0]*q[2]-g_vec_enu[1]*q[1]+g_vec_enu[2]*q[0]), 2*(g_vec_enu[0]*q[1]+g_vec_enu[1]*q[2]+g_vec_enu[2]*q[3]), 2*(-g_vec_enu[0]*q[0]-g_vec_enu[1]*q[3]+g_vec_enu[2]*q[2]),
        2*(g_vec_enu[0]*q[2]-g_vec_enu[1]*q[1]+g_vec_enu[2]*q[0]), 2*(g_vec_enu[0]*q[3]-g_vec_enu[1]*q[0]-g_vec_enu[2]*q[1]), 2*(g_vec_enu[0]*q[0]+g_vec_enu[1]*q[3]-g_vec_enu[2]*q[2]), 2*(g_vec_enu[0]*q[1]+g_vec_enu[1]*q[2]+g_vec_enu[2]*q[3]),
        2*(m_vec_enu[0]*q[0]+m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]), 2*(m_vec_enu[0]*q[1]+m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]), 2*(-m_vec_enu[0]*q[2]+m_vec_enu[1]*q[1]-m_vec_enu[2]*q[0]), 2*(-m_vec_enu[0]*q[3]+m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]),
        2*(-m_vec_enu[0]*q[3]+m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]), 2*(m_vec_enu[0]*q[2]-m_vec_enu[1]*q[1]+m_vec_enu[2]*q[0]), 2*(m_vec_enu[0]*q[1]+m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]), 2*(-m_vec_enu[0]*q[0]-m_vec_enu[1]*q[3]+m_vec_enu[2]*q[2]),
        2*(m_vec_enu[0]*q[2]-m_vec_enu[1]*q[1]+m_vec_enu[2]*q[0]), 2*(m_vec_enu[0]*q[3]-m_vec_enu[1]*q[0]-m_vec_enu[2]*q[1]), 2*(m_vec_enu[0]*q[0]+m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]), 2*(m_vec_enu[0]*q[1]+m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3])       
    };*/

#endif // NAVIGATION_H