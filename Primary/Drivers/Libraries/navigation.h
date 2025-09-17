#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "main.h"
#include "stdbool.h"
#include "armMathAddon.h"

// Quaternion EKF Settings
#define magnetic_dip_angle 66.0f

#define GYRO_VAR 0.3*0.3
#define BIAS_VAR 1e-12

// Height EKF Settings
#define ACCEL_VAR   0.5*0.5
#define BIAS_VAR    1e-12
#define BARO_VAR    0.2*0.2

// EKF sizes
#define x_size1 6
#define z_size1 3
#define u_size1 3

#define x_size2 2
#define z_size2 1
#define u_size2 1

#define x_size3 7
#define z_size3 6
#define u_size3 3 // NO B MATRIX THOUGH

// DCM rotation type
typedef enum {
    DCM_bi_WorldToBody = -1,
    DCM_ib_BodyToWorld = 1
} direct_cosine_matrix_t;

typedef enum {
    EKF1_type,
    EKF2_type,
    EKF3_type
} kalman_instance_t;

// standard KF variables
typedef struct {
    kalman_instance_t type;
    uint8_t x_size;
    uint8_t z_size;
    uint8_t u_size;
    float dt;
    float *h;
    float *u;
    float *v;
    float *x;
    float *z;
    arm_matrix_instance_f32 *B;
    arm_matrix_instance_f32 *F;
    arm_matrix_instance_f32 *H;
    arm_matrix_instance_f32 *K;
    arm_matrix_instance_f32 *P;
    arm_matrix_instance_f32 *Q;
    arm_matrix_instance_f32 *R;
    arm_matrix_instance_f32 *S;
} kalman_data_t;

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

void BaroPressureToHeight(float pressure, float *height);
void BaroHeightToPressure(float height, float *pressure);

void EKFInit(kalman_data_t *Kalman, kalman_instance_t kalman_type, uint8_t x_vec_size, uint8_t z_vec_size, uint8_t u_vec_size, const float dt,
                      arm_matrix_instance_f32 *F_mat, arm_matrix_instance_f32 *H_mat, arm_matrix_instance_f32 *K_mat, arm_matrix_instance_f32 *P_mat, 
                      arm_matrix_instance_f32 *Q_mat, arm_matrix_instance_f32 *R_mat, arm_matrix_instance_f32 *S_mat, arm_matrix_instance_f32 *B_mat,
                      float *x_vec, float *z_vec, float *h_vec, float *u_vec, float *v_vec);
void EKFStateVInit(kalman_data_t *Kalman);
void EKFPredictStateV(kalman_data_t *Kalman);
void EKFPredictCovariance(kalman_data_t *Kalman);
void EKFPredictMeasurement(kalman_data_t *Kalman);
void EKFGetInnovation(kalman_data_t *Kalman);
void EKFUpdateKalmanGain(kalman_data_t *Kalman);
void EKFCorrectStateV(kalman_data_t *Kalman);
void EKFCorrectCovariance(kalman_data_t *Kalman);

void EKFPredictionStep(kalman_data_t *Kalman);
void EKFCorrectionStep(kalman_data_t *Kalman, uint8_t correction_variant);

void EKFGetStateTransitionJacobian(kalman_data_t *Kalman);
void EKFGetObservationJacobian(kalman_data_t *Kalman);

    /*float H_data[6*4] = { original observation model jacobian
        2*(g_vec_enu[0]*q[0]+g_vec_enu[1]*q[3]-g_vec_enu[2]*q[2]), 2*(g_vec_enu[0]*q[1]+g_vec_enu[1]*q[2]+g_vec_enu[2]*q[3]), 2*(-g_vec_enu[0]*q[2]+g_vec_enu[1]*q[1]-g_vec_enu[2]*q[0]), 2*(-g_vec_enu[0]*q[3]+g_vec_enu[1]*q[0]+g_vec_enu[2]*q[1]),
        2*(-g_vec_enu[0]*q[3]+g_vec_enu[1]*q[0]+g_vec_enu[2]*q[1]), 2*(g_vec_enu[0]*q[2]-g_vec_enu[1]*q[1]+g_vec_enu[2]*q[0]), 2*(g_vec_enu[0]*q[1]+g_vec_enu[1]*q[2]+g_vec_enu[2]*q[3]), 2*(-g_vec_enu[0]*q[0]-g_vec_enu[1]*q[3]+g_vec_enu[2]*q[2]),
        2*(g_vec_enu[0]*q[2]-g_vec_enu[1]*q[1]+g_vec_enu[2]*q[0]), 2*(g_vec_enu[0]*q[3]-g_vec_enu[1]*q[0]-g_vec_enu[2]*q[1]), 2*(g_vec_enu[0]*q[0]+g_vec_enu[1]*q[3]-g_vec_enu[2]*q[2]), 2*(g_vec_enu[0]*q[1]+g_vec_enu[1]*q[2]+g_vec_enu[2]*q[3]),
        2*(m_vec_enu[0]*q[0]+m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]), 2*(m_vec_enu[0]*q[1]+m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]), 2*(-m_vec_enu[0]*q[2]+m_vec_enu[1]*q[1]-m_vec_enu[2]*q[0]), 2*(-m_vec_enu[0]*q[3]+m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]),
        2*(-m_vec_enu[0]*q[3]+m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]), 2*(m_vec_enu[0]*q[2]-m_vec_enu[1]*q[1]+m_vec_enu[2]*q[0]), 2*(m_vec_enu[0]*q[1]+m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]), 2*(-m_vec_enu[0]*q[0]-m_vec_enu[1]*q[3]+m_vec_enu[2]*q[2]),
        2*(m_vec_enu[0]*q[2]-m_vec_enu[1]*q[1]+m_vec_enu[2]*q[0]), 2*(m_vec_enu[0]*q[3]-m_vec_enu[1]*q[0]-m_vec_enu[2]*q[1]), 2*(m_vec_enu[0]*q[0]+m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]), 2*(m_vec_enu[0]*q[1]+m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3])       
    };*/

#endif // NAVIGATION_H