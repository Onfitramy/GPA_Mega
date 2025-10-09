#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "main.h"
#include "stdbool.h"
#include "armMathAddon.h"

/* --- GNSS delay compensation settings --- */
#define GNSS_VELOCITY_DELAY 350 // ms
#define GNSS_POSITION_DELAY 200 // ms | needs to be >= 50 in order for liftoff detection to work


/* --- Quaternion EKF Settings --- */
#define magnetic_dip_angle 66.0f

#define GYRO_VAR 0.3*0.3
#define GYRO_BIAS_VAR 1e-12


/* --- Height EKF Settings --- */
#define ACCEL_VAR   0.5*0.5
#define BARO_VAR    2*2
#define REFERENCE_PRESSURE_VAR 1e-6


/* --- Event detection Settings --- */
#define LIFTOFF_ACC_BUFFER_SIZE 50
#define LIFTOFF_MIN_OVERSHOOTS  30
#define LIFTOFF_ACC_THRESHOLD   10  // m/s²


// EKF sizes
#define x_size2 3
#define u_size2 1
#define z_size2_corr1 1
#define z_size2_corr2 2

#define x_size3 7
#define u_size3 3 // NO B MATRIX THOUGH
#define z_size3_corr1 6

// DCM rotation type
typedef enum {
    DCM_bi_WorldToBody = -1,
    DCM_ib_BodyToWorld = 1
} direct_cosine_matrix_t;

typedef enum {
    EKF1_type,
    EKF2_type,
    EKF3_type
} ekf_instance_t;

typedef enum {
    corr1_type,
    corr2_type
} ekf_correction_t;

typedef enum {
    inactive = 0,
    active = 1
} state_t;

// standard EKF variables
typedef struct {
    ekf_instance_t type;

    state_t prediction_state;

    uint8_t x_size;
    uint8_t u_size;

    float dt;
    float *u;
    float *x;
    
    arm_matrix_instance_f32 *B;
    arm_matrix_instance_f32 *F;
    arm_matrix_instance_f32 *P;
    arm_matrix_instance_f32 *Q;
} ekf_data_t;

// correction step EKF variables
typedef struct {
    ekf_correction_t type;

    state_t correction_state;

    uint8_t z_size;

    float *h;
    float *v;
    float *z;

    arm_matrix_instance_f32 *H;
    arm_matrix_instance_f32 *K;
    arm_matrix_instance_f32 *R;
    arm_matrix_instance_f32 *S;
    arm_matrix_instance_f32 *S_inv;
} ekf_corr_data_t;

/* EKF variables, vectors and matrices */
extern const float dt;

extern ekf_data_t EKF2;
extern float x2[x_size2];
extern arm_matrix_instance_f32 F2;
extern arm_matrix_instance_f32 Q2;
extern arm_matrix_instance_f32 P2;
extern float P2_data[x_size2*x_size2];

extern ekf_corr_data_t EKF2_corr1;
extern float z2_corr1[z_size2_corr1];
extern float h2_corr1[z_size2_corr1];
extern float v2_corr1[z_size2_corr1];
extern arm_matrix_instance_f32 H2_corr1;
extern arm_matrix_instance_f32 R2_corr1;
extern arm_matrix_instance_f32 S2_corr1;
extern arm_matrix_instance_f32 S2_inv_corr1;
extern arm_matrix_instance_f32 K2_corr1;

extern ekf_corr_data_t EKF2_corr2;
extern float z2_corr2[z_size2_corr2];
extern float h2_corr2[z_size2_corr2];
extern float v2_corr2[z_size2_corr2];
extern arm_matrix_instance_f32 H2_corr2;
extern arm_matrix_instance_f32 R2_corr2;
extern arm_matrix_instance_f32 S2_corr2;
extern arm_matrix_instance_f32 S2_inv_corr2;
extern arm_matrix_instance_f32 K2_corr2;
extern float R2_corr2_data[z_size2_corr2*z_size2_corr2];

extern float NIS_EKF2_corr1;
extern float NIS_EKF2_corr2;

extern ekf_data_t EKF3;
extern float x3[x_size3];
extern arm_matrix_instance_f32 F3;
extern arm_matrix_instance_f32 Q3;
extern arm_matrix_instance_f32 P3;
extern float P3_data[x_size3*x_size3];

extern ekf_corr_data_t EKF3_corr1;
extern float z3_corr1[z_size3_corr1];
extern float h3_corr1[z_size3_corr1];
extern float v3_corr1[z_size3_corr1];
extern arm_matrix_instance_f32 H3_corr1;
extern arm_matrix_instance_f32 R3_corr1;
extern arm_matrix_instance_f32 S3_corr1;
extern arm_matrix_instance_f32 S3_inv_corr1;
extern arm_matrix_instance_f32 K3_corr1;

extern float NIS_EKF3_corr1;

extern double WGS84[3];
extern double WGS84_ref[3];
extern double ECEF[3];
extern double ECEF_ref[3];
extern double ENU[3];

extern arm_matrix_instance_f32 M_rot_bi;
extern arm_matrix_instance_f32 M_rot_ib;
extern arm_matrix_instance_f32 M_rot_q;

extern float euler_from_q[3];

extern float a_WorldFrame[3];
extern float a_BodyFrame[3];
extern float a_abs;
extern float gravity_world_vec[3];
extern float gravity_body_vec[3];

extern float corr_acc_buf[GNSS_VELOCITY_DELAY];
extern float corr_acc_sum;
extern uint16_t corr_acc_index;
extern float corr_delta_v;

extern float corr_vel_buf[GNSS_POSITION_DELAY];
extern float corr_vel_sum;
extern uint16_t corr_vel_index;
extern float corr_delta_h;

extern float gnss_height_corr;
extern float gnss_velZ_corr;

extern float acc_z_buf[LIFTOFF_ACC_BUFFER_SIZE];
extern uint16_t acc_z_index;

/* --- Function declarations --- */
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

void CompensateGNSSDelay(float acc_meas, float vel_meas, float *v_corr_val, float *h_corr_val);
void BaroPressureToHeight(float pressure, float pressure_reference, float *height);
void BaroHeightToPressure(float height, float pressure_reference, float *pressure);

void EKFInit(ekf_data_t *ekf, ekf_instance_t kalman_type, uint8_t x_vec_size, uint8_t u_vec_size, const float dt,
             arm_matrix_instance_f32 *F_mat, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *Q_mat,
             arm_matrix_instance_f32 *B_mat, float *x_vec, float *u_vec);
void EKFCorrectionInit(ekf_data_t ekf, ekf_corr_data_t *ekf_corr, ekf_correction_t corr_type, uint8_t z_vec_size,
                       arm_matrix_instance_f32 *H_mat, arm_matrix_instance_f32 *K_mat, arm_matrix_instance_f32 *R_mat,
                       arm_matrix_instance_f32 *S_mat, arm_matrix_instance_f32 *S_inv_mat, float *z_vec, float *h_vec, float *v_vec);
void EKFStateVInit(ekf_data_t *Kalman, ekf_corr_data_t *ekf_corr);
void EKFPredictStateV(ekf_data_t *Kalman);
void EKFPredictCovariance(ekf_data_t *Kalman);
void EKFPredictMeasurement(ekf_data_t *Kalman, ekf_corr_data_t *ekf_corr);
void EKFGetInnovation(ekf_data_t *Kalman, ekf_corr_data_t *ekf_corr);
void EKFUpdateKalmanGain(ekf_data_t *Kalman, ekf_corr_data_t *ekf_corr);
void EKFCorrectStateV(ekf_data_t *Kalman, ekf_corr_data_t *ekf_corr);
void EKFCorrectCovariance(ekf_data_t *Kalman, ekf_corr_data_t *ekf_corr);

void EKFGetStateTransitionJacobian(ekf_data_t *Kalman);
void EKFGetObservationJacobian(ekf_data_t *Kalman, ekf_corr_data_t *ekf_corr);

void EKFPredictionStep(ekf_data_t *Kalman);
void EKFCorrectionStep(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr);

void EKFgetNIS(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr, float *NIS);

    /*float H_data[6*4] = { original observation model jacobian
        2*(g_vec_enu[0]*q[0]+g_vec_enu[1]*q[3]-g_vec_enu[2]*q[2]), 2*(g_vec_enu[0]*q[1]+g_vec_enu[1]*q[2]+g_vec_enu[2]*q[3]), 2*(-g_vec_enu[0]*q[2]+g_vec_enu[1]*q[1]-g_vec_enu[2]*q[0]), 2*(-g_vec_enu[0]*q[3]+g_vec_enu[1]*q[0]+g_vec_enu[2]*q[1]),
        2*(-g_vec_enu[0]*q[3]+g_vec_enu[1]*q[0]+g_vec_enu[2]*q[1]), 2*(g_vec_enu[0]*q[2]-g_vec_enu[1]*q[1]+g_vec_enu[2]*q[0]), 2*(g_vec_enu[0]*q[1]+g_vec_enu[1]*q[2]+g_vec_enu[2]*q[3]), 2*(-g_vec_enu[0]*q[0]-g_vec_enu[1]*q[3]+g_vec_enu[2]*q[2]),
        2*(g_vec_enu[0]*q[2]-g_vec_enu[1]*q[1]+g_vec_enu[2]*q[0]), 2*(g_vec_enu[0]*q[3]-g_vec_enu[1]*q[0]-g_vec_enu[2]*q[1]), 2*(g_vec_enu[0]*q[0]+g_vec_enu[1]*q[3]-g_vec_enu[2]*q[2]), 2*(g_vec_enu[0]*q[1]+g_vec_enu[1]*q[2]+g_vec_enu[2]*q[3]),
        2*(m_vec_enu[0]*q[0]+m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]), 2*(m_vec_enu[0]*q[1]+m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]), 2*(-m_vec_enu[0]*q[2]+m_vec_enu[1]*q[1]-m_vec_enu[2]*q[0]), 2*(-m_vec_enu[0]*q[3]+m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]),
        2*(-m_vec_enu[0]*q[3]+m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]), 2*(m_vec_enu[0]*q[2]-m_vec_enu[1]*q[1]+m_vec_enu[2]*q[0]), 2*(m_vec_enu[0]*q[1]+m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]), 2*(-m_vec_enu[0]*q[0]-m_vec_enu[1]*q[3]+m_vec_enu[2]*q[2]),
        2*(m_vec_enu[0]*q[2]-m_vec_enu[1]*q[1]+m_vec_enu[2]*q[0]), 2*(m_vec_enu[0]*q[3]-m_vec_enu[1]*q[0]-m_vec_enu[2]*q[1]), 2*(m_vec_enu[0]*q[0]+m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]), 2*(m_vec_enu[0]*q[1]+m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3])       
    };*/

#endif // NAVIGATION_H