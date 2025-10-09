#include "navigation.h"

/* ------------------------------------------- CONSTANTS ------------------------------------------- */

// define constants of WGS84
const double a = 6378137.;
const double f = 1. / 298.257223563;
const double e2 = 0.006694379990141317;

// define atmospheric constants
const double p0_const = 101325.;    // Pa
const double R_const = 287.05;      // J/(kgK)
const double T0_const = 288.15;     // K
const double L_const = -0.0065;     // K/m
const double g0_const = 9.80665;    // m/s²

// time step
const float dt = 0.001;

/* ------------------------------------------- VARIABLES ------------------------------------------- */

/* --- EKF vectors and matrices --- */

// Vertical Position EKF variables
ekf_data_t EKF2;
float x2[x_size2] = {0};
float F2_data[x_size2*x_size2] = {0};
arm_matrix_instance_f32 F2 = {x_size2, x_size2, F2_data};
float Q2_data[x_size2*x_size2] = {0};
arm_matrix_instance_f32 Q2 = {x_size2, x_size2, Q2_data};
float P2_data[x_size2*x_size2] = {0};
arm_matrix_instance_f32 P2 = {x_size2, x_size2, P2_data};

ekf_corr_data_t EKF2_corr1;
float z2_corr1[z_size2_corr1] = {0};
float h2_corr1[z_size2_corr1] = {0};
float v2_corr1[z_size2_corr1] = {0};
float H2_corr1_data[z_size2_corr1*x_size2] = {0};
arm_matrix_instance_f32 H2_corr1 = {z_size2_corr1, x_size2, H2_corr1_data};
float R2_corr1_data[z_size2_corr1*z_size2_corr1] = {0};
arm_matrix_instance_f32 R2_corr1 = {z_size2_corr1, z_size2_corr1, R2_corr1_data};
float S2_corr1_data[z_size2_corr1*z_size2_corr1] = {0};
arm_matrix_instance_f32 S2_corr1 = {z_size2_corr1, z_size2_corr1, S2_corr1_data};
float S2_inv_corr1_data[z_size2_corr1*z_size2_corr1] = {0};
arm_matrix_instance_f32 S2_inv_corr1 = {z_size2_corr1, z_size2_corr1, S2_inv_corr1_data};
float K2_corr1_data[x_size2*z_size2_corr1];
arm_matrix_instance_f32 K2_corr1 = {x_size2, z_size2_corr1, K2_corr1_data};

ekf_corr_data_t EKF2_corr2;
float z2_corr2[z_size2_corr2] = {0};
float h2_corr2[z_size2_corr2] = {0};
float v2_corr2[z_size2_corr2] = {0};
float H2_corr2_data[z_size2_corr2*x_size2] = {0};
arm_matrix_instance_f32 H2_corr2 = {z_size2_corr2, x_size2, H2_corr2_data};
float R2_corr2_data[z_size2_corr2*z_size2_corr2] = {0};
arm_matrix_instance_f32 R2_corr2 = {z_size2_corr2, z_size2_corr2, R2_corr2_data};
float S2_corr2_data[z_size2_corr2*z_size2_corr2] = {0};
arm_matrix_instance_f32 S2_corr2 = {z_size2_corr2, z_size2_corr2, S2_corr2_data};
float S2_inv_corr2_data[z_size2_corr2*z_size2_corr2] = {0};
arm_matrix_instance_f32 S2_inv_corr2 = {z_size2_corr2, z_size2_corr2, S2_inv_corr2_data};
float K2_corr2_data[x_size2*z_size2_corr2];
arm_matrix_instance_f32 K2_corr2 = {x_size2, z_size2_corr2, K2_corr2_data};

float NIS_EKF2_corr1;
float NIS_EKF2_corr2;

// Quaternion EKF variables
ekf_data_t EKF3;
float x3[x_size3] = {0};
float F3_data[x_size3*x_size3] = {0}; // 7x7
arm_matrix_instance_f32 F3 = {x_size3, x_size3, F3_data};
float Q3_data[x_size3*x_size3] = {0}; // 7x7
arm_matrix_instance_f32 Q3 = {x_size3, x_size3, Q3_data};
float P3_data[x_size3*x_size3] = {0}; // 7x7
arm_matrix_instance_f32 P3 = {x_size3, x_size3, P3_data};

ekf_corr_data_t EKF3_corr1;
float z3_corr1[z_size3_corr1] = {0};
float h3_corr1[z_size3_corr1] = {0};
float v3_corr1[z_size3_corr1] = {0};
float H3_corr1_data[z_size3_corr1*x_size3] = {0};
arm_matrix_instance_f32 H3_corr1 = {z_size3_corr1, x_size3, H3_corr1_data};
float R3_corr1_data[z_size3_corr1*z_size3_corr1] = {0};
arm_matrix_instance_f32 R3_corr1 = {z_size3_corr1, z_size3_corr1, R3_corr1_data};
float S3_corr1_data[z_size3_corr1*z_size3_corr1] = {0};
arm_matrix_instance_f32 S3_corr1 = {z_size3_corr1, z_size3_corr1, S3_corr1_data};
float S3_inv_corr1_data[z_size3_corr1*z_size3_corr1] = {0};
arm_matrix_instance_f32 S3_inv_corr1 = {z_size3_corr1, z_size3_corr1, S3_inv_corr1_data};
float K3_corr1_data[x_size3*z_size3_corr1];
arm_matrix_instance_f32 K3_corr1 = {x_size3, z_size3_corr1, K3_corr1_data};

float NIS_EKF3_corr1;


/* --- Coordinate system variables --- */

// reference frames
double WGS84[3];
double WGS84_ref[3];
double ECEF[3];
double ECEF_ref[3];
double ENU[3];

// rotation matrix
float M_rot_bi_data[9];
arm_matrix_instance_f32 M_rot_bi = {3, 3, M_rot_bi_data};
float M_rot_ib_data[9];
arm_matrix_instance_f32 M_rot_ib = {3, 3, M_rot_ib_data};
float M_rot_q_data[9];
arm_matrix_instance_f32 M_rot_q = {3, 3, M_rot_q_data};

float euler_from_q[3] = {0};

// vectors
float a_WorldFrame[3] = {0}; // Acceleration
float a_BodyFrame[3] = {0};
float a_abs;
float gravity_world_vec[3] = {0, 0, 9.8};
float gravity_body_vec[3];

// GNSS delay compensation
float corr_acc_buf[GNSS_VELOCITY_DELAY] = {0};
float corr_acc_sum = 0;
uint16_t corr_acc_index = 0;
float corr_delta_v = 0;

float corr_vel_buf[GNSS_POSITION_DELAY] = {0};
float corr_vel_sum = 0;
uint16_t corr_vel_index = 0;
float corr_delta_h = 0;

float gnss_height_corr;
float gnss_velZ_corr;

// Liftoff detection
float acc_z_buf[LIFTOFF_ACC_BUFFER_SIZE] = {0};
uint16_t acc_z_index = 0;

/* ------------------------------------------- FUNCTIONS ------------------------------------------- */

// set boundaries for angle
void normalizeAngle(float *angle, float upper_boundary, float lower_boundary, float step_size) {
    if (*angle > upper_boundary) {
        *angle -= step_size;
    } 
    if (*angle < lower_boundary) {
        *angle += step_size;
    }
}

// set boundaries for angles
void normalizeAngleVector(float *angle_vec, uint8_t pos_top, uint8_t pos_bottom, float upper_boundary, float lower_boundary, float step_size) {
    for (int i = pos_top; i <= pos_bottom; i++) {
        if ( angle_vec[i] > upper_boundary) {
            angle_vec[i] -= step_size;
        } 
        if ( angle_vec[i] < lower_boundary) {
            angle_vec[i] += step_size;
        }
    }
}

// make mod angle keep normalized difference towards lead angle
void normalizeAnglePair(float angle_lead, float *angle_mod, float upper_boundary, float lower_boundary, float step_size) {
    *angle_mod -= angle_lead;
    normalizeAngle(angle_mod, upper_boundary, lower_boundary, step_size);
    *angle_mod += angle_lead;
}

// make mod angles keep normalized difference towards lead angles
void normalizeAnglePairVector(float *angle_lead_vec, float *angle_mod_vec, uint8_t pos_top, uint8_t pos_bottom, float upper_boundary, float lower_boundary, float step_size) {
    for (int i = pos_top; i <= pos_bottom; i++) {
        angle_mod_vec[i] -= angle_lead_vec[i];
        normalizeAngle(&angle_mod_vec[i], upper_boundary, lower_boundary, step_size);
        angle_mod_vec[i] += angle_lead_vec[i];
   } 
}

// convert ublox outputs to metric
void UBLOXtoWGS84(int32_t lat_e7, int32_t lon_e7, int32_t height_e3, double *WGS84) {
    WGS84[0] = (double)lat_e7 * 1e-7;
    WGS84[1] = (double)lon_e7 * 1e-7;
    WGS84[2] = (double)height_e3 * 1e-3;
}

// convert WGS84 coordinates to ECEF (Earth Centered Earth Fixed) coordinates
void WGS84toECEF(double *WGS84, double *ECEF) {
    // convert angles to radians
    double lat_rad = WGS84[0] * PI / 180.;
    double lon_rad = WGS84[1] * PI / 180.;

    // transform WGS84 to ECEF
    double N = a / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));
    ECEF[0] = (N + WGS84[2]) * cos(lat_rad) * cos(lon_rad);
    ECEF[1] = (N + WGS84[2]) * cos(lat_rad) * sin(lon_rad);
    ECEF[2] = (N * (1 - e2) + WGS84[2]) * sin(lat_rad);
}

// convert ECEF (Earth Centered Earth Fixed) coordinates to ENU (East North Up) coordinates
void ECEFtoENU(double *WGS84_ref, double *ECEF_ref, double *ECEF, double *ENU) {
    // calculate differences in ECEF frame
    double dx = ECEF[0] - ECEF_ref[0];
    double dy = ECEF[1] - ECEF_ref[1];
    double vt = ECEF[2] - ECEF_ref[2];

    // convert reference point angles to radians
    double lat_rad = WGS84_ref[0] * PI / 180.;
    double lon_rad = WGS84_ref[1] * PI / 180.;

    // transform difference to ENU frame
    ENU[0] = -sin(lon_rad) * dx + cos(lon_rad) * dy;                                                    // East
    ENU[1] = -sin(lat_rad) * cos(lon_rad) * dx - sin(lat_rad) * sin(lon_rad) * dy + cos(lat_rad) * vt;  // North
    ENU[2] =  cos(lat_rad) * cos(lon_rad) * dx + cos(lat_rad) * sin(lon_rad) * dy + sin(lat_rad) * vt;  // Up
}

// attitude estimation using magnetometer and accelerometer
void EulerOrientationFix(float *a_vec, float *m_vec, float *output_angles) {
    // unit basis vectors of inertial system expressed in body coordinates
    float base_xi[3];
    float base_yi[3];
    float base_zi[3];

    // normalize a and m vectors
    arm_vec3_copy_f32(m_vec, base_yi);
    arm_vec3_copy_f32(a_vec, base_zi);
    arm_vec3_normalize_f32(base_zi);

    // calculate unit basis vectors
    arm_vec3_cross_product_f32(base_yi, base_zi, base_xi);
    arm_vec3_normalize_f32(base_xi);
    arm_vec3_cross_product_f32(base_zi, base_xi, base_yi);

    // calculate fix angles
    output_angles[0] = atan2(base_zi[1], base_zi[2]);
    output_angles[1] = asin(-base_zi[0]);
    output_angles[2] = atan2(base_yi[0], base_xi[0]);
}

// attitude estimation using magnetometer and accelerometer | World To Body, DCMbi
void RotationMatrixOrientationFix(float *a_vec, float *m_vec, arm_matrix_instance_f32 *mat, direct_cosine_matrix_t dcm_type) {
    // unit basis vectors of inertial system expressed in body coordinates
    float base_xi[3];
    float base_yi[3];
    float base_zi[3];

    // normalize a and m vectors
    arm_vec3_copy_f32(m_vec, base_yi);
    arm_vec3_copy_f32(a_vec, base_zi);
    arm_vec3_normalize_f32(base_zi);

    // calculate unit basis vectors
    arm_vec3_cross_product_f32(base_yi, base_zi, base_xi);
    arm_vec3_normalize_f32(base_xi);
    arm_vec3_cross_product_f32(base_zi, base_xi, base_yi);

    // set rotation matrix
    if (dcm_type == DCM_bi_WorldToBody) {
        arm_mat_set_column_f32(mat, 0, base_xi);
        arm_mat_set_column_f32(mat, 1, base_yi);
        arm_mat_set_column_f32(mat, 2, base_zi);
    } else if(dcm_type == DCM_ib_BodyToWorld) {
        arm_mat_set_row_f32(mat, 0, base_xi);
        arm_mat_set_row_f32(mat, 1, base_zi);
        arm_mat_set_row_f32(mat, 2, base_yi);
    }
}

// create rotation matrix from euler angles | World To Body, DCMbi
void RotationMatrixFromEuler(float phi, float theta, float psi, arm_matrix_instance_f32 *mat) {
    float sin_vec[3]; // vector containing sines
    float cos_vec[3]; // vector containing cosines
    float c1[3], c2[3], c3[3];
    
    sin_vec[0] = sin(phi);
    sin_vec[1] = sin(theta);
    sin_vec[2] = sin(psi);
    cos_vec[0] = cos(phi);
    cos_vec[1] = cos(theta);
    cos_vec[2] = cos(psi);

    c2[0] = sin_vec[2] * cos_vec[1];
    c2[1] = sin_vec[2] * sin_vec[1] * sin_vec[0] + cos_vec[2] * cos_vec[0];
    c2[2] = sin_vec[2] * sin_vec[1] * cos_vec[0] - cos_vec[2] * sin_vec[0];

    c3[0] = -sin_vec[1];
    c3[1] = cos_vec[1] * sin_vec[0];
    c3[2] = cos_vec[1] * cos_vec[0];

    arm_vec3_cross_product_f32(c2, c3, c1);

    arm_mat_set_column_f32(mat, 0, c1);
    arm_mat_set_column_f32(mat, 1, c2);
    arm_mat_set_column_f32(mat, 2, c3);
}

void EulerFromRotationMatrix(arm_matrix_instance_f32 *mat, float *euler) {
    euler[0] = atan2(arm_mat_get_entry_f32(mat, 1, 2), arm_mat_get_entry_f32(mat, 2, 2));
    euler[1] = asin(-arm_mat_get_entry_f32(mat, 0, 2));
    euler[2] = atan2(arm_mat_get_entry_f32(mat, 0, 1), arm_mat_get_entry_f32(mat, 0, 0));
}

// create rotation matrix from quaternion
void RotationMatrixFromQuaternion(float *q, arm_matrix_instance_f32 *mat, direct_cosine_matrix_t dcm_type) {
    arm_mat_set_entry_f32(mat, 0, 0, 1 - 2*(q[2]*q[2] + q[3]*q[3]));
    arm_mat_set_entry_f32(mat, 0, 1, 2*(q[1]*q[2] - q[0]*q[3]*dcm_type));
    arm_mat_set_entry_f32(mat, 0, 2, 2*(q[1]*q[3] + q[0]*q[2]*dcm_type));
    arm_mat_set_entry_f32(mat, 1, 0, 2*(q[1]*q[2] + q[0]*q[3]*dcm_type));
    arm_mat_set_entry_f32(mat, 1, 1, 1 - 2*(q[1]*q[1] + q[3]*q[3]));
    arm_mat_set_entry_f32(mat, 1, 2, 2*(q[2]*q[3] - q[0]*q[1]*dcm_type));
    arm_mat_set_entry_f32(mat, 2, 0, 2*(q[1]*q[3] - q[0]*q[2]*dcm_type));
    arm_mat_set_entry_f32(mat, 2, 1, 2*(q[2]*q[3] + q[0]*q[1]*dcm_type));
    arm_mat_set_entry_f32(mat, 2, 2, 1 - 2*(q[1]*q[1] + q[2]*q[2]));
}

// create quaternion from rotation matrix
void QuaternionFromRotationMatrix(arm_matrix_instance_f32 *mat, float *q) {
    float trace = arm_mat_get_entry_f32(mat, 0, 0) + arm_mat_get_entry_f32(mat, 1, 1) + arm_mat_get_entry_f32(mat, 2, 2);
    if (trace > 0) {
        float s = 2.0f * sqrtf(trace + 1.0f);
        q[0] = 0.25f * s;
        q[1] = (arm_mat_get_entry_f32(mat, 2, 1) - arm_mat_get_entry_f32(mat, 1, 2)) / s;
        q[2] = (arm_mat_get_entry_f32(mat, 0, 2) - arm_mat_get_entry_f32(mat, 2, 0)) / s;
        q[3] = (arm_mat_get_entry_f32(mat, 1, 0) - arm_mat_get_entry_f32(mat, 0, 1)) / s;
    } else if (arm_mat_get_entry_f32(mat, 0, 0) > arm_mat_get_entry_f32(mat, 1, 1) && arm_mat_get_entry_f32(mat, 0, 0) > arm_mat_get_entry_f32(mat, 2, 2)) {
        float s = 2.0f * sqrtf(1.0f + arm_mat_get_entry_f32(mat, 0, 0) - arm_mat_get_entry_f32(mat, 1, 1) - arm_mat_get_entry_f32(mat, 2, 2));
        q[0] = (arm_mat_get_entry_f32(mat, 2, 1) - arm_mat_get_entry_f32(mat, 1, 2)) / s;
        q[1] = 0.25f * s;
        q[2] = (arm_mat_get_entry_f32(mat, 0, 1) + arm_mat_get_entry_f32(mat, 1, 0)) / s;
        q[3] = (arm_mat_get_entry_f32(mat, 0, 2) + arm_mat_get_entry_f32(mat, 2, 0)) / s;
    } else if (arm_mat_get_entry_f32(mat, 1, 1) > arm_mat_get_entry_f32(mat, 2, 2)) {
        float s = 2.0f * sqrtf(1.0f + arm_mat_get_entry_f32(mat, 1, 1) - arm_mat_get_entry_f32(mat, 0, 0) - arm_mat_get_entry_f32(mat, 2, 2));
        q[0] = (arm_mat_get_entry_f32(mat, 0, 2) - arm_mat_get_entry_f32(mat, 2, 0)) / s;
        q[1] = (arm_mat_get_entry_f32(mat, 0, 1) + arm_mat_get_entry_f32(mat, 1, 0)) / s;
        q[2] = 0.25f * s;
        q[3] = (arm_mat_get_entry_f32(mat, 1, 2) + arm_mat_get_entry_f32(mat, 2, 1)) / s;
    } else {
        float s = 2.0f * sqrtf(1.0f + arm_mat_get_entry_f32(mat, 2, 2) - arm_mat_get_entry_f32(mat, 0, 0) - arm_mat_get_entry_f32(mat, 1, 1));
        q[0] = (arm_mat_get_entry_f32(mat, 1, 0) - arm_mat_get_entry_f32(mat, 0, 1)) / s;
        q[1] = (arm_mat_get_entry_f32(mat, 0, 2) + arm_mat_get_entry_f32(mat, 2, 0)) / s;
        q[2] = (arm_mat_get_entry_f32(mat, 1, 2) + arm_mat_get_entry_f32(mat, 2, 1)) / s;
        q[3] = 0.25f * s;
    }
    arm_quaternion_normalize_f32(q);
}

void DeulerMatrixFromEuler(float phi, float theta, arm_matrix_instance_f32 *mat) {
    float sin_vec[2]; // vector containing sines
    float cos_vec[2]; // vector containing cosines
    
    sin_vec[0] = sin(phi);
    sin_vec[1] = sin(theta);
    cos_vec[0] = cos(phi);
    cos_vec[1] = cos(theta);
    
    arm_mat_set_entry_f32(mat, 0, 0, 1);
    arm_mat_set_entry_f32(mat, 0, 1, sin_vec[0]*sin_vec[1]/cos_vec[1]);
    arm_mat_set_entry_f32(mat, 0, 2, cos_vec[0]*sin_vec[1]/cos_vec[1]);
    arm_mat_set_entry_f32(mat, 1, 0, 0);
    arm_mat_set_entry_f32(mat, 1, 1, cos_vec[0]);
    arm_mat_set_entry_f32(mat, 1, 2, -sin_vec[0]);
    arm_mat_set_entry_f32(mat, 2, 0, 0);
    arm_mat_set_entry_f32(mat, 2, 1, sin_vec[0]/cos_vec[1]);
    arm_mat_set_entry_f32(mat, 2, 2, cos_vec[0]/cos_vec[1]);
}



// compensate GNSS measurement delay
void CompensateGNSSDelay(float acc_meas, float vel_meas, float *v_corr_val, float *h_corr_val) {
    // update acceleration buffer and sum
    corr_acc_sum -= corr_acc_buf[corr_acc_index];
    corr_acc_buf[corr_acc_index] = acc_meas;
    corr_acc_sum += corr_acc_buf[corr_acc_index];
    corr_acc_index = (corr_acc_index + 1) % GNSS_VELOCITY_DELAY;

    // calculate velocity difference between gnss delay and present
    *v_corr_val = corr_acc_sum * dt;

    // update velocity buffer and sum
    corr_vel_sum -= corr_vel_buf[corr_vel_index] + 0.5 * dt * corr_acc_buf[corr_vel_index];
    corr_vel_buf[corr_vel_index] = vel_meas;
    corr_vel_sum += corr_vel_buf[corr_vel_index] + 0.5 * dt * corr_acc_buf[corr_vel_index];
    corr_vel_index = (corr_vel_index + 1) % GNSS_POSITION_DELAY;

    // calculate position difference between gnss delay and present
    *h_corr_val = corr_vel_sum * dt;
}

// calculate barometric height from static pressure measurement
void BaroPressureToHeight(float pressure, float pressure_reference, float *height) {
    double buffer;
    buffer = (pow(pressure / pressure_reference, -L_const * R_const / g0_const) - 1.) * T0_const / L_const;
    *height = (float)buffer;
}

// calculate barometric pressure from height
void BaroHeightToPressure(float height, float pressure_reference, float *pressure) {
    double buffer;
    buffer = pow(L_const / T0_const * height + 1., -g0_const / L_const / R_const) * pressure_reference;
    *pressure = (float)buffer;
}

// create new Kalman Filter instance
void EKFInit(ekf_data_t *ekf, ekf_instance_t kalman_type, uint8_t x_vec_size, uint8_t u_vec_size, const float dt,
             arm_matrix_instance_f32 *F_mat, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *Q_mat,
             arm_matrix_instance_f32 *B_mat, float *x_vec, float *u_vec) 
{
    // DEFAULT: prediction step is inactive
    ekf->prediction_state = inactive;

    // set type and sizes
    ekf->type = kalman_type;
    ekf->x_size = x_vec_size;
    ekf->u_size = u_vec_size;
    ekf->dt = dt;

    // set vector addresses
    ekf->x = x_vec;
    ekf->u = u_vec;

    // set matrix addresses
    ekf->B = B_mat;
    ekf->F = F_mat;
    ekf->P = P_mat;
    ekf->Q = Q_mat;

    if (kalman_type == EKF1_type) {
        // initialize EKF1 matrices

    } else if (kalman_type == EKF2_type) {
        // initialize EKF2 matrices
        // configure height EKF matrices
        arm_mat_fill_diag_f32(ekf->F, 0, 0, 1.);
        arm_mat_set_entry_f32(ekf->F, 0, 1, dt);

        arm_mat_set_entry_f32(ekf->P, 0, 0, 10.);       // initial guess for height variance
        arm_mat_set_entry_f32(ekf->P, 1, 1, 1.);        // initial guess for vertical velocity variance
        arm_mat_set_entry_f32(ekf->P, 2, 2, 100.);      // initial guess for reference pressure variance   

        arm_mat_set_entry_f32(ekf->Q, 0, 0, ACCEL_VAR*0.25*dt*dt*dt*dt);  // variance of accelerometer data   (noise)
        arm_mat_set_entry_f32(ekf->Q, 0, 1, ACCEL_VAR*0.5*dt*dt*dt);
        arm_mat_set_entry_f32(ekf->Q, 1, 0, ACCEL_VAR*0.5*dt*dt*dt);
        arm_mat_set_entry_f32(ekf->Q, 1, 1, ACCEL_VAR*dt*dt);
        arm_mat_set_entry_f32(ekf->Q, 2, 2, REFERENCE_PRESSURE_VAR);
        //arm_mat_set_diag_f32(ekf->Q, 6, 6, 3, 1e-12f);       // variance of accelerometer offset (drift)

    } else if (kalman_type == EKF3_type) {
        // initialize EKF3 matrices
        // configure Quaternion EKF matrices
        arm_mat_fill_diag_f32(ekf->F, 0, 0, 1.);

        arm_mat_set_diag_f32(ekf->P, 0, 0, 4, 1.);       // first guess of orientation
        arm_mat_set_diag_f32(ekf->P, 4, 4, 3, 1e-3);     // first guess of gyro bias

    }
}

// Create new EKF correction step instance
void EKFCorrectionInit(ekf_data_t ekf, ekf_corr_data_t *ekf_corr, ekf_correction_t corr_type, uint8_t z_vec_size,
                       arm_matrix_instance_f32 *H_mat, arm_matrix_instance_f32 *K_mat, arm_matrix_instance_f32 *R_mat,
                       arm_matrix_instance_f32 *S_mat, arm_matrix_instance_f32 *S_inv_mat, float *z_vec, float *h_vec, float *v_vec)
{
    // DEFAULT: correction step is inactive
    ekf_corr->correction_state = inactive;

    // set type and sizes
    ekf_corr->type = corr_type;
    ekf_corr->z_size = z_vec_size;
    
    // set vector addresses
    ekf_corr->z = z_vec;
    ekf_corr->h = h_vec;
    ekf_corr->v = v_vec;

    // set matrix addresses
    ekf_corr->H = H_mat;
    ekf_corr->K = K_mat;
    ekf_corr->R = R_mat;
    ekf_corr->S = S_mat;
    ekf_corr->S_inv = S_inv_mat;

    if (ekf.type == EKF1_type) {
        // configure EKF1 correction matrices

    } else if (ekf.type == EKF2_type) {
        // configure height EKF correction matrices

    } else if (ekf.type == EKF3_type) {
        // configure Quaternion EKF correction matrices
        arm_mat_set_diag_f32(ekf_corr->R, 0, 0, 3, 0.5*0.5);  // accelerometer variance (noise)
        arm_mat_set_diag_f32(ekf_corr->R, 3, 3, 3, 0.8*0.8);  // magnetometer variance (noise)

    }
}

void EKFStateVInit(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr) {
    if (ekf->type == EKF2_type) {
        ekf->x[0] = ekf_corr->z[0];

    } else if (ekf->type == EKF3_type) {
        // calculate initial quaternion from accelerometer and magnetometer readings
        float *q = ekf->x;

        float DCMbi_data[9] = {0};
        arm_matrix_instance_f32 DCMbi = {3, 3, DCMbi_data};

        RotationMatrixOrientationFix(&ekf_corr->z[0], &ekf_corr->z[3], &DCMbi, DCM_bi_WorldToBody);
        QuaternionFromRotationMatrix(&DCMbi, q); // does this function want body to world????
        arm_quaternion_conjugate_f32(q);         // quick fix
    }
}

void EKFPredictStateV(ekf_data_t *ekf) {
    if (ekf->type == EKF1_type) {
        // calculate predicted state x^(t) = F * x(t-1) + B * u(t)

    } else if (ekf->type == EKF2_type) {
        ekf->x[0] += ekf->x[1]*ekf->dt + 0.5*ekf->u[0]*ekf->dt*ekf->dt;
        ekf->x[1] += ekf->u[0]*ekf->dt;

    } else if (ekf->type == EKF3_type) {
        // prediction step for quaternion state vector
        float *q = ekf->x;

        float omega[4];
        omega[0] = 0;
        omega[1] = ekf->u[0] - ekf->x[4];
        omega[2] = ekf->u[1] - ekf->x[5];
        omega[3] = ekf->u[2] - ekf->x[6];

        float q_omega[4];
        arm_quaternion_product_f32(q, omega, q_omega);

        for (int i = 0; i < 4; i++) {
            q[i] = q[i] + 0.5f * q_omega[i] * ekf->dt;
        }
        arm_quaternion_normalize_f32(q);

    }
}


void EKFPredictCovariance(ekf_data_t *ekf) {
    // calculate Q separately for quaternion EKF
    if (ekf->type == EKF3_type) {
        float *q = ekf->x;

        float M1_data[4*4];
        arm_matrix_instance_f32 M1 = {4, 4, M1_data};
        // define W and W'
        float W_mat_data[4*3] = {-q[1], -q[2], -q[3],
                                  q[0], -q[3],  q[2],
                                  q[3],  q[0], -q[1],
                                 -q[2],  q[1],  q[0]};
        arm_matrix_instance_f32 W_mat = {4, 3, W_mat_data};
        float W_trans_data[3*4];
        arm_matrix_instance_f32 W_trans = {3, 4, W_trans_data};
        arm_mat_trans_f32(&W_mat, &W_trans);
        
        // calculate Process Noise Covariance Matrix Q = gyro_var^2 * W * W'
        arm_mat_mult_f32(&W_mat, &W_trans, &M1); // 4x4
        arm_mat_insert_mult_f32(&M1, ekf->Q, 0, 0, 0.25f*ekf->dt*ekf->dt*GYRO_VAR);
        arm_mat_set_diag_f32(ekf->Q, 4, 4, 3, GYRO_BIAS_VAR);

        EKFGetStateTransitionJacobian(ekf);
    }

    float Temp1_data[ekf->x_size*ekf->x_size];
    arm_matrix_instance_f32 Temp1_mat = {ekf->x_size, ekf->x_size, Temp1_data};
    float Temp2_data[ekf->x_size*ekf->x_size];
    arm_matrix_instance_f32 Temp2_mat = {ekf->x_size, ekf->x_size, Temp2_data};

    // calculate Covariance Matrix P(t)^ = F * P(t-1) * F' + Q(t)
    arm_mat_trans_f32(ekf->F, &Temp1_mat);
    arm_mat_mult_f32(ekf->P, &Temp1_mat, &Temp2_mat);
    arm_mat_mult_f32(ekf->F, &Temp2_mat, &Temp1_mat);
    arm_mat_add_f32(&Temp1_mat, ekf->Q, ekf->P);
}


void EKFPredictMeasurement(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr) {
    if (ekf->type == EKF1_type) {
        // h(t) = H * x^(t)

    } else if (ekf->type == EKF2_type) {
        if (ekf_corr->type == corr1_type) {
            // measurement prediction of type 1 correction step
            // expected barometer measurement equivalent to current height estimate
            BaroHeightToPressure(ekf->x[0], ekf->x[2], ekf_corr->h);
        } else if (ekf_corr->type == corr2_type) {
            // measurement prediction of type 2 correction step
            ekf_corr->h[0] = ekf->x[0];
            ekf_corr->h[1] = ekf->x[1];
        }

    } else if (ekf->type == EKF3_type) {
        // predict expected accelerometer and magnetometer readings (z vector)
        float *q = ekf->x;

        // Calculate expected a and m vectors from quaternion
        float DCMbi_data[9];
        arm_matrix_instance_f32 DCMbi = {3, 3, DCMbi_data};

        // define expected vectors in ENU frame
        float g_vec_enu[3] = {0, 0, 1};
        float m_vec_enu[3] = {0, cos(magnetic_dip_angle * PI / 180.f), -sin(magnetic_dip_angle * PI / 180.f)};

        // this section could be optimized (g and m contain zeroes)
        RotationMatrixFromQuaternion(q, &DCMbi, DCM_bi_WorldToBody);
        arm_mat_vec_mult_f32(&DCMbi, g_vec_enu, ekf_corr->h);
        arm_mat_vec_mult_f32(&DCMbi, m_vec_enu, &ekf_corr->h[3]);

    }
}

// calculate innovation v(t) = z(t) - h(q^(t))
void EKFGetInnovation(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr) {
    if (ekf->type == EKF1_type || ekf->type == EKF2_type) {
        arm_vecN_sub_f32(ekf_corr->z_size, ekf_corr->z, ekf_corr->h, ekf_corr->v);

    } else if (ekf->type == EKF3_type) {
        float *vt = ekf_corr->v;

        float a_norm_vec[3];
        float m_norm_vec[3];

        arm_vec3_copy_f32(&ekf_corr->z[0], a_norm_vec);
        arm_vec3_copy_f32(&ekf_corr->z[3], m_norm_vec);
        arm_vec3_normalize_f32(a_norm_vec);
        arm_vec3_normalize_f32(m_norm_vec);
    
        for (int i = 0; i < 3; i++) {
            vt[i]   = a_norm_vec[i] - ekf_corr->h[i];
            vt[i+3] = m_norm_vec[i] - ekf_corr->h[i+3];
        }

    }
}

// update ekf Gain
void EKFUpdateKalmanGain(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr) {
    float Temp1_data[ekf->x_size*ekf_corr->z_size];
    arm_matrix_instance_f32 Temp1_mat = {ekf->x_size, ekf_corr->z_size, Temp1_data};
    float Temp2_data[ekf_corr->z_size*ekf_corr->z_size];
    arm_matrix_instance_f32 Temp2_mat = {ekf_corr->z_size, ekf_corr->z_size, Temp2_data};
    float H_trans_data[ekf->x_size*ekf_corr->z_size];
    arm_matrix_instance_f32 H_trans_mat = {ekf->x_size, ekf_corr->z_size, H_trans_data};

    // retrieve observation jacobian H
    EKFGetObservationJacobian(ekf, ekf_corr);

    // calculate Measurement Prediction Covariance Matrix S(t) = H * P^(t) * H' + R
    arm_mat_trans_f32(ekf_corr->H, &H_trans_mat);
    arm_mat_mult_f32(ekf->P, &H_trans_mat, &Temp1_mat);
    arm_mat_mult_f32(ekf_corr->H, &Temp1_mat, &Temp2_mat);
    arm_mat_add_f32(&Temp2_mat, ekf_corr->R, ekf_corr->S);

    // calculate ekf Gain K(t) = P^(t) * H'(t) * inv(S(t))
    arm_mat_copy_f32(ekf_corr->S, &Temp2_mat);
    arm_mat_inverse_f32(&Temp2_mat, ekf_corr->S_inv);
    arm_mat_mult_f32(&H_trans_mat, ekf_corr->S_inv, &Temp1_mat);
    arm_mat_mult_f32(ekf->P, &Temp1_mat, ekf_corr->K);
}


// correct quaternion state vector x(t) = x^(t) + K(t) * vt(t)
void EKFCorrectStateV(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr) {
    float dx[ekf->x_size];

    arm_mat_vec_mult_f32(ekf_corr->K, ekf_corr->v, dx);
    arm_vecN_add_f32(ekf->x_size, ekf->x, dx, ekf->x);
}

// correct quaternion covariance matrix P(t) = P^(t) - K(t) * S(t) * K'(t)
void EKFCorrectCovariance(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr) {
    float Temp1_data[ekf->x_size*ekf->x_size];
    arm_matrix_instance_f32 Temp1_mat = {ekf->x_size, ekf->x_size, Temp1_data};
    float Temp2_data[ekf_corr->z_size*ekf->x_size];
    arm_matrix_instance_f32 Temp2_mat = {ekf_corr->z_size, ekf->x_size, Temp2_data};

    // update Covariance Matrix P(t) = P^(t) - K(t) * H(t) * P^(t)
    arm_mat_mult_f32(ekf_corr->H, ekf->P, &Temp2_mat);
    arm_mat_mult_f32(ekf_corr->K, &Temp2_mat, &Temp1_mat);
    arm_mat_sub_f32(ekf->P, &Temp1_mat, ekf->P);
}

// diagonal must be set to 1 in advance
void EKFGetStateTransitionJacobian(ekf_data_t *ekf) {
    if (ekf->type == EKF1_type) {

    } else if (ekf->type == EKF2_type) {

    } else if (ekf->type == EKF3_type) {
        float *q = ekf->x;

        float gyro_vec[3];
        float gyro_bias[3];

        arm_vec3_copy_f32(ekf->u, gyro_vec);
        arm_vec3_copy_f32(&ekf->x[4], gyro_bias);
        arm_vec3_sub_f32(gyro_vec, gyro_bias, gyro_vec);

        arm_mat_set_entry_f32(ekf->F, 0, 1, -0.5f*ekf->dt*gyro_vec[0]);
        arm_mat_set_entry_f32(ekf->F, 0, 2, -0.5f*ekf->dt*gyro_vec[1]);
        arm_mat_set_entry_f32(ekf->F, 0, 3, -0.5f*ekf->dt*gyro_vec[2]);
        arm_mat_set_entry_f32(ekf->F, 1, 0,  0.5f*ekf->dt*gyro_vec[0]);
        arm_mat_set_entry_f32(ekf->F, 1, 2,  0.5f*ekf->dt*gyro_vec[2]);
        arm_mat_set_entry_f32(ekf->F, 1, 3, -0.5f*ekf->dt*gyro_vec[1]);
        arm_mat_set_entry_f32(ekf->F, 2, 0,  0.5f*ekf->dt*gyro_vec[1]);
        arm_mat_set_entry_f32(ekf->F, 2, 1, -0.5f*ekf->dt*gyro_vec[2]);
        arm_mat_set_entry_f32(ekf->F, 2, 3,  0.5f*ekf->dt*gyro_vec[0]);
        arm_mat_set_entry_f32(ekf->F, 3, 0,  0.5f*ekf->dt*gyro_vec[2]);
        arm_mat_set_entry_f32(ekf->F, 3, 1,  0.5f*ekf->dt*gyro_vec[1]);
        arm_mat_set_entry_f32(ekf->F, 3, 2, -0.5f*ekf->dt*gyro_vec[0]);
        arm_mat_set_entry_f32(ekf->F, 0, 4,  0.5f*ekf->dt*q[1]);
        arm_mat_set_entry_f32(ekf->F, 0, 5,  0.5f*ekf->dt*q[2]);
        arm_mat_set_entry_f32(ekf->F, 0, 6,  0.5f*ekf->dt*q[3]);
        arm_mat_set_entry_f32(ekf->F, 1, 4, -0.5f*ekf->dt*q[0]);
        arm_mat_set_entry_f32(ekf->F, 1, 5,  0.5f*ekf->dt*q[3]);
        arm_mat_set_entry_f32(ekf->F, 1, 6, -0.5f*ekf->dt*q[2]);
        arm_mat_set_entry_f32(ekf->F, 2, 4, -0.5f*ekf->dt*q[3]);
        arm_mat_set_entry_f32(ekf->F, 2, 5, -0.5f*ekf->dt*q[0]);
        arm_mat_set_entry_f32(ekf->F, 2, 6,  0.5f*ekf->dt*q[1]);
        arm_mat_set_entry_f32(ekf->F, 3, 4,  0.5f*ekf->dt*q[2]);
        arm_mat_set_entry_f32(ekf->F, 3, 5, -0.5f*ekf->dt*q[1]);
        arm_mat_set_entry_f32(ekf->F, 3, 6, -0.5f*ekf->dt*q[0]);
    }
}

void EKFGetObservationJacobian(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr) {

    if (ekf->type == EKF1_type) {
        return;
    } else if (ekf->type == EKF2_type) {
        if(ekf_corr->type == corr1_type) {
            arm_mat_set_entry_f32(ekf_corr->H, 0, 0, (-g0_const * ekf_corr->h[0]) / (R_const * (T0_const + L_const * ekf->x[0])));
            arm_mat_set_entry_f32(ekf_corr->H, 0, 2, ekf_corr->h[0] / ekf->x[2]);

        } else if (ekf_corr->type == corr2_type) {
            arm_mat_set_diag_f32(ekf_corr->H, 0, 0, 2, 1);
        }

    } else if (ekf->type == EKF3_type) {
        float *q = ekf->x;

        //float g_vec_enu[3] = {0, 0, 1};
        float m_vec_enu[3] = {0, cos(magnetic_dip_angle * PI / 180.f), -sin(magnetic_dip_angle * PI / 180.f)};

        arm_mat_set_entry_f32(ekf_corr->H, 0, 0, -2*q[2]);
        arm_mat_set_entry_f32(ekf_corr->H, 0, 1,  2*q[3]);
        arm_mat_set_entry_f32(ekf_corr->H, 0, 2, -2*q[0]);
        arm_mat_set_entry_f32(ekf_corr->H, 0, 3,  2*q[1]);
        arm_mat_set_entry_f32(ekf_corr->H, 1, 0,  2*q[1]);
        arm_mat_set_entry_f32(ekf_corr->H, 1, 1,  2*q[0]);
        arm_mat_set_entry_f32(ekf_corr->H, 1, 2,  2*q[3]);
        arm_mat_set_entry_f32(ekf_corr->H, 1, 3,  2*q[2]);
        arm_mat_set_entry_f32(ekf_corr->H, 2, 0,  2*q[0]);
        arm_mat_set_entry_f32(ekf_corr->H, 2, 1, -2*q[1]);
        arm_mat_set_entry_f32(ekf_corr->H, 2, 2, -2*q[2]);
        arm_mat_set_entry_f32(ekf_corr->H, 2, 3,  2*q[3]);
        arm_mat_set_entry_f32(ekf_corr->H, 3, 0,  2*(m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]));
        arm_mat_set_entry_f32(ekf_corr->H, 3, 1,  2*(m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]));
        arm_mat_set_entry_f32(ekf_corr->H, 3, 2,  2*(m_vec_enu[1]*q[1]-m_vec_enu[2]*q[0]));
        arm_mat_set_entry_f32(ekf_corr->H, 3, 3,  2*(m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]));
        arm_mat_set_entry_f32(ekf_corr->H, 4, 0,  2*(m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]));
        arm_mat_set_entry_f32(ekf_corr->H, 4, 1, -2*(m_vec_enu[1]*q[1]-m_vec_enu[2]*q[0]));
        arm_mat_set_entry_f32(ekf_corr->H, 4, 2,  2*(m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]));
        arm_mat_set_entry_f32(ekf_corr->H, 4, 3, -2*(m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]));
        arm_mat_set_entry_f32(ekf_corr->H, 5, 0, -2*(m_vec_enu[1]*q[1]-m_vec_enu[2]*q[0]));
        arm_mat_set_entry_f32(ekf_corr->H, 5, 1, -2*(m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]));
        arm_mat_set_entry_f32(ekf_corr->H, 5, 2,  2*(m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]));
        arm_mat_set_entry_f32(ekf_corr->H, 5, 3,  2*(m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]));
    }
}

void EKFPredictionStep(ekf_data_t *ekf) {
    // skip prediction step if status flag is set to inactive
    if (ekf->prediction_state != active) return;

    // predicting state vector x
    EKFPredictStateV(ekf);

    // predicting state vector's covariance matrix P
    EKFPredictCovariance(ekf);
}

void EKFCorrectionStep(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr) {
    // skip correction step if status flag is set to inactive
    if (ekf_corr->correction_state != active) return;

    // predict measurement
    EKFPredictMeasurement(ekf, ekf_corr);

    // calculate Kalman Gain
    EKFUpdateKalmanGain(ekf, ekf_corr);

    // calculate difference between predicted and actual measurement
    EKFGetInnovation(ekf, ekf_corr);

    // correct state vector based on innovation and Kalman Gain
    EKFCorrectStateV(ekf, ekf_corr);

    // update covariance matrix
    EKFCorrectCovariance(ekf, ekf_corr);
}

// calculate Normalized Innovation Squared
void EKFgetNIS(ekf_data_t *ekf, ekf_corr_data_t *ekf_corr, float *NIS) {
    float buffer[ekf_corr->z_size];

    // NIS = v' * S_inv * v
    arm_mat_vec_mult_f32(ekf_corr->S_inv, ekf_corr->v, buffer);
    arm_vecN_dot_prod_f32(ekf_corr->z_size, ekf_corr->v, buffer, NIS);
}

// returns true if predicted variance obeys thresholds
bool EKFisAligned(ekf_data_t *ekf) {
    if(ekf->type == EKF2_type) {
        if (arm_mat_get_entry_f32(ekf->P, 0, 0) > P_VAR_HEIGHT_THRESH) return false;
        else if (arm_mat_get_entry_f32(ekf->P, 1, 1) > P_VAR_VELZ_THRESH) return false;
        else if (arm_mat_get_entry_f32(ekf->P, 2, 2) > P_VAR_PREF_THRESH) return false;
        else return true;
    } else if (ekf->type == EKF3_type) {
        return true;
    }
}