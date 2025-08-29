#include "navigation.h"

// define constants of WGS84
const double a = 6378137.;
const double f = 1. / 298.257223563;
const double e2 = 0.006694379990141317;

// set boundaries for angle
void normalizeAngle(float *angle, float upper_boundary, float lower_boundary, float step_size) {
    if(*angle > upper_boundary) {
        *angle -= step_size;
    } 
    if(*angle < lower_boundary) {
        *angle += step_size;
    }
}

// set boundaries for angles
void normalizeAngleVector(float *angle_vec, uint8_t pos_top, uint8_t pos_bottom, float upper_boundary, float lower_boundary, float step_size) {
    for(int i = pos_top; i <= pos_bottom; i++) {
        if( angle_vec[i] > upper_boundary) {
            angle_vec[i] -= step_size;
        } 
        if( angle_vec[i] < lower_boundary) {
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
    for(int i = pos_top; i <= pos_bottom; i++) {
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
    if(dcm_type == DCM_bi_WorldToBody) {
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
    if(trace > 0) {
        float s = 2.0f * sqrtf(trace + 1.0f);
        q[0] = 0.25f * s;
        q[1] = (arm_mat_get_entry_f32(mat, 2, 1) - arm_mat_get_entry_f32(mat, 1, 2)) / s;
        q[2] = (arm_mat_get_entry_f32(mat, 0, 2) - arm_mat_get_entry_f32(mat, 2, 0)) / s;
        q[3] = (arm_mat_get_entry_f32(mat, 1, 0) - arm_mat_get_entry_f32(mat, 0, 1)) / s;
    } else if(arm_mat_get_entry_f32(mat, 0, 0) > arm_mat_get_entry_f32(mat, 1, 1) && arm_mat_get_entry_f32(mat, 0, 0) > arm_mat_get_entry_f32(mat, 2, 2)) {
        float s = 2.0f * sqrtf(1.0f + arm_mat_get_entry_f32(mat, 0, 0) - arm_mat_get_entry_f32(mat, 1, 1) - arm_mat_get_entry_f32(mat, 2, 2));
        q[0] = (arm_mat_get_entry_f32(mat, 2, 1) - arm_mat_get_entry_f32(mat, 1, 2)) / s;
        q[1] = 0.25f * s;
        q[2] = (arm_mat_get_entry_f32(mat, 0, 1) + arm_mat_get_entry_f32(mat, 1, 0)) / s;
        q[3] = (arm_mat_get_entry_f32(mat, 0, 2) + arm_mat_get_entry_f32(mat, 2, 0)) / s;
    } else if(arm_mat_get_entry_f32(mat, 1, 1) > arm_mat_get_entry_f32(mat, 2, 2)) {
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
    arm_quaternion_normalize_f32(q, q);
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

// transform vector from body frame to world frame
void Vec3_BodyToWorld(float *vec3_body, arm_matrix_instance_f32 *mat, float *vec3_world) {
    float M_data[9];
    arm_matrix_instance_f32 M = {3, 3, M_data};

    arm_mat_trans_f32(mat, &M);
    arm_mat_vec_mult_f32(&M, vec3_body, vec3_world);
}

// calculate initial quaternion from accelerometer and magnetometer readings
void EKFInitQuaternion(float *q, float *a_vec, float *m_vec) {
    float DCMbi_data[9];
    arm_matrix_instance_f32 DCMbi = {3, 3, DCMbi_data};

    RotationMatrixOrientationFix(a_vec, m_vec, &DCMbi, DCM_bi_WorldToBody);
    QuaternionFromRotationMatrix(&DCMbi, q);
}

// prediction step for quaternion state vector
void EKFPredictQuaternionSV(float *q, float *gyr_vec, const float dt) {
    float omega[4];
    omega[0] = 0;
    omega[1] = gyr_vec[0];
    omega[2] = gyr_vec[1];
    omega[3] = gyr_vec[2];

    float q_omega[4];
    arm_quaternion_product_f32(q, omega, q_omega);

    for(int i = 0; i < 4; i++) {
        q[i] = q[i] + 0.5f * q_omega[i] * dt;
    }
    arm_quaternion_normalize_f32(q, q);
}

// prediction step for quaternion covariance matrix
void EKFPredictQuaternionCM(float *q, const float dt, const float gyro_var, arm_matrix_instance_f32 *F_mat, arm_matrix_instance_f32 *P_mat) {
    float M1_data[4*4];
    arm_matrix_instance_f32 M1 = {4*4, M1_data};
    float M2_data[4*4];
    arm_matrix_instance_f32 M2 = {4*4, M2_data};

    // defube W and W'
    float W_mat_data[4*3] = {-q[1], -q[2], -q[3],
                              q[0], -q[3],  q[2],
                              q[3],  q[0], -q[1],
                             -q[2],  q[1],  q[0]};
    arm_matrix_instance_f32 W_mat = {4, 3, W_mat_data};
    float W_trans_data[3*4];
    arm_matrix_instance_f32 W_trans = {3, 4, W_trans_data};
    arm_mat_trans_f32(&W_mat, &W_trans);

    // calculate Process Noise Covariance Matrix Q = gyro_var^2 * W * W'
    float Q_temp_data[4*4];
    arm_matrix_instance_f32 Q_mat = {4, 4, Q_temp_data};
    arm_mat_mult_f32(&W_mat, &W_trans, &Q_mat);
    arm_mat_scale_f32(&Q_mat, 0.25f*dt*dt*gyro_var*gyro_var, &Q_mat);

    // calculate Covariance Matrix P(t)^ = F * P(t-1) * F' + Q(t)
    arm_mat_trans_f32(F_mat, &M1);
    arm_mat_mult_f32(P_mat, &M1, &M2);
    arm_mat_mult_f32(F_mat, &M2, &M1);
    arm_mat_add_f32(&M1, &Q_mat, P_mat);
}

// predict expected accelerometer and magnetometer readings (z vector)
void EKFPredictQuaternionZ(float *q, float *a_exp, float *m_exp) {
    // Calculate expected a and m vectors from quaternion
    float DCMbi_data[9];
    arm_matrix_instance_f32 DCMbi = {3, 3, DCMbi_data};

    // define expected vectors in ENU frame
    float g_vec_enu[3] = {0, 0, 1};
    float m_vec_enu[3] = {0, cos(magnetic_dip_angle * PI / 180.f), -sin(magnetic_dip_angle * PI / 180.f)};

    // this section could be optimized (g and m contain zeroes)
    RotationMatrixFromQuaternion(q, &DCMbi, DCM_bi_WorldToBody);
    arm_mat_vec_mult_f32(&DCMbi, g_vec_enu, a_exp);
    arm_mat_vec_mult_f32(&DCMbi, m_vec_enu, m_exp);
}

// calculate innovation v(t) = z(t) - h(q^(t))
void EKFGetInnovation(float *a_exp, float *m_exp, float *a_vec, float *m_vec, float *vt) {
    float a_norm_vec[3];
    float m_norm_vec[3];

    arm_vec3_copy_f32(a_vec, a_norm_vec);
    arm_vec3_copy_f32(m_vec, m_norm_vec);
    arm_vec3_normalize_f32(a_norm_vec);
    arm_vec3_normalize_f32(m_norm_vec);
    
    
    for(int i = 0; i < 3; i++) {
        vt[i]   = a_vec[i] - a_exp[i];
        vt[i+3] = m_vec[i] - m_exp[i];
    }
}

// update Kalman Gain
void EKFUpdateKalmanGain(arm_matrix_instance_f32 *H_mat, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *R_mat, arm_matrix_instance_f32 *S_mat, arm_matrix_instance_f32 *K_mat) {
    float H_trans_data[4*6];
    arm_matrix_instance_f32 H_trans_mat = {4, 6, H_trans_data};
    float Temp1_data[4*6];
    arm_matrix_instance_f32 Temp1_mat = {4, 6, Temp1_data};
    float Temp2_data[6*6];
    arm_matrix_instance_f32 Temp2_mat = {6, 6, Temp2_data};

    // calculate Measurement Prediction Covariance Matrix S(t) = H * P^(t) * H' + R
    arm_mat_trans_f32(H_mat, &H_trans_mat);
    arm_mat_mult_f32(P_mat, &H_trans_mat, &Temp1_mat);
    arm_mat_mult_f32(H_mat, &Temp1_mat, &Temp2_mat);
    arm_mat_add_f32(&Temp2_mat, R_mat, S_mat);

    // calculate Kalman Gain K(t) = P^(t) * H'(t) * inv(S(t))
    arm_mat_inverse_f32(S_mat, &Temp2_mat);
    arm_mat_mult_f32(&H_trans_mat, &Temp2_mat, &Temp1_mat);
    arm_mat_mult_f32(P_mat, &Temp1_mat, K_mat);
}


// correct quaternion state vector x(t) = x^(t) + K(t) * vt(t)
void EKFCorrectQuaternionSV(float *q, arm_matrix_instance_f32 *K_mat, float *vt) {
    float dq[4];
    arm_mat_vec_mult_f32(K_mat, vt, dq);
    arm_vecN_add_f32(4, q, dq, q);
}

// correct quaternion covariance matrix P(t) = P^(t) - K(t) * S(t) * K'(t)
void EKFCorrectQuaternionCM(kalman_data_t *Kalman, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *K_mat, arm_matrix_instance_f32 *H_mat) {
    float M2_data[Kalman->x_size*Kalman->x_size];
    arm_matrix_instance_f32 M2 = {Kalman->x_size, Kalman->x_size, M2_data};
    float M7_data[Kalman->z_size*Kalman->x_size];
    arm_matrix_instance_f32 M7 = {Kalman->z_size, Kalman->x_size, M7_data};

    // update Covariance Matrix P(t) = P^(t) - K(t) * H(t) * P^(t)
    arm_mat_mult_f32(H_mat, P_mat, &M7);
    arm_mat_mult_f32(K_mat, &M7, &M2);
    arm_mat_sub_f32(P_mat, &M2, P_mat);
}

void EKFGetStateTransitionJacobian(float *gyro_vec, float dt, arm_matrix_instance_f32 *F_mat) {
    arm_mat_set_entry_f32(F_mat, 0, 1, -0.5f*dt*gyro_vec[0]);
    arm_mat_set_entry_f32(F_mat, 0, 2, -0.5f*dt*gyro_vec[1]);
    arm_mat_set_entry_f32(F_mat, 0, 3, -0.5f*dt*gyro_vec[2]);
    arm_mat_set_entry_f32(F_mat, 1, 0,  0.5f*dt*gyro_vec[0]);
    arm_mat_set_entry_f32(F_mat, 1, 2,  0.5f*dt*gyro_vec[2]);
    arm_mat_set_entry_f32(F_mat, 1, 3, -0.5f*dt*gyro_vec[1]);
    arm_mat_set_entry_f32(F_mat, 2, 0,  0.5f*dt*gyro_vec[1]);
    arm_mat_set_entry_f32(F_mat, 2, 1, -0.5f*dt*gyro_vec[2]);
    arm_mat_set_entry_f32(F_mat, 2, 3,  0.5f*dt*gyro_vec[0]);
    arm_mat_set_entry_f32(F_mat, 3, 0,  0.5f*dt*gyro_vec[2]);
    arm_mat_set_entry_f32(F_mat, 3, 1,  0.5f*dt*gyro_vec[1]);
    arm_mat_set_entry_f32(F_mat, 3, 2, -0.5f*dt*gyro_vec[0]);
}

void EKFGetObservationJacobian(float *q, arm_matrix_instance_f32 *H_mat) {
    //float g_vec_enu[3] = {0, 0, 1};
    float m_vec_enu[3] = {0, cos(magnetic_dip_angle * PI / 180.f), -sin(magnetic_dip_angle * PI / 180.f)};

    arm_mat_set_entry_f32(H_mat, 0, 0, -2*q[2]);
    arm_mat_set_entry_f32(H_mat, 0, 1,  2*q[3]);
    arm_mat_set_entry_f32(H_mat, 0, 2, -2*q[0]);
    arm_mat_set_entry_f32(H_mat, 0, 3,  2*q[1]);
    arm_mat_set_entry_f32(H_mat, 1, 0,  2*q[1]);
    arm_mat_set_entry_f32(H_mat, 1, 1,  2*q[0]);
    arm_mat_set_entry_f32(H_mat, 1, 2,  2*q[3]);
    arm_mat_set_entry_f32(H_mat, 1, 3,  2*q[2]);
    arm_mat_set_entry_f32(H_mat, 2, 0,  2*q[0]);
    arm_mat_set_entry_f32(H_mat, 2, 1, -2*q[1]);
    arm_mat_set_entry_f32(H_mat, 2, 2, -2*q[2]);
    arm_mat_set_entry_f32(H_mat, 2, 3,  2*q[3]);
    arm_mat_set_entry_f32(H_mat, 3, 0,  2*(m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]));
    arm_mat_set_entry_f32(H_mat, 3, 1,  2*(m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]));
    arm_mat_set_entry_f32(H_mat, 3, 2,  2*(m_vec_enu[1]*q[1]-m_vec_enu[2]*q[0]));
    arm_mat_set_entry_f32(H_mat, 3, 3,  2*(m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]));
    arm_mat_set_entry_f32(H_mat, 4, 0,  2*(m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]));
    arm_mat_set_entry_f32(H_mat, 4, 1, -2*(m_vec_enu[1]*q[1]-m_vec_enu[2]*q[0]));
    arm_mat_set_entry_f32(H_mat, 4, 2,  2*(m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]));
    arm_mat_set_entry_f32(H_mat, 4, 3, -2*(m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]));
    arm_mat_set_entry_f32(H_mat, 5, 0, -2*(m_vec_enu[1]*q[1]-m_vec_enu[2]*q[0]));
    arm_mat_set_entry_f32(H_mat, 5, 1, -2*(m_vec_enu[1]*q[0]+m_vec_enu[2]*q[1]));
    arm_mat_set_entry_f32(H_mat, 5, 2,  2*(m_vec_enu[1]*q[3]-m_vec_enu[2]*q[2]));
    arm_mat_set_entry_f32(H_mat, 5, 3,  2*(m_vec_enu[1]*q[2]+m_vec_enu[2]*q[3]));
}

// create new Kalman Filter instance
void KalmanFilterInit(kalman_data_t *Kalman, kalman_instance_t kalman_type, uint8_t x_vec_size, uint8_t z_vec_size, uint8_t u_vec_size) {
    Kalman->type = kalman_type;
    Kalman->x_size = x_vec_size;
    Kalman->z_size = z_vec_size;
    Kalman->u_size = u_vec_size;
}

// predicting state vector x
void KalmanFilterPredictSV(kalman_data_t *Kalman, arm_matrix_instance_f32 *F_mat, float *x_vec, arm_matrix_instance_f32 *B_mat, float *u_vec) {
    float dx[Kalman->x_size];

    // calculate predicted state x^(t) = F * x(t-1) + B * u(t)
    arm_mat_vec_mult_f32(F_mat, x_vec, x_vec);
    arm_mat_vec_mult_f32(B_mat, u_vec, dx);
    arm_vecN_add_f32(Kalman->x_size, x_vec, dx, x_vec);
}

// predicting covariance matrix P
void KalmanFilterPredictCM(kalman_data_t *Kalman, const arm_matrix_instance_f32 *F_mat, arm_matrix_instance_f32 *P_mat, const arm_matrix_instance_f32 *Q_mat) {
    float M1_data[Kalman->x_size*Kalman->x_size];
    arm_matrix_instance_f32 M1 = {Kalman->x_size, Kalman->x_size, M1_data};
    float M2_data[Kalman->x_size*Kalman->x_size];
    arm_matrix_instance_f32 M2 = {Kalman->x_size, Kalman->x_size, M2_data};

    // calculate Covariance Matrix P^(t) = F * P(t-1) * F' + Q(t)
    arm_mat_trans_f32(F_mat, &M1);
    arm_mat_mult_f32(P_mat, &M1, &M2);
    arm_mat_mult_f32(F_mat, &M2, &M1);
    arm_mat_add_f32(&M1, Q_mat, P_mat);
}

// updating Kalman Gain
void KalmanFilterUpdateGain(kalman_data_t *Kalman, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *H_mat, arm_matrix_instance_f32 *R_mat, arm_matrix_instance_f32 *K_mat) {
    float S_data[Kalman->z_size*Kalman->z_size];
    arm_matrix_instance_f32 S_mat = {Kalman->z_size, Kalman->z_size, S_data};
    float H_trans_data[Kalman->x_size*Kalman->z_size];
    arm_matrix_instance_f32 H_trans_mat = {Kalman->x_size, Kalman->z_size, H_trans_data};
    float Temp1_data[Kalman->x_size*Kalman->z_size];
    arm_matrix_instance_f32 Temp1_mat = {Kalman->x_size, Kalman->z_size, Temp1_data};
    float Temp2_data[Kalman->z_size*Kalman->z_size];
    arm_matrix_instance_f32 Temp2_mat = {Kalman->z_size, Kalman->z_size, Temp2_data};
    
    // calculate Measurement Prediction Covariance S(t) = H(t) * P^(t) * H'(t) + R
    arm_mat_trans_f32(H_mat, &H_trans_mat);
    arm_mat_mult_f32(P_mat, &H_trans_mat, &Temp1_mat);
    arm_mat_mult_f32(H_mat, &Temp1_mat, &Temp2_mat);
    arm_mat_add_f32(&Temp2_mat, R_mat, &S_mat);

    // calculate Kalman Gain K(t) = P^(t) * H'(t) * inv(S(t))
    arm_mat_inverse_f32(&S_mat, &Temp2_mat);
    arm_mat_mult_f32(&H_trans_mat, &Temp2_mat, &Temp1_mat);
    arm_mat_mult_f32(P_mat, &Temp1_mat, K_mat);
}

// corecting state vector x
void KalmanFilterCorrectSV(kalman_data_t *Kalman, arm_matrix_instance_f32 *K_mat, float *z_vec, arm_matrix_instance_f32 *H_mat, float *x_vec) {
    float dx[Kalman->x_size];
    float vt[Kalman->z_size];

    // calculate innovation vt(t) = z(t) - H * x^(t)
    arm_mat_vec_mult_f32(H_mat, x_vec, vt);
    arm_vecN_sub_f32(Kalman->z_size, z_vec, vt, vt);

    // update state x(t) = x^(t) + K(t) * vt(t)
    arm_mat_vec_mult_f32(K_mat, vt, dx);
    arm_vecN_add_f32(Kalman->x_size, x_vec, dx, x_vec);
}

// correcting x's covariance matrix P
void KalmanFilterCorrectCM(kalman_data_t *Kalman, arm_matrix_instance_f32 *K_mat, arm_matrix_instance_f32 *H_mat, arm_matrix_instance_f32 *P_mat) {
    float M2_data[Kalman->x_size*Kalman->x_size];
    arm_matrix_instance_f32 M2 = {Kalman->x_size, Kalman->x_size, M2_data};
    float M7_data[Kalman->z_size*Kalman->x_size];
    arm_matrix_instance_f32 M7 = {Kalman->z_size, Kalman->x_size, M7_data};

    // update Covariance Matrix P(t) = P^(t) - K(t) * H(t) * P^(t)
    arm_mat_mult_f32(H_mat, P_mat, &M7);
    arm_mat_mult_f32(K_mat, &M7, &M2);
    arm_mat_sub_f32(P_mat, &M2, P_mat);
}