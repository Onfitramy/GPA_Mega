#include "navigation.h"
#include "armMathAddon.h"

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
    double dz = ECEF[2] - ECEF_ref[2];

    // convert reference point angles to radians
    double lat_rad = WGS84_ref[0] * PI / 180.;
    double lon_rad = WGS84_ref[1] * PI / 180.;

    // transform difference to ENU frame
    ENU[0] = -sin(lon_rad) * dx + cos(lon_rad) * dy;                                                    // East
    ENU[1] = -sin(lat_rad) * cos(lon_rad) * dx - sin(lat_rad) * sin(lon_rad) * dy + cos(lat_rad) * dz;  // North
    ENU[2] =  cos(lat_rad) * cos(lon_rad) * dx + cos(lat_rad) * sin(lon_rad) * dy + sin(lat_rad) * dz;  // West
}

// attitude estimation using magnetometer and accelerometer
void OrientationFix(float *accel_vec, float *mag_vec, float *output_angles) {
    // unit basis vectors of inertial system expressed in body coordinates
    float base_xi[3];
    float base_yi[3];
    float base_zi[3];

    // normalize a and m vectors
    arm_vec3_copy_f32(mag_vec, base_yi);
    arm_vec3_copy_f32(accel_vec, base_zi);
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

// create rotation matrix from euler angles
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
void Vec3_BodyToWorld(float *vec3_body, arm_matrix_instance_f32 *mat_rot, float *vec3_world) {
    float M_data[9];
    arm_matrix_instance_f32 M = {3, 3, M_data};

    arm_mat_trans_f32(mat_rot, &M);
    arm_mat_vec_mult_f32(&M, vec3_body, vec3_world);
}

// create new Kalman Filter instance
void KalmanFilterInit(Kalman_Instance *Kalman, x6z3u3KalmanData *data, uint8_t x_vec_size, uint8_t z_vec_size, uint8_t u_vec_size) {
    Kalman->x_size = x_vec_size;
    Kalman->z_size = z_vec_size;
    Kalman->u_size = u_vec_size;

    arm_mat_init_f32(&Kalman->M1, x_vec_size, x_vec_size, data->M1);
    arm_mat_init_f32(&Kalman->M2, x_vec_size, x_vec_size, data->M2);
    arm_mat_init_f32(&Kalman->M3, z_vec_size, z_vec_size, data->M3);
    arm_mat_init_f32(&Kalman->M4, z_vec_size, z_vec_size, data->M4);
    arm_mat_init_f32(&Kalman->M5, x_vec_size, z_vec_size, data->M5);
    arm_mat_init_f32(&Kalman->M6, x_vec_size, z_vec_size, data->M6);
    arm_mat_init_f32(&Kalman->M7, z_vec_size, x_vec_size, data->M7);

    Kalman->dx = data->dx;
    Kalman->dz = data->dz;
}

// predicting state vector x
void KalmanFilterPredictSV(Kalman_Instance *Kalman, arm_matrix_instance_f32 *A_mat, float *x_vec, arm_matrix_instance_f32 *B_mat, float *u_vec) {
    arm_mat_vec_mult_f32(A_mat, x_vec, x_vec);
    arm_mat_vec_mult_f32(B_mat, u_vec, Kalman->dx);
    arm_vecN_add_f32(Kalman->x_size, x_vec, Kalman->dx, x_vec);
}

// predicting covariance matrix P
void KalmanFilterPredictCM(Kalman_Instance *Kalman, arm_matrix_instance_f32 *A_mat, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *Q_mat) {
    arm_mat_trans_f32(A_mat, &Kalman->M1);
    arm_mat_mult_f32(P_mat, &Kalman->M1, &Kalman->M2);
    arm_mat_mult_f32(A_mat, &Kalman->M2, P_mat);
    arm_mat_add_f32(P_mat, Q_mat, P_mat);
}

// updating Kalman Gain
void KalmanFilterUpdateGain(Kalman_Instance *Kalman, arm_matrix_instance_f32 *P_mat, arm_matrix_instance_f32 *C_mat, arm_matrix_instance_f32 *R_mat, arm_matrix_instance_f32 *K_mat) {
    arm_mat_trans_f32(C_mat, &Kalman->M5);
    arm_mat_mult_f32(P_mat, &Kalman->M5, &Kalman->M6);
    arm_mat_mult_f32(C_mat, &Kalman->M6, &Kalman->M3);
    arm_mat_add_f32(&Kalman->M3, R_mat, &Kalman->M4);
    arm_mat_inverse_f32(&Kalman->M4, &Kalman->M3);
    arm_mat_mult_f32(&Kalman->M5, &Kalman->M3, &Kalman->M6);
    arm_mat_mult_f32(P_mat, &Kalman->M6, K_mat);
}

// corecting state vector x
void KalmanFilterCorrectSV(Kalman_Instance *Kalman, arm_matrix_instance_f32 *K_mat, float *z_vec, arm_matrix_instance_f32 *C_mat, float *x_vec) {
    arm_mat_vec_mult_f32(C_mat, x_vec, Kalman->dz);
    arm_vec3_sub_f32(z_vec, Kalman->dz, Kalman->dz);
    arm_mat_vec_mult_f32(K_mat, Kalman->dz, Kalman->dx);
    arm_vecN_add_f32(6, x_vec, Kalman->dx, x_vec);
}

// correcting x's covariance matrix P
void KalmanFilterCorrectCM(Kalman_Instance *Kalman, arm_matrix_instance_f32 *K_mat, arm_matrix_instance_f32 *C_mat, arm_matrix_instance_f32 *P_mat) {
    arm_mat_mult_f32(C_mat, P_mat, &Kalman->M7);
    arm_mat_mult_f32(K_mat, &Kalman->M7, &Kalman->M2);
    arm_mat_sub_f32(P_mat, &Kalman->M2, P_mat);
}