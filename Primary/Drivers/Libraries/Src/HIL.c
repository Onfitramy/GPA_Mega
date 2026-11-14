#include "HIL.h"
#include "navigation.h"
#include "control.h"

// all variables used by HIL
float magfield[3] = {0.0069f, 0.1973f, -0.4431f};

float HIL_DCM_bi_data[9] = { 0 };
arm_matrix_instance_f32 HIL_DCM_bi = {3, 3, HIL_DCM_bi_data};
float HIL_DCM_ib_data[9] = { 0 };
arm_matrix_instance_f32 HIL_DCM_ib = {3, 3, HIL_DCM_ib_data};

HIL_t HIL;

void HILInit() {
    // store pointer to DCMs
    HIL.DCM_bi = &HIL_DCM_bi;
    HIL.DCM_ib = &HIL_DCM_ib;

    // set start time such that ignition occurs at t = 0s
    HIL.t = -HIL_IGNITION_TIME;

    // initialize the states
    HIL.state = HIL_STATE_PRELAUNCH;
    HIL.m = HIL_WET_MASS;
    memset(HIL.F_b, 0, 3 * sizeof(float));
    memset(HIL.r_i, 0, 3 * sizeof(float));
    memset(HIL.v_i, 0, 3 * sizeof(float));
    memset(HIL.v_b, 0, 3 * sizeof(float));
    memset(HIL.w_b, 0, 3 * sizeof(float));
    memset(HIL.a_b, 0, 3 * sizeof(float));
    memset(HIL.a_i, 0, 3 * sizeof(float));

    // calculate initial attitude
    HIL.euler[0] = 0.5f*PI;
    HIL.euler[1] = 0.0f;
    HIL.euler[2] = -PI;
    RotationMatrixFromEuler(HIL.euler[0], HIL.euler[1], HIL.euler[2], HIL.DCM_bi);
    arm_mat_trans_f32(HIL.DCM_bi, HIL.DCM_ib);
}

void HIL_compute_grav_drag_force(float cd_s, float v_abs) {
    // calculate body-frame gravity vector
    arm_mat_vec_mult_f32(HIL.DCM_bi, gravity_world_vec, HIL.F_b);
    arm_vec3_scalar_mult_f32(HIL.F_b, HIL.m, HIL.F_b);

    // calculate body-frame drag vector
    float rho = CalculateAirDensity(HIL.r_i[2]);
    HIL.F_b[1] -= 0.5f * rho * v_abs * v_abs * cd_s;
}
void HIL_body_force(float v_abs) {
    if (HIL.state == HIL_STATE_PRELAUNCH || HIL.state == HIL_STATE_LANDED) {
        memset(HIL.F_b, 0, 3 * sizeof(float));
        return;
    }

    // calculate the current mass
    if (HIL.state == HIL_STATE_BURN)
        HIL.m = HIL_WET_MASS - HIL_PROPELLANT_MASS / HIL_MOTOR_BURN_TIME * HIL.t;
    else
        HIL.m = HIL_WET_MASS - HIL_PROPELLANT_MASS;

    // determine the CD depending on the current flight phase
    float CD_S;
    if (HIL.state == HIL_STATE_DROGUE)
        CD_S = HIL_CD_S_DROGUE;
    else if (HIL.state == HIL_STATE_MAIN)
        CD_S = HIL_CD_S_MAIN;
    else
        CD_S = ComputeAirbrakeDrag(v_abs, acs_target_angle_deg) * AREF;

    // compute the total force in body-frame
    HIL_compute_grav_drag_force(CD_S, v_abs);
    if (HIL.state == HIL_STATE_BURN)
        HIL.F_b[1] += HIL_MOTOR_THRUST;
    if (HIL.r_i[2] < HIL_RAIL_LENGTH) {
        HIL.F_b[0] = 0.0f;
        HIL.F_b[2] = 0.0f;
    }
}
void HILupdateStates(float dt) {
    // update the simulation time
    HIL.t += dt;

    // conduct state transitions
    switch (HIL.state) {
    case HIL_STATE_PRELAUNCH:
        if (HIL.t >= 0.0f)
            HIL.state = HIL_STATE_BURN;
        break;
    case HIL_STATE_BURN:
        if (HIL.t >= HIL_MOTOR_BURN_TIME)
            HIL.state = HIL_STATE_COAST;
        break;
    case HIL_STATE_COAST:
        if (HIL.v_i[2] < 0.0f)
            HIL.state = HIL_STATE_DROGUE;
        break;
    case HIL_STATE_DROGUE:
        if (HIL.r_i[2] <= MAIN_MIN_DEPLOY_HEIGHT)
            HIL.state = HIL_STATE_MAIN;
        break;
    case HIL_STATE_MAIN:
        if (HIL.r_i[2] <= 0.0f)
            HIL.state = HIL_STATE_LANDED;
        break;
    default:
        // landed
        memset(HIL.v_i, 0, 3 * sizeof(float));
        memset(HIL.v_b, 0, 3 * sizeof(float));
        break;
    }

    // compute the total force acting on the vehicle in body-frame
    float velo_abs = arm_vec3_length_f32(HIL.v_i);
    HIL_body_force(velo_abs);

    // rigid-body kinetics (integrate acceleration)
    arm_vec3_scalar_mult_f32(HIL.F_b, 1.0f/HIL.m, HIL.a_b);
    arm_mat_vec_mult_f32(HIL.DCM_ib, HIL.a_b, HIL.a_i);
    for (int i = 0; i < 3; i++) {
        HIL.r_i[i] += HIL.v_i[i] * dt;
        HIL.v_i[i] += HIL.a_i[i] * dt;
    }
    arm_mat_vec_mult_f32(HIL.DCM_bi, HIL.v_i, HIL.v_b);

    // --- update orientation ---
    float HIL_DCM_bi_data_prev[9] = { 0 };
    arm_matrix_instance_f32 HIL_DCM_bi_prev = {3, 3, HIL_DCM_bi_data_prev};
    arm_mat_copy_f32(HIL.DCM_bi, &HIL_DCM_bi_prev);
    float buffer_mat_data[9] = { 0 };
    arm_matrix_instance_f32 buffer_mat = {3, 3, buffer_mat_data};

    // rotate around y-axis to simulate roll
    float roll_angle = HIL_C_ROLL * velo_abs * dt;
    float R_roll_data[9] = { 0 }; // body_old -> body_new
    arm_matrix_instance_f32 R_roll = {3, 3, R_roll_data};
    arm_mat_set_entry_f32(&R_roll, 1, 1, 1.0f);
    arm_mat_set_entry_f32(&R_roll, 0, 0, cosf(roll_angle));
    arm_mat_set_entry_f32(&R_roll, 2, 2, cosf(roll_angle));
    arm_mat_set_entry_f32(&R_roll, 0, 2, sinf(roll_angle));
    arm_mat_set_entry_f32(&R_roll, 2, 0, -sinf(roll_angle));
    arm_mat_mult_f32(HIL.DCM_bi, &R_roll, &buffer_mat);

    // extract the rotated x-axis
    float bx_axis_i[3];
    arm_mat_get_column_f32(&buffer_mat, 0, bx_axis_i);

    // the new y-axis is parallel to the velocity
    float by_axis_i[3];
    if (velo_abs >= 1e-3) {
        arm_vec3_copy_f32(HIL.v_i, by_axis_i);
        arm_vec3_normalize_f32(by_axis_i);
    } else {
        arm_mat_get_column_f32(&buffer_mat, 1, by_axis_i);
    }

    // the z-axis completes the right-hand cartesian coordinate system
    float bz_axis_i[3];
    arm_vec3_cross_product_f32(bx_axis_i, by_axis_i, bz_axis_i);
    arm_vec3_normalize_f32(bz_axis_i);

    // re-orthogonalize the x-axis and normalize
    arm_vec3_cross_product_f32(by_axis_i, bz_axis_i, bx_axis_i);
    arm_vec3_normalize_f32(bx_axis_i);
    arm_vec3_normalize_f32(by_axis_i);

    // fill DCM and calculate euler angles
    arm_mat_set_column_f32(HIL.DCM_ib, 0, bx_axis_i);
    arm_mat_set_column_f32(HIL.DCM_ib, 1, by_axis_i);
    arm_mat_set_column_f32(HIL.DCM_ib, 2, bz_axis_i);
    arm_mat_trans_f32(HIL.DCM_ib, HIL.DCM_bi);
    EulerFromRotationMatrix(HIL.DCM_bi, HIL.euler);

    // calculate the angular velocity during the time step
    arm_mat_mult_f32(&HIL_DCM_bi_prev, HIL.DCM_ib, &buffer_mat);
    float delta_angle = 0.0f;
    float trace_dDCM = arm_mat_trace_f32(&buffer_mat);
    if (trace_dDCM < 3.0f && trace_dDCM > -3.0f)
        delta_angle = acosf((trace_dDCM - 1.0f) / 2.0f);
    HIL.w_b[0] = arm_mat_get_entry_f32(&buffer_mat, 2, 1) - arm_mat_get_entry_f32(&buffer_mat, 1, 2);
    HIL.w_b[1] = arm_mat_get_entry_f32(&buffer_mat, 0, 2) - arm_mat_get_entry_f32(&buffer_mat, 2, 0);
    HIL.w_b[2] = arm_mat_get_entry_f32(&buffer_mat, 1, 0) - arm_mat_get_entry_f32(&buffer_mat, 0, 1);
    if ((delta_angle < 1e-3f) && (delta_angle > -1e-3f))
        arm_vec3_scalar_mult_f32(HIL.w_b, -0.5f/dt, HIL.w_b); // small angle approximation
    else
        arm_vec3_scalar_mult_f32(HIL.w_b, -0.5f/sinf(delta_angle)*delta_angle/dt, HIL.w_b);
}

void HILgetIMUData(IMU_AverageData_t *HIL_imu_data) {
    // accelerometer
    float grav_vec_b[3] = { 0.f, 0.f, 0.f };

    // Cancel gravity, since it is not detected by IMUs (except on the launch pad)
    arm_mat_vec_mult_f32(HIL.DCM_bi, gravity_world_vec, grav_vec_b);
    arm_vec3_sub_f32(HIL.a_b, grav_vec_b, HIL_imu_data->accel);

    // gyroscope measures angular velocity in body-frame
    arm_vec3_copy_f32(HIL.w_b, HIL_imu_data->gyro);
}

void HILgetMagnetometerData(LIS3MDL_Data_t *HIL_mag_data) {
    // magnetometer data is simply the Earth's magnetic field expressed in body-frame
    arm_mat_vec_mult_f32(HIL.DCM_bi, magfield, HIL_mag_data->field);
}

void HILgetBarometerData(bmp390_data_t *HIL_bmp_data) {
    // ISA model for h < 11 km
    BaroHeightToPressure(HIL.r_i[2], p0_const, &HIL_bmp_data->pressure);
    HIL_bmp_data->temperature = T0_const + L_const * HIL.r_i[2];
    HIL_bmp_data->height = HIL.r_i[2];
}

void HILgetGPSData(UBX_NAV_PVT *HIL_gps_data) {
    // simply constants
    HIL_gps_data->gpsFix = 3.f;
    HIL_gps_data->numSV = 69.f;
    HIL_gps_data->hAcc = 1600.f;
    HIL_gps_data->vAcc = 3000.f;
    HIL_gps_data->sAcc = 30.f;

    // convert everything to correct format
    HIL_gps_data->height = (int32_t)roundf(HIL.r_i[2]*1e3f);
    HIL_gps_data->velN = (int32_t)roundf(HIL.v_i[1]*1e3f);
    HIL_gps_data->velE = (int32_t)roundf(HIL.v_i[0]*1e3f);
    HIL_gps_data->velD = (int32_t)roundf(-HIL.v_i[2]*1e3f);
}