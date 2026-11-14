#include "HIL.h"
#include "navigation.h"
#include "control.h"

float magfield[3] = {0.0069f, 0.1973f, -0.4431f};

float HIL_DCM_bi_data[9] = { 0 };
arm_matrix_instance_f32 HIL_DCM_bi = {3, 3, HIL_DCM_bi_data};
float HIL_DCM_ib_data[9] = { 0 };
arm_matrix_instance_f32 HIL_DCM_ib = {3, 3, HIL_DCM_ib_data};

HIL_t HIL;

void HILInit() {
    HIL.DCM_bi = &HIL_DCM_bi;
    HIL.DCM_ib = &HIL_DCM_ib;
    HIL.t = -IGNITION_TIME;
    for (int i = 0; i < 3; i++) {
        HIL.r_i[i] = 0.f;
        HIL.v_i[i] = 0.f;
        HIL.v_b[i] = 0.f;
        HIL.w_b[i] = 0.f;
        HIL.a_b[i] = 0.f;
        HIL.a_i[i] = 0.f;
    }
    HIL.euler[0] = 0.5f*PI;
    HIL.euler[1] = 0.f;
    HIL.euler[2] = -PI;
    RotationMatrixFromEuler(HIL.euler[0], HIL.euler[1], HIL.euler[2], HIL.DCM_bi);
    arm_mat_trans_f32(HIL.DCM_bi, HIL.DCM_ib);
}

void HILupdateStates(float dt) {
    HIL.t += dt;

    float F_b[3] = { 0 };
    if (HIL.t < 0) {
        // pre-launch
        HIL.m = WET_MASS;
        
    } else if (HIL.t < MOTOR_BURN_TIME) {
        // burn
        HIL.m = WET_MASS - PROPELLANT_MASS / MOTOR_BURN_TIME * HIL.t;

        // compute forces in body frame
        arm_mat_vec_mult_f32(HIL.DCM_bi, gravity_world_vec, F_b);
        arm_vec3_scalar_mult_f32(F_b, HIL.m, F_b);
        float velo_abs = arm_vec3_length_f32(HIL.v_i);
        float CD = ComputeAirbrakeDrag(velo_abs, acs_target_angle_deg);
        float rho = CalculateAirDensity(HIL.r_i[2]);
        F_b[1] -= 0.5*rho*velo_abs*velo_abs*AREF*CD;
        F_b[1] += MOTOR_THRUST;
        if (HIL.r_i[2] < RAIL_LENGTH)
            F_b[0] = F_b[2] = 0.f;
        
    } else if (HIL.r_i[2] > 0) {
        // coast
        HIL.m = WET_MASS - PROPELLANT_MASS;

        // compute forces in body frame
        arm_mat_vec_mult_f32(HIL.DCM_bi, gravity_world_vec, F_b);
        arm_vec3_scalar_mult_f32(F_b, HIL.m, F_b);
        float velo_abs = arm_vec3_length_f32(HIL.v_i);
        float CD = ComputeAirbrakeDrag(velo_abs, acs_target_angle_deg);
        float rho = CalculateAirDensity(HIL.r_i[2]);
        F_b[1] -= 0.5f*rho*velo_abs*velo_abs*AREF*CD;

        if (HIL.v_i[2] < 0) {
            float cd_s;
            if (HIL.r_i[2] > MAIN_MIN_DEPLOY_HEIGHT) {
                cd_s = 0.467f;
            } else {
                cd_s = 5.77f;
            }
            F_b[1] -= 0.5f * rho * velo_abs * velo_abs * cd_s;
        }
    } else {
        // landed
        HIL.v_i[0] = 0.f;
        HIL.v_i[1] = 0.f;
        HIL.v_i[2] = 0.f;
        HIL.a_i[0] = 0.f;
        HIL.a_i[1] = 0.f;
        HIL.a_i[2] = 0.f;
    }
    // rigid-body kinetics
    arm_vec3_scalar_mult_f32(F_b, 1/HIL.m, HIL.a_b);
    arm_mat_vec_mult_f32(HIL.DCM_ib, HIL.a_b, HIL.a_i);
    for (int i = 0; i < 3; i++) {
        float a_old = HIL.a_i[i];
        float v_old = HIL.v_i[i];
        HIL.v_i[i] += a_old*dt;
        HIL.r_i[i] += v_old*dt;
    }
    arm_mat_vec_mult_f32(HIL.DCM_bi, HIL.v_i, HIL.v_b);

    // update orientation
    //if ((HIL.v_i[0] != 0) || (HIL.v_i[1] != 0)) {
    if(1) {
        float HIL_DCMo_bi_data[9] = { 0 };
        arm_matrix_instance_f32 HIL_DCMo_bi = {3, 3, HIL_DCMo_bi_data};
        arm_mat_copy_f32(HIL.DCM_bi, &HIL_DCMo_bi);

        float bx_axis_i[3];
        bx_axis_i[0] = HIL.v_i[1];
        bx_axis_i[1] = -HIL.v_i[0];
        bx_axis_i[2] = 0.f;
        arm_vec3_normalize_f32(bx_axis_i);
        float by_axis_i[3];
        arm_vec3_copy_f32(HIL.v_i, by_axis_i);
        arm_vec3_normalize_f32(by_axis_i);
        float bz_axis_i[3];
        arm_vec3_cross_product_f32(bx_axis_i, by_axis_i, bz_axis_i);
        arm_mat_set_column_f32(HIL.DCM_ib,0,bx_axis_i);
        arm_mat_set_column_f32(HIL.DCM_ib,1,by_axis_i);
        arm_mat_set_column_f32(HIL.DCM_ib,2,bz_axis_i);
        arm_mat_trans_f32(HIL.DCM_ib, HIL.DCM_bi);
        EulerFromRotationMatrix(HIL.DCM_bi, HIL.euler);

        float HIL_DCM_bbn_data[9] = { 0 };
        arm_matrix_instance_f32 HIL_DCM_bbn = {3, 3, HIL_DCM_bbn_data};
        arm_mat_mult_f32(&HIL_DCMo_bi, HIL.DCM_ib, &HIL_DCM_bbn);
        float bbn_angle = acosf((arm_mat_trace_f32(&HIL_DCM_bbn) - 1.f) / 2.f);
        HIL.w_b[0] = arm_mat_get_entry_f32(&HIL_DCM_bbn,2,1)-arm_mat_get_entry_f32(&HIL_DCM_bbn,1,2);
        HIL.w_b[1] = arm_mat_get_entry_f32(&HIL_DCM_bbn,0,2)-arm_mat_get_entry_f32(&HIL_DCM_bbn,2,0);
        HIL.w_b[2] = arm_mat_get_entry_f32(&HIL_DCM_bbn,1,0)-arm_mat_get_entry_f32(&HIL_DCM_bbn,0,1);
        if ((bbn_angle < M_PI/180.f) && (bbn_angle > -M_PI/180.f))
            arm_vec3_scalar_mult_f32(HIL.w_b, 0.5f/dt, HIL.w_b);
        else
            arm_vec3_scalar_mult_f32(HIL.w_b, 0.5f/sinf(bbn_angle)*bbn_angle/dt, HIL.w_b);
    }
    
}

void HILgetIMUData(IMU_AverageData_t *HIL_imu_data) {
    // accelerometer
    float grav_vec_b[3] = { 0.f, 0.f, 0.f };
    float grav_vec_i[3];
    // Cancel gravity, since it is not detected by IMUs (except on the launch pad)
    arm_vec3_scalar_mult_f32(gravity_world_vec, -1.f, grav_vec_i);
    arm_mat_vec_mult_f32(HIL.DCM_bi, grav_vec_i, grav_vec_b);
    arm_vec3_add_f32(HIL.a_b,grav_vec_b,HIL_imu_data->accel);

    // gyroscope
    arm_vec3_copy_f32(HIL.w_b, HIL_imu_data->gyro);
}

void HILgetMagnetometerData(LIS3MDL_Data_t *HIL_mag_data) {
    arm_mat_vec_mult_f32(HIL.DCM_bi, magfield, HIL_mag_data->field);
}

void HILgetBarometerData(bmp390_data_t *HIL_bmp_data) {
    BaroHeightToPressure(HIL.r_i[2],p0_const, &HIL_bmp_data->pressure);
    HIL_bmp_data->temperature = T0_const + L_const * HIL.r_i[2];
    HIL_bmp_data->height = HIL.r_i[2];
}

void HILgetGPSData(UBX_NAV_PVT *HIL_gps_data) {
    HIL_gps_data->gpsFix = 3.f;
    HIL_gps_data->numSV = 69.f;
    HIL_gps_data->hAcc = 1600.f;
    HIL_gps_data->vAcc = 3000.f;
    HIL_gps_data->sAcc = 30.f;
    HIL_gps_data->height = (int32_t)roundf(HIL.r_i[2]);
    HIL_gps_data->velN = (int32_t)roundf(HIL.v_i[1]);
    HIL_gps_data->velE = (int32_t)roundf(HIL.v_i[0]);
    HIL_gps_data->velD = (int32_t)roundf(-HIL.v_i[2]);
}