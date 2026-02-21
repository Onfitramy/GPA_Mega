#include "control.h"

double cACS[6] = {0.000083245890724,
                  0.037473638753597,
                  0.789768302306386,
                 -0.051091578657429,
                 -0.001630768977842,
                  0.000369857971148};

mpc_t a_mpc;

float A_data[2*2] = { 0 };
arm_matrix_instance_f32 A_mat = {2, 2, A_data};

float B_vec[2] = { 0 };

float r_vec[2] = { 0 };

float Ca_data[2] = { 1, 1 };
arm_matrix_instance_f32 Ca_mat = {1, 2, Ca_data};

float D_data[PREDICTION_HORIZON*PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 D_mat = {PREDICTION_HORIZON, PREDICTION_HORIZON, D_data};

float d_data[PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 d_vec = {1, PREDICTION_HORIZON, d_data};

float E_data[2*PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 E_mat = {2, PREDICTION_HORIZON, E_data};

float R_du_data[PREDICTION_HORIZON*PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 R_du_mat = {PREDICTION_HORIZON, PREDICTION_HORIZON, R_du_data};

float R_u_data[PREDICTION_HORIZON*PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 R_u_mat = {PREDICTION_HORIZON, PREDICTION_HORIZON, R_u_data};

float M1_data[PREDICTION_HORIZON*PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 M1_mat = {PREDICTION_HORIZON, PREDICTION_HORIZON, M1_data};

float M2_data[PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 M2_vec = {1, PREDICTION_HORIZON, M2_data};

float CaT_data[2] = { 0 };
arm_matrix_instance_f32 CaT_mat = {2, 1, CaT_data};

float ET_data[PREDICTION_HORIZON*2] = { 0 };
arm_matrix_instance_f32 ET_mat = {PREDICTION_HORIZON, 2, ET_data};

float ETCaT_data[PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 ETCaT_mat = {PREDICTION_HORIZON, 1, ETCaT_data};

float CaE_data[PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 CaE_mat = {1, PREDICTION_HORIZON, CaE_data};

/* Cost Function P Matrix */
float P_data[PREDICTION_HORIZON*PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 P_mat = {PREDICTION_HORIZON, PREDICTION_HORIZON, P_data};

/* Cost Function q vector */
float q_data[PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 q_vec = {1, PREDICTION_HORIZON, q_data};

/* Inequality Constraint G Matrix in CCS format */
float G_data[NUM_INEQUALITY_CONSTRAINTS*PREDICTION_HORIZON] = { 0 };
arm_matrix_instance_f32 G_mat = {NUM_INEQUALITY_CONSTRAINTS, PREDICTION_HORIZON, G_data};

/* Inequality Constraint h vector */
qp_real h_vec[NUM_INEQUALITY_CONSTRAINTS] = { 0 };

float u_star[PREDICTION_HORIZON] = { 0 };
float x_star[2] = { 0 };

// temporary cast fix
qp_real P_dataQP[PREDICTION_HORIZON*PREDICTION_HORIZON];
qp_real q_dataQP[PREDICTION_HORIZON];
qp_real G_dataQP[NUM_INEQUALITY_CONSTRAINTS*PREDICTION_HORIZON];

float ComputeAirbrakeDrag(float vel_abs, float gamma) {
    float gam = gamma;
    if (gamma > ACS_ANGLE_MAX_DEG)
        gam = ACS_ANGLE_MAX_DEG;
    else if (gamma < ACS_ANGLE_MIN_DEG)
        gam = ACS_ANGLE_MIN_DEG;

    float vel = vel_abs;
    if (vel_abs < 0)
        vel = -vel;
    if (vel_abs > 300)
        vel = 300;

    float M = vel / a0_const;

    return (float)cACS[0]*gam*gam+cACS[1]*gam+cACS[2]+cACS[3]*M*M+cACS[4]*M*gam+cACS[5]*M*M*gam*gam;
}

float predictApogeeFromGamma(float height, float *velocity, float Aref, float m, float gamma, float t_max, float dt, float *t_apogee) {
    float p_ver = (float)height;
    float v_ver = (float)velocity[2];
    float v_hor = sqrtf(velocity[0]*velocity[0]+velocity[1]*velocity[1]);

    float v_abs;
    float rho;
    float CD;
    float F_hor, F_ver;
    float a_hor, a_ver;

    int8_t dir = 1;
    if (v_ver < 0) {
        dir = -1;
        v_ver = -v_ver;
    } else if (v_ver == 0) {
        *t_apogee = 0;
        return p_ver;
    }

    uint32_t steps = t_max / dt;
    float t_sim = 0;

    for (uint32_t i = 0; i < steps; i++) {
        t_sim += dt * dir;

        // calculate absolute velocity, air density and drag coefficient
        v_abs = sqrt(v_ver*v_ver+v_hor*v_hor);
        rho = CalculateAirDensity(p_ver);
        CD = ComputeAirbrakeDrag(v_abs, gamma);

        // update horizontal velocity
        F_hor = -0.5 * Aref * CD * rho * v_abs * v_hor * dir;
        a_hor = F_hor / m;
        v_hor += a_hor * dt;

        // update vertical velocity and position
        F_ver = -0.5 * Aref * CD * rho * v_abs * v_ver * dir - m * g0_const;
        a_ver = F_ver / m;
        v_ver += a_ver * dt;
        p_ver += v_ver * dt + 0.5 * a_ver * dt * dt;

        // stop criteria
        if (v_ver <= 0) {
            t_sim = t_sim - v_ver / a_ver;
            p_ver = p_ver - v_ver*v_ver/(2*a_ver);
            break;
        }
    }

    *t_apogee = t_sim;
    return p_ver;
}

void predictFutureStateGamma(float height, float *velocity, float Aref, float m, float gamma, float t_max, float dt, float *h_pred, float *v_pred) {
    float p_ver = (float)height;
    float v_ver = (float)velocity[2];
    float v_hor = sqrt(velocity[0]*velocity[0]+velocity[1]*velocity[1]);

    float v_abs;
    float rho;
    float CD;
    float F_hor, F_ver;
    float a_hor, a_ver;

    uint32_t steps = t_max / dt;

    for (uint32_t i = 0; i < steps; i++) {
        // calculate absolute velocity, air density and drag coefficient
        v_abs = sqrt(v_ver*v_ver+v_hor*v_hor);
        rho = CalculateAirDensity(p_ver);
        CD = ComputeAirbrakeDrag(v_abs, gamma);

        // update horizontal velocity
        F_hor = -0.5 * Aref * CD * rho * v_abs * v_hor;
        a_hor = F_hor / m;
        v_hor += a_hor * dt;

        // update vertical velocity and position
        F_ver = -0.5 * Aref * CD * rho * v_abs * v_ver - m * g0_const;
        a_ver = F_ver / m;
        v_ver += a_ver * dt;
        p_ver += v_ver * dt + 0.5 * a_ver * dt * dt;
    }

    *h_pred = p_ver;
    *v_pred = v_ver;
}

void MPCInit(mpc_t *mpc, uint8_t pred_horz, uint8_t ineq_constr_num, float dt, float *ustar, float *xstar) {
    // store settings
    mpc->N = pred_horz;
    mpc->n = ineq_constr_num;
    mpc->dt = dt;

    // link vectors
    mpc->u_nom = GAMMA_NOMINAL;

    mpc->ustar = ustar;
    mpc->xstar = xstar;

    // fill vector of inequality constraints
    for(int i = 0; i < (int)(0.5*mpc->n); i++) {
        h_vec[i] = (qp_real)ACS_ANGLE_MAX_DEG;
    }
    for(int i = (int)(0.5*mpc->n); i < mpc->n; i++) {
        h_vec[i] = (qp_real)ACS_ANGLE_MIN_DEG;
    }

    // fill matrix of inequality constraints
    arm_mat_fill_diag_f32(&G_mat, 0, 0, 1);
    arm_mat_fill_diag_f32(&G_mat, (int)(0.5*mpc->n), 0, -1);

    // fill D matrix
    arm_mat_fill_diag_f32(&D_mat, 0, 0, 1);
    arm_mat_fill_diag_f32(&D_mat, 1, 0, -1);

    // fill R_du matrix
    arm_mat_fill_diag_f32(&R_du_mat, 0, 0, 1);

    // calculate second P and q terms
    float DT_data[PREDICTION_HORIZON*PREDICTION_HORIZON] = { 0 };
    arm_matrix_instance_f32 DT_mat = {PREDICTION_HORIZON, PREDICTION_HORIZON, DT_data};
    arm_mat_trans_f32(&D_mat, &DT_mat);
    arm_mat_mult_f32(&R_du_mat, &D_mat, &P_mat);
    arm_mat_scale_f32(&P_mat, MPC_W_DU, &R_du_mat);
    arm_mat_mult_f32(&DT_mat, &R_du_mat, &M1_mat);
}

float runMPC(mpc_t *mpc, float height, float *velocity) {

    float m = DRYMASS;
    float buffer[2];

    // initial states
    float x_0[2];
    x_0[0] = height;
    x_0[1] = velocity[2];

    float v_input[3];
    v_input[0] = velocity[0];
    v_input[1] = velocity[1];
    v_input[2] = mpc->xstar[1];
    float x_N0[2];
    predictFutureStateGamma(mpc->xstar[0], v_input, AREF, m, mpc->u_nom, mpc->dt, mpc->dt, &x_N0[0], &x_N0[1]);

    // model operating point
    float h_op = 0.5 * ((float)x_0[0] + x_N0[0]);
    float v_op = 0.5 * ((float)x_0[1] + x_N0[1]);
    float CD_op = ComputeAirbrakeDrag((float)v_op, (float)mpc->ustar[0]);   // drag only computed from vertical velocity component! Might be worth looking into...
    float rho = CalculateAirDensity(h_op);                                 // in theory, this does not compensate the launch site altitude

    /* calculate linearization constants */
    float a_op = -g0_const - 0.5*rho * v_op * v_op * AREF * CD_op / m;
    float a_v = -rho*AREF*v_op/m*(v_op*v_op*(cACS[3]+cACS[5]*mpc->ustar[0]*mpc->ustar[0])/(a0_const*a0_const)+0.5*v_op*cACS[4]*mpc->ustar[0]/a0_const+CD_op);
    float a_g = -0.5*rho*v_op*v_op*AREF/m*(2*mpc->ustar[0]*(cACS[0]+cACS[5]*v_op*v_op/(a0_const*a0_const))+cACS[1]+cACS[4]*v_op/a0_const);
    float a_h = rho*v_op*v_op*AREF*CD_op*g0_const/(2*m*T0_const*R_const)*pow(1+L_const*h_op/T0_const, -g0_const/(R_const*L_const)-2);

    float a_c = a_op - a_v * v_op - a_g * mpc->ustar[0] - a_h * h_op;

    /* Fill A, B and r matrices and vectors */
    arm_mat_set_entry_f32(&A_mat, 0, 0, 1 + 0.5 * a_h * mpc->dt * mpc->dt);
    arm_mat_set_entry_f32(&A_mat, 0, 1, mpc->dt + 0.5 * a_v * mpc->dt * mpc->dt);
    arm_mat_set_entry_f32(&A_mat, 1, 0, a_h * mpc->dt);
    arm_mat_set_entry_f32(&A_mat, 1, 1, 1 + a_v * mpc->dt);

    B_vec[0] = 0.5 * a_g * mpc->dt * mpc->dt;
    B_vec[1] = a_g * mpc->dt;

    r_vec[0] = 0.5 * a_c * mpc->dt * mpc->dt;
    r_vec[1] = a_c * mpc->dt;

    // update d vector
    arm_mat_set_entry_f32(&d_vec, 0, 0, mpc->ustar[0]);

    /* calculate E */
    // handle columns 0 through N-2
    for (int i = 0; i < mpc->N - 1; i++) {
        int A_exponent = mpc->N - 1 - i;
        float column[2];

        arm_mat_vec_mult_f32(&A_mat, B_vec, buffer);
        for (int j = 1; j < A_exponent; j++) {
            if (j % 2) {
                arm_mat_vec_mult_f32(&A_mat, buffer, column);
            } else {
                arm_mat_vec_mult_f32(&A_mat, column, buffer);
            }
        }
        if (A_exponent % 2) {
            arm_mat_set_column_f32(&E_mat, i, buffer);
        } else {
            arm_mat_set_column_f32(&E_mat, i, column);
        }
    }
    // handle last column
    arm_mat_set_column_f32(&E_mat, mpc->N-1, B_vec);

    /* time between end of prediction horizon and apogee */
    v_input[2] = x_N0[1];
    float t_nom;
    float h_nom = predictApogeeFromGamma(x_N0[0], v_input, AREF, m, mpc->u_nom, 20, 0.1, &t_nom);

    float a_t = 0.5 * x_N0[1]*x_N0[1] / (h_nom - x_N0[0]);
    float h_ref = 0.5 * x_N0[1]*x_N0[1] / a_t + H_TARGET;

    // define apogee gradient Ca
    arm_mat_set_entry_f32(&Ca_mat, 0, 1, x_N0[1]/a_t);

    /* compute x_N_free */
    float x_N_free[2];
    x_N_free[0] = x_0[0];
    x_N_free[1] = x_0[1];
    for (int i = 0; i < mpc->N; i++) {
        arm_mat_vec_mult_f32(&A_mat, x_N_free, buffer);
        arm_vecN_add_f32(2, buffer, r_vec, x_N_free);
    }

    /* compute P */
    // first term
    arm_mat_trans_f32(&Ca_mat, &CaT_mat);
    arm_mat_scale_f32(&CaT_mat, MPC_W_A, &CaT_mat);
    arm_mat_trans_f32(&E_mat, &ET_mat);

    arm_mat_mult_f32(&ET_mat, &CaT_mat, &ETCaT_mat);
    arm_mat_mult_f32(&Ca_mat, &E_mat, &CaE_mat);
    arm_mat_mult_f32(&ETCaT_mat, &CaE_mat, &P_mat);
    arm_mat_add_f32(&P_mat, &M1_mat, &P_mat); // add second term

    float w_u = MPC_W_U * (t_nom + mpc->N*mpc->dt) / MPC_TCOAST; // third term
    for (int i = 0; i < mpc->N; i++) {
        arm_mat_set_entry_f32(&R_u_mat, i, i, (MPC_ALPHA0+(1-MPC_ALPHA0)/(mpc->N-1)*i)*w_u);
    }
    arm_mat_add_f32(&P_mat, &R_u_mat, &P_mat); // add third term

    /* compute q^T */
    arm_mat_mult_f32(&d_vec, &R_du_mat, &M2_vec);
    arm_mat_scale_f32(&CaE_mat, MPC_W_A*(Ca_data[0]*x_N_free[0]+Ca_data[1]*x_N_free[1]-h_ref), &q_vec);
    arm_mat_sub_f32(&q_vec, &M2_vec, &q_vec);

    /* solve QP */
    QP *mpcQP;

    // cast into qp_real (temporary fix)
    for (int i = 0; i < PREDICTION_HORIZON*PREDICTION_HORIZON; i++) {
        P_dataQP[i] = (qp_real)P_data[i];
    }
    for (int i = 0; i < PREDICTION_HORIZON; i++) {
        q_dataQP[i] = (qp_real)q_data[i];
    }
    for (int i = 0; i < NUM_INEQUALITY_CONSTRAINTS*PREDICTION_HORIZON; i++) {
        G_dataQP[i] = (qp_real)G_data[i];
    }

    mpcQP = QP_SETUP_dense(mpc->N, mpc->n, 0, P_dataQP, NULL, G_dataQP, q_dataQP, h_vec, NULL, NULL, ROW_MAJOR_ORDERING);

	mpc->ExitCode = (uint8_t)QP_SOLVE(mpcQP);

    mpc->tsetup = (float)mpcQP->stats->tsetup;
    mpc->tsolve = (float)mpcQP->stats->tsolve;
    mpc->iterations = (uint8_t)mpcQP->stats->IterationCount;

    QP_CLEANUP_dense(mpcQP);

    if ((mpc->ExitCode == QP_OPTIMAL) || (mpc->ExitCode == QP_MAXIT)) {
        // accept solution
        for (int i = 0; i < mpc->N; i++) {
            mpc->ustar[i] = (float)mpcQP->x[i];
        }
    } else {
        // use previous valid solution
        for (int i = 0; i < (mpc->N-1); i++) {
            mpc->ustar[i] = mpc->ustar[i+1];
        }
        mpc->ustar[mpc->N-1] = mpc->u_nom;
    }

    /* propagate x star */
    float x_sum[2] = {0};
    arm_mat_vec_mult_f32(&E_mat, mpc->ustar, x_sum);
    mpc->xstar[0] = x_N_free[0] + x_sum[0];
    mpc->xstar[1] = x_N_free[1] + x_sum[1];

    if (t_nom < -1.5) {
        return 0;
    }
    return mpc->ustar[0];
}

/* --- Airbrake Kinematics --- */

float stepper_zero_position;
float stepper_target_position;
float stepper_target_angle_deg;
float stepper_neutral_angle = 100.f;
float acs_target_angle_deg;

void CircleIntersectionPoints(float x1, float y1, float r1, float x2, float y2, float r2,
                              float *xi1, float *yi1, float *xi2, float *yi2, int *num_intersections)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float d = sqrtf(dx * dx + dy * dy);

    // No intersection
    if (d > (r1 + r2) || d < fabsf(r1 - r2)) {
        *num_intersections = 0;
        return;
    }

    // Single intersection (tangent)
    if (d == 0 && r1 == r2) {
        *num_intersections = -1; // Infinite intersections
        return;
    }

    float a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    float h = sqrtf(r1 * r1 - a * a);
    float xm = x1 + a * (dx) / d;
    float ym = y1 + a * (dy) / d;

    *xi1 = xm + h * (dy) / d;
    *yi1 = ym - h * (dx) / d;

    *xi2 = xm - h * (dy) / d;
    *yi2 = ym + h * (dx) / d;

    if (h == 0) {
        *num_intersections = 1; // Tangent
    } else {
        *num_intersections = 2; // Two intersections
    }
}

void ACSAngleFromStepperPosition(float stepper_pos, float *acs_angle_deg) {
    float AE_squared = LENGTH_AE * LENGTH_AE;
    float BE_squared = LENGTH_BE * LENGTH_BE;

    float Ay = stepper_pos;

    float AB_squared = Ay * Ay + POSITION_BX * POSITION_BX;

    float W_term = sqrtf(4 * AE_squared * BE_squared - powf(AB_squared - BE_squared - AE_squared, 2));

    float Ex = ((AB_squared - BE_squared + AE_squared) * POSITION_BX + Ay * W_term) / (2 * AB_squared);
    float Ey = ((AB_squared + BE_squared - AE_squared) * Ay + POSITION_BX * W_term) / (2 * AB_squared);

    float Dx = LENGTH_BD / LENGTH_BE * (Ex - POSITION_BX) + POSITION_BX;
    float Dy = LENGTH_BD / LENGTH_BE * Ey;

    W_term = sqrtf(((POSITION_CX - Dx) * (POSITION_CX - Dx) + (POSITION_CY - Dy) * (POSITION_CY - Dy)) / (F_DIST * F_DIST) - 1);

    *acs_angle_deg = atanf((Dy - POSITION_CY + (Dx - POSITION_CX) * W_term) / ((Dx - POSITION_CX) - (Dy - POSITION_CY) * W_term)) * 180.0f / M_PI;
}

void StepperPositionFromACSAngle(float acs_angle_deg, float *stepper_pos) {
    if (acs_angle_deg > ACS_ANGLE_MAX_DEG) {
        acs_angle_deg = ACS_ANGLE_MAX_DEG;
    } else if (acs_angle_deg < ACS_ANGLE_MIN_DEG) {
        acs_angle_deg = ACS_ANGLE_MIN_DEG;
    }

    float AE_squared = LENGTH_AE * LENGTH_AE;
    float BD_squared = LENGTH_BD * LENGTH_BD;

    float acs_angle_rad = acs_angle_deg * M_PI / 180.0f;

    float a_distance = (POSITION_CX - POSITION_BX) * cosf(acs_angle_rad) + POSITION_CY * sinf(acs_angle_rad) + F_DIST;

    float Dx = POSITION_BX + a_distance * cosf(acs_angle_rad) - sqrtf(BD_squared - a_distance * a_distance) * sinf(acs_angle_rad);
    float Dy = a_distance * sinf(acs_angle_rad) + sqrtf(BD_squared - a_distance * a_distance) * cosf(acs_angle_rad);

    float Ex = (LENGTH_BE / LENGTH_BD) * (Dx - POSITION_BX) + POSITION_BX;
    float Ey = (LENGTH_BE / LENGTH_BD) * Dy;

    float Ay = Ey + sqrtf(AE_squared - Ex * Ex);

    *stepper_pos = Ay;
}

void StepperAngleFromPosition(float stepper_pos, float stepper_zero_pos, float *stepper_angle_deg) {
    float stepper_pos_diff = stepper_zero_pos - stepper_pos;
    *stepper_angle_deg = stepper_pos_diff / ROD_SLOPE * 360.f;
}