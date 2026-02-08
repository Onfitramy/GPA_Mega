#ifndef CONTROL_H
#define CONTROL_H

#include <math.h>
#include "qpSWIFT.h"
#include "main.h"
#include "navigation.h"

/* --- Model Predictive Controller --- */
#define H_TARGET 2300.f
#define PREDICTION_HORIZON 20
#define NUM_INEQUALITY_CONSTRAINTS 40
#define GAMMA_NOMINAL 25.f
#define AREF 0.01f
#define DRYMASS 15.f
#define MPC_DELTA_T 0.1f

#define MPC_W_A     1.f
#define MPC_W_DU    1e-5f
#define MPC_W_U     5e-5f
#define MPC_ALPHA0  0.3f
#define MPC_TCOAST  20.f

typedef struct {
    uint8_t N;
    uint8_t n;
    float dt;

    float u_nom;

    float *ustar;
    float *xstar;
} mpc_t;

extern mpc_t a_mpc;

extern float u_star[PREDICTION_HORIZON];
extern float x_star[2];

void MPCInit(mpc_t *mpc, uint8_t pred_horz, uint8_t ineq_constr_num, float delta_t, float *ustar, float *xstar);
float runMPC(mpc_t mpc, float height, float *velocity);
float predictApogeeFromGamma(float height, float *velocity, float Aref, float m, float gamma, float t_max, float delta_t, float *t_apogee);
void predictFutureStateGamma(float height, float *velocity, float Aref, float m, float gamma, float t_max, float delta_t, float *h_pred, float *v_pred);

float ComputeAirbrakeDrag(float vel_abs, float gamma);

/* --- Airbrake kinematics --- */
#define ACS_ANGLE_MAX_DEG 50.f
#define ACS_ANGLE_MIN_DEG 0.f

#define LENGTH_AE 45.0f
#define LENGTH_BE 50.0f
#define LENGTH_BD 140.0f

#define POSITION_BX -2.0f
#define POSITION_CX 18.0f
#define POSITION_CY 155.6f

#define F_DIST 6.0f

#define ROD_SLOPE 8.0f

extern float stepper_zero_position;
extern float stepper_target_position;
extern float stepper_target_angle_deg;
extern float stepper_neutral_angle;
extern float acs_target_angle_deg;

void CircleIntersectionPoints(float x1, float y1, float r1, float x2, float y2, float r2,
                              float *xi1, float *yi1, float *xi2, float *yi2, int *num_intersections);

void ACSAngleFromStepperPosition(float stepper_pos, float *acs_angle_deg);
void StepperPositionFromACSAngle(float acs_angle_deg, float *stepper_pos);
void StepperAngleFromPosition(float stepper_pos, float stepper_zero_pos, float *stepper_angle_deg);


#endif // CONTROL_H