#ifndef CONTROL_H
#define CONTROL_H

#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "armMathAddon.h"
#include "SERVO.h"
#include "navigation.h"

typedef struct {
    uint8_t dim;

    float Kp;
    float Ki;
    float Kd;
} PID_Instance;

void TVC_command(float angle_x, float angle_z, float twist_y, float angle_constrain, float twist_constrain);

void Thrust_command(float thrust, float difference);

void PID_vec3(PID_Instance *PID, float *euler_set, float *euler_deg, float *PID_out, bool I_reset_condition);

float fconstrain(float variable, float min, float max);

#endif // CONTROL_H