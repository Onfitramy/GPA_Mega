#include "control.h"

void PID_Init(PID_Instance *PID, uint8_t dimension, float Kp, float Ki, float Kd) {
    PID->dim = dimension;
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
}

// PID __ phi = x | theta = z | psi = y
void PID_vec3(PID_Instance *PID, float *euler_set, float *euler_deg, float *PID_out, bool I_reset_condition) {

}

float fconstrain(float variable, float min, float max) {
    if(variable > max) {
        return max;
    } else if(variable < min) {
        return min;
    } else {
        return variable;
    }
}