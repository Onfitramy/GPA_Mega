#include "control.h"

// send TVC command | angles in deg
void TVC_command(float angle_x, float angle_z, float twist_y, float angle_constrain, float twist_constrain) {
    twist_y = fconstrain(twist_y, -twist_constrain, twist_constrain);
    SERVO_MoveToAngle(PX_SERVO, (float)PX_ZERO + fconstrain(twist_y + angle_x, -angle_constrain, angle_constrain));
    SERVO_MoveToAngle(NX_SERVO, (float)NX_ZERO + fconstrain(twist_y - angle_x, -angle_constrain, angle_constrain));
    SERVO_MoveToAngle(PZ_SERVO, (float)PZ_ZERO + fconstrain(twist_y - angle_z, -angle_constrain, angle_constrain));
    SERVO_MoveToAngle(NZ_SERVO, (float)NZ_ZERO + fconstrain(twist_y + angle_z, -angle_constrain, angle_constrain));
}

// send thrust command | thrust level in %
void Thrust_command(float thrust, float difference) {
    thrust = fconstrain(thrust, 0, 100);
    SERVO_MoveToAngle(PY_MOTOR, (thrust+0.5*difference)*1.8);
    SERVO_MoveToAngle(NY_MOTOR, (thrust-0.5*difference)*1.8);
}

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