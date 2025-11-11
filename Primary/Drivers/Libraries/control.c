#include "control.h"

float stepper_zero_position;
float stepper_target_position;
float stepper_target_angle_deg;
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

    // TODO: relative angle wrt stepper zero position
    float Ay = stepper_pos;

    float AB_squared = Ay * Ay + POSITION_BX * POSITION_BX;

    float W_term = sqrtf(4 * AB_squared * BE_squared - powf(AB_squared - BE_squared - AE_squared, 2));

    float Ex = ((AB_squared - BE_squared + AE_squared) * POSITION_BX + Ay * W_term) / (2 * AB_squared);
    float Ey = ((AB_squared + BE_squared - AE_squared) * Ay + POSITION_BX * W_term) / (2 * AB_squared);

    float Dx = LENGTH_BD / LENGTH_BE * (Ex - POSITION_BX) + POSITION_BX;
    float Dy = LENGTH_BD / LENGTH_BE * Ey;

    W_term = sqrtf(((POSITION_CX - Dx) * (POSITION_CX - Dx) + (POSITION_CY - Dy) * (POSITION_CY - Dy)) / (F_DIST * F_DIST) - 1);

    *acs_angle_deg = atanf((Dy - POSITION_CY + (Dx - POSITION_CX) * W_term) / ((Dx - POSITION_CX) - (Dy - POSITION_CY) * W_term)) * 180.0f / M_PI;
}

void StepperPositionFromACSAngle(float acs_angle_deg, float *stepper_pos) {
    float AE_squared = LENGTH_AE * LENGTH_AE;
    float BD_squared = LENGTH_BD * LENGTH_BD;

    float acs_angle_rad = acs_angle_deg * M_PI / 180.0f;

    float a_distance = (POSITION_CX - POSITION_BX) * cosf(acs_angle_rad) + POSITION_CY * sinf(acs_angle_rad) + F_DIST;

    float Dx = POSITION_BX + a_distance * cosf(acs_angle_rad) - sqrtf(BD_squared - a_distance * a_distance) * sinf(acs_angle_rad);
    float Dy = a_distance * sinf(acs_angle_rad) + sqrtf(BD_squared - a_distance * a_distance) * cosf(acs_angle_rad);

    float Ex = (LENGTH_BE / LENGTH_BD) * (Dx - POSITION_BX) + POSITION_BX;
    float Ey = (LENGTH_BE / LENGTH_BD) * Dy;

    float Ay = Ey + sqrtf(AE_squared - Ex * Ex);

    // TODO: relative angle wrt stepper zero position
    *stepper_pos = Ay;
}

void StepperAngleFromPosition(float stepper_pos, float stepper_zero_pos, float *stepper_angle_deg) {
    float stepper_pos_diff = stepper_zero_pos - stepper_pos;
    *stepper_angle_deg = stepper_pos_diff / ROD_SLOPE * 360.f;
}