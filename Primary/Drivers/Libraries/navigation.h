#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "stm32h7xx_hal.h"
#include "stdbool.h"

void normalizeAngle(float* angle, float upper_boundary, float lower_boundary);
void normalizeAngleVector(float* angle_vec, uint8_t pos_top, uint8_t pos_bottom, float upper_boundary, float lower_boundary);

void normalizeAnglePair(float angle_lead, float* angle_mod);
void normalizeAnglePairVector(float* angle_lead_vec, float* angle_mod_vec, uint8_t pos_top, uint8_t pos_bottom);

#endif // NAVIGATION_H