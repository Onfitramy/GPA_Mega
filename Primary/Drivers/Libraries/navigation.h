#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "stm32h7xx_hal.h"
#include "stdbool.h"

void normalizeAngle(float* angle, float upper_boundary, float lower_boundary);
void normalizeAngleVector(float* angle_vec, uint8_t pos_top, uint8_t pos_bottom, float upper_boundary, float lower_boundary);

void normalizeAnglePair(float angle_lead, float* angle_mod);
void normalizeAnglePairVector(float* angle_lead_vec, float* angle_mod_vec, uint8_t pos_top, uint8_t pos_bottom);

void UBLOXtoWGS84(int32_t lat_e7, int32_t lon_e7, int32_t height_e3, double* WGS84_vec);
void WGS84toECEF(double* WGS84, double* ECEF_vec);
void ECEFtoENU(double* WGS84_ref, double* ECEF_ref, double* ECEF, double* ENU);

void OrientationFix(float* accel_vec, float* mag_vec, float* output_angles);

#endif // NAVIGATION_H