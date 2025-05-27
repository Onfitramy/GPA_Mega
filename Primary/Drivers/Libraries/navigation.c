#include "navigation.h"
#include "arm_math.h"

void normalizeAngle(float* angle, float upper_boundary, float lower_boundary) {
    if(*angle > upper_boundary) {
        *angle -= 2*PI;
    } 
    if(*angle < lower_boundary) {
        *angle += 2*PI;
    }
}

void normalizeAngleVector(float* angle_vec, uint8_t pos_top, uint8_t pos_bottom, float upper_boundary, float lower_boundary) {
    for(int i = pos_top; i <= pos_bottom; i++) {
        if( angle_vec[i] > upper_boundary) {
            angle_vec[i] -= 2*PI;
        } 
        if( angle_vec[i] < lower_boundary) {
            angle_vec[i] += 2*PI;
        }
    }
}

void normalizeAnglePair(float angle_lead, float* angle_mod) {
    *angle_mod -= angle_lead;
    if( *angle_mod > PI) {
        *angle_mod -= 2*PI;
    } 
    if( *angle_mod < -PI) {
        *angle_mod += 2*PI;
    }
    *angle_mod += angle_lead;
}

void normalizeAnglePairVector(float* angle_lead_vec, float* angle_mod_vec, uint8_t pos_top, uint8_t pos_bottom) {
    for(int i = pos_top; i <= pos_bottom; i++) {
        angle_mod_vec[i] -= angle_lead_vec[i];
        if( angle_mod_vec[i] > PI) {
            angle_mod_vec[i] -= 2*PI;
        } 
        if( angle_mod_vec[i] < -PI) {
            angle_mod_vec[i] += 2*PI;
        }
            angle_mod_vec[i] += angle_lead_vec[i];
   } 
}