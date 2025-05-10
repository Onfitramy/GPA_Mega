
#pragma once

#include "arm_math.h"

void arm_mat_set_entry_f32(arm_matrix_instance_f32* mat, int row, int col, float value);
float arm_mat_get_entry_f32(const arm_matrix_instance_f32* mat, int row, int col);
void arm_mat_set_zero_f32(arm_matrix_instance_f32* mat);
void arm_mat_copy_f32(const arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst);
void arm_mat_vec_mult_f32(arm_matrix_instance_f32 *pSrcMat, float32_t *pVec, float32_t *pDst);
void arm_mat_get_column_f32(const arm_matrix_instance_f32 *mat, int col, float32_t *vec);
void arm_mat_set_column_f32(arm_matrix_instance_f32 *mat, int col, float32_t *vec);
void arm_mat_print_f32(arm_matrix_instance_f32 *pMat);

void arm_vec3_add_f32(float32_t *vecA, float32_t *vecB, float32_t *result);
void arm_vec3_sub_f32(float32_t *vecA, float32_t *vecB, float32_t *result);
void arm_vec3_scalar_mult_f32(float32_t *vec, float scalar, float32_t *result);
float arm_vec3_length_f32(float32_t *vec);
float arm_vec3_length_squared_f32(float32_t *vec);
void arm_vec3_copy_f32(float32_t *vecSrc, float32_t *vecDst);
float arm_vec3_dot_product_f32(float32_t *vecA, float32_t *vecB);
void arm_vec3_cross_product_f32(float32_t *vecA, float32_t *vecB, float32_t *result);
void arm_vec3_element_product_f32(float32_t *vecA, float32_t *vecB, float32_t *result);
void arm_vec3_print_f32(float32_t *vec);
void arm_vec3_normalize_f32(float32_t *vec);

void arm_vecN_add_f32(int N, float32_t *vecA, float32_t *vecB, float32_t *result);
void arm_vecN_sub_f32(int N, float32_t *vecA, float32_t *vecB, float32_t *result);
void arm_vecN_scalar_mult_f32(int N, float32_t *vec, float scalar, float32_t *result);
void arm_vecN_print_f32(int N, float32_t *vec);

