#include "armMathAddon.h"
#include "math.h"
#include "stdio.h"

void arm_mat_set_entry_f32(arm_matrix_instance_f32 *mat, int row, int col, float value) {
  mat->pData[mat->numCols * row + col] = value;
}

void arm_mat_set_zero_f32(arm_matrix_instance_f32 *mat) {
  memset(mat->pData, 0, mat->numCols * mat->numRows * sizeof(float));
}

float arm_mat_get_entry_f32(const arm_matrix_instance_f32 *mat, int row, int col) {
  return mat->pData[mat->numCols * row + col];
}

void arm_mat_copy_f32(const arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst) {
  memcpy(dst->pData, src->pData, dst->numCols * dst->numRows * sizeof(float));
}

void arm_mat_vec_mult_f32(arm_matrix_instance_f32 *pSrcMat, float32_t *pVec, float32_t *pDst) {

  arm_matrix_instance_f32 vecSrc;
  arm_matrix_instance_f32 vecDst;

  arm_mat_init_f32(&vecSrc, pSrcMat->numCols, 1, pVec);
  arm_mat_init_f32(&vecDst, pSrcMat->numRows, 1, pDst);

  arm_mat_mult_f32(pSrcMat, &vecSrc, &vecDst);
}

void arm_mat_get_column_f32(const arm_matrix_instance_f32 *mat, int col, float32_t *vec) {
  for (int row = 0; row < mat->numRows; row++) {
    vec[row] = arm_mat_get_entry_f32(mat, row, col);
  }
}

void arm_mat_set_column_f32(arm_matrix_instance_f32 *mat, int col, float32_t *vec) {
  for (int row = 0; row < mat->numRows; row++) {
    arm_mat_set_entry_f32(mat, row, col,  vec[row]);
  }
}

void arm_mat_fill_diag_f32(arm_matrix_instance_f32 *mat, int row_00, int col_00, float value) {
  for (int i = 0; (i < (mat->numRows-row_00)) && (i < mat->numCols-col_00); i++) {
    arm_mat_set_entry_f32(mat, i+row_00, i+col_00, value);
  }
}

void arm_mat_set_diag_f32(arm_matrix_instance_f32 *mat, int first, int last, float value) {
  for (int i = first; i <= last; i++) {
    arm_mat_set_entry_f32(mat, i, i, value);
  }
}

void arm_mat_insert_32(const arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst, int row_00, int col_00) {
  for(int i = 0; i < src->numRows; i++) {
    for(int j = 0; j < src->numCols; j++) {
      arm_mat_set_entry_f32(dst, i+row_00, j+col_00, arm_mat_get_entry_f32(src, i, j));
    }
  }
}

void arm_mat_insert_mult_32(const arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst, int row_00, int col_00, float factor) {
  for(int i = 0; i < src->numRows; i++) {
    for(int j = 0; j < src->numCols; j++) {
      arm_mat_set_entry_f32(dst, i+row_00, j+col_00, factor * arm_mat_get_entry_f32(src, i, j));
    }
  }
}

void arm_mat_print_f32(arm_matrix_instance_f32 *pMat) {

  printf("%dx%d-Matrix:\n", pMat->numRows, pMat->numCols);

  for (int row = 0; row < pMat->numRows; row++) {
    printf("[ ");
    for (int col = 0; col < pMat->numCols; col++) {
      printf("%+0.3E  ", arm_mat_get_entry_f32(pMat, row, col));
    }
    printf("]\n");
  }
}

void arm_vec3_print_f32(float32_t *vec) {

  printf("Vector:\n");

  for (int row = 0; row < 3; row++) {
    printf("[ ");
    printf("%+0.3E ", vec[row]);
    printf("]\n");
  }
}

void arm_vec3_add_f32(float32_t *vecA, float32_t *vecB, float32_t *result) {
  result[0] = vecA[0] + vecB[0];
  result[1] = vecA[1] + vecB[1];
  result[2] = vecA[2] + vecB[2];
}

void arm_vec3_sub_f32(float32_t *vecA, float32_t *vecB, float32_t *result) {
  result[0] = vecA[0] - vecB[0];
  result[1] = vecA[1] - vecB[1];
  result[2] = vecA[2] - vecB[2];
}

void arm_vec3_scalar_mult_f32(float32_t *vec, float scalar, float32_t *result) {
  result[0] = vec[0] * scalar;
  result[1] = vec[1] * scalar;
  result[2] = vec[2] * scalar;
}

float arm_vec3_length_f32(float32_t *vec) {
  return sqrtf(arm_vec3_length_squared_f32(vec));
}

float arm_vec3_length_squared_f32(float32_t *vec) {
  float result = 0.;
  result += vec[0] * vec[0];
  result += vec[1] * vec[1];
  result += vec[2] * vec[2];
  return result;
}

float arm_vec3_dot_product_f32(float32_t *vecA, float32_t *vecB) {
  float result = 0.;
  result += vecA[0] * vecB[0];
  result += vecA[1] * vecB[1];
  result += vecA[2] * vecB[2];
  return result;
}

void arm_vec3_cross_product_f32(float32_t *vecA, float32_t *vecB, float32_t *result) {
  result[0] = vecA[1] * vecB[2] - vecA[2] * vecB[1];
  result[1] = vecA[2] * vecB[0] - vecA[0] * vecB[2];
  result[2] = vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

void arm_vec3_element_product_f32(float32_t *vecA, float32_t *vecB, float32_t *result) {
  result[0] = vecA[0] * vecB[0];
  result[1] = vecA[1] * vecB[1];
  result[2] = vecA[2] * vecB[2];
}

void arm_vec3_copy_f32(float32_t *vecSrc, float32_t *vecDst) {
  vecDst[0] = vecSrc[0];
  vecDst[1] = vecSrc[1];
  vecDst[2] = vecSrc[2];
}

void arm_vec3_normalize_f32(float32_t *vec) {
  float len;
  len = arm_vec3_length_f32(vec);
  len = 1/len;
  arm_vec3_scalar_mult_f32(vec, len, vec);
  
}

void arm_vecN_add_f32(int N, float32_t *vecA, float32_t *vecB, float32_t *result) {
  N--;
  for (; N >= 0; N--) {
    result[N] = vecA[N] + vecB[N];
  }
}

void arm_vecN_sub_f32(int N, float32_t *vecA, float32_t *vecB, float32_t *result) {
  N--;
  for (; N >= 0; N--) {
    result[N] = vecA[N] - vecB[N];
  }
}

void arm_vecN_scalar_mult_f32(int N, float32_t *vec, float scalar, float32_t *result) {
  N--;
  for (; N >= 0; N--) {
    result[N] = vec[N] * scalar;
  }
}

void arm_vecN_print_f32(int N, float32_t *vec) {

  printf("Vector:\n");

  for (int row = 0; row < N; row++) {
    printf("[ ");
    printf("%+0.3E ", vec[row]);
    printf("]\n");
  }
}