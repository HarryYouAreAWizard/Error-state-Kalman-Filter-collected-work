
#ifndef LINALG_H
#define LINALG_H

#include "arm_math.h"

// #define MAX_MATRIX_SIZE 10
#define MAXROWS 10
#define MAXCOLS 10


#define DIVISION_BY_ZERO_THRESHOLD 1E-10f
#define ERROR_DIVISION_BY_ZERO 1

// getset
float32_t get_element(
    arm_matrix_instance_f32 *pMatrix, 
    int row, int col);

void set_element(
    arm_matrix_instance_f32 *pMatrix, 
    int row, int col, float32_t value);

int copy_matrix(arm_matrix_instance_f32 *pMatrixA, arm_matrix_instance_f32 *pMatrixB);

// initialization and resizing
void mat_init(
    arm_matrix_instance_f32 *pMatrix, 
    int numRows, int numCols, float32_t *data);

void set_zero(
    arm_matrix_instance_f32 *pMatrix);

void set_identity(
    arm_matrix_instance_f32 *pMatrix);

void init_mat_zeros(
    arm_matrix_instance_f32 *pMatrix, 
    int numRows, int numCols, float32_t *data);

int mat_resize(
    arm_matrix_instance_f32 *pMatrix, 
    int newNumRows, int newNumCols);

void intmedsetup(
    arm_matrix_instance_f32 *pMatrix, 
    int numRows, int numCols);

// matrix operations
float32_t vecnorm(arm_matrix_instance_f32 *pVector);

int matscale(
    arm_matrix_instance_f32 *pMatrix, 
    float32_t scalar, 
    arm_matrix_instance_f32 *pResult);

int matadd(
    arm_matrix_instance_f32 *pMatrixA, 
    arm_matrix_instance_f32 *pMatrixB, 
    arm_matrix_instance_f32 *pResult);

int matsub(
    arm_matrix_instance_f32 *pMatrixA, 
    arm_matrix_instance_f32 *pMatrixB, 
    arm_matrix_instance_f32 *pResult);

int matmul(
    arm_matrix_instance_f32 *pMatrixA, 
    arm_matrix_instance_f32 *pMatrixB, 
    arm_matrix_instance_f32 *pResult);

int mattrans(
    arm_matrix_instance_f32 *pMatrix, 
    arm_matrix_instance_f32 *pResult);

int matinv(
    arm_matrix_instance_f32 *pMatrix, 
    arm_matrix_instance_f32 *pResult);


#endif