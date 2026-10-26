

/*
 * linalg.c
 *
 * library for linear algebra functions.
 * Using the CMSIS-DSP library as much as possible.
 *  
 */


#include "arm_math.h"
#include "linalg.h"
// #include "user/logging.h"

// #define MAXROWS MAX_MATRIX_SIZE
// #define MAXCOLS MAX_MATRIX_SIZE

// getset
float32_t get_element(
    arm_matrix_instance_f32 *pMatrix, 
    int row, int col) {
        /* 
        there are currently problems with the get function when accessed from python wrapper
        it returns garbage values when called from python, but works fine when called from C.
        */ 

        // check if indices are within bounds
        if (row >= pMatrix->numRows || col >= pMatrix->numCols) {
            return 0.0f; // out of bounds
        }
        // calculate index in row-major order and return value
        return pMatrix->pData[row * pMatrix->numCols + col];
    }

void set_element(
    arm_matrix_instance_f32 *pMatrix, 
    int row, int col, float32_t value) {
        /*
        set an element in the matrix

        boundary checks are removed for performance
        */
        // if (row >= pMatrix->numRows || col >= pMatrix->numCols) {
        //     // out of bounds
        //     return 1;
        // }
        pMatrix->pData[row * pMatrix->numCols + col] = value;
    }

int copy_matrix(arm_matrix_instance_f32 *pMatrixA, arm_matrix_instance_f32 *pMatrixB) {
    /*copy from matrix A to matrix B
    matrices must be of equal size
    
    check is removed for performance
    */ 

    // check size
    // if (pMatrixA->numRows != pMatrixB->numRows || pMatrixA->numCols != pMatrixB->numCols) {
    //     return ARM_MATH_SIZE_MISMATCH;
    // } 
    // for (int i=0; i<pMatrixA->numRows; i++) {
    //     for (int j=0; j<pMatrixA->numCols; j++) {
    //         set_element(pMatrixB, i, j, pMatrixA->pData[i*pMatrixA->numCols + j]);
    //     }
    // }
    uint32_t total_elements = pMatrixA->numRows * pMatrixA->numCols;
    arm_copy_f32(pMatrixA->pData, pMatrixB->pData, total_elements);
    return 0;
    // return ARM_MATH_SUCCESS;
}

// initialization and resizing
void mat_init(
    arm_matrix_instance_f32 *pMatrix, 
    int numRows, int numCols, float32_t *data) {
        arm_mat_init_f32(pMatrix, numRows, numCols, data);
    }

void set_zero(
    arm_matrix_instance_f32 *pMatrix) {
    uint32_t total_elements = pMatrix->numRows * pMatrix->numCols;
    arm_fill_f32(0.0f, pMatrix->pData, total_elements);
}

        // for (int i = 0; i < pMatrix->numRows * pMatrix->numCols; i++) {
        //     pMatrix->pData[i] = 0.0f;
        // }
    // }

void set_identity(
    arm_matrix_instance_f32 *pMatrix) {
        // for (int i = 0; i < pMatrix->numRows * pMatrix->numCols; i++) {
        //     if (i % (pMatrix->numCols + 1) == 0) {
        //         pMatrix->pData[i] = 1.0f; // diagonal element
        //     } else {
        //         pMatrix->pData[i] = 0.0f; // off-diagonal element
        //     }
        //     // alternative implementation using set_element function
        //     // pMatrix->pData[i] = (i % (pMatrix->numCols + 1) == 0) ? 1.0f : 0.0f; // Identity matrix
        // }
        set_zero(pMatrix);
        // run through diagonal and set to 1
        for (int i = 0; i < pMatrix->numRows && i < pMatrix->numCols; i++) {
            set_element(pMatrix, i, i, 1.0f);
        }
    }

void init_mat_zeros(
    arm_matrix_instance_f32 *pMatrix, 
    int numRows, int numCols, float32_t *data) {
        // use arm initialization and immediately set all values to zero
        arm_mat_init_f32(pMatrix, numRows, numCols, data);
        set_zero(pMatrix);
        // for (int i = 0; i < numRows * numCols; i++) {
            // pMatrix->pData[i] = 0.0f;
        // }
    }

int mat_resize(
    arm_matrix_instance_f32 *pMatrix, 
    int newNumRows, int newNumCols) {
    // check if new size is within bounds
    if (newNumRows > MAXROWS || newNumCols > MAXCOLS) {
        return -1; // error: new size exceeds maximum allowed
    }
    pMatrix->numRows = newNumRows;
    pMatrix->numCols = newNumCols;
    return 0; // success
}

void intmedsetup(
    arm_matrix_instance_f32 *pMatrix, 
    int numRows, int numCols) {
        /*
        function to resize and set zero intermediate matrices on the fly
        */ 
        mat_resize(pMatrix, numRows, numCols);
        set_zero(pMatrix);
        return;
    }

// matrix operations

float32_t vecnorm(arm_matrix_instance_f32 *pVector) {
    // if (pVector->numCols != 1) {
    //     return 0.0;
    // }

    // float32_t norm2 = 0.0f;
    // for (int i = 0; i < pVector->numRows; i++) {
    //     norm2 += pVector->pData[i] * pVector->pData[i];
    // }
    // float32_t output = 0.0f;
    // arm_sqrt_f32(norm2, &output);
    // return output;
    if (pVector->numCols != 1) {
        return 0.0f;
    }

    float32_t sum_of_squares = 0.0f;
    uint32_t total_elements = pVector->numRows;
    
    // Computes sum(x_i ^ 2) instantly using hardware SIMD/loop-unrolling
    arm_power_f32(pVector->pData, total_elements, &sum_of_squares);
    
    float32_t output = 0.0f;
    arm_sqrt_f32(sum_of_squares, &output);
    return output;
}

int matscale(
    arm_matrix_instance_f32 *pMatrix, 
    float32_t scalar, 
    arm_matrix_instance_f32 *pResult) {
        // check if result matrix has correct dimensions
        if (pResult->numRows != pMatrix->numRows || pResult->numCols != pMatrix->numCols) {
            return -1; // error: result matrix has incorrect dimensions
        }
        // perform matrix scaling using CMSIS-DSP function
        arm_mat_scale_f32(pMatrix, scalar, pResult);
        return 0; // success
    }

int matadd(
    arm_matrix_instance_f32 *pMatrixA, 
    arm_matrix_instance_f32 *pMatrixB, 
    arm_matrix_instance_f32 *pResult) {
        // check if matrices have correct dimensions
        if (pMatrixA->numRows != pMatrixB->numRows || pMatrixA->numCols != pMatrixB->numCols) {
            return -1; // error: matrices have incorrect dimensions
        }
        // check if result matrix has correct dimensions
        if (pResult->numRows != pMatrixA->numRows || pResult->numCols != pMatrixA->numCols) {
            return -1; // error: result matrix has incorrect dimensions
        }
        // perform matrix addition using CMSIS-DSP function
        arm_mat_add_f32(pMatrixA, pMatrixB, pResult);
        return 0; // success
    }

int matsub(
    arm_matrix_instance_f32 *pMatrixA, 
    arm_matrix_instance_f32 *pMatrixB, 
    arm_matrix_instance_f32 *pResult) {
        // check if matrices have correct dimensions
        if (pMatrixA->numRows != pMatrixB->numRows || pMatrixA->numCols != pMatrixB->numCols) {
            return -1; // error: matrices have incorrect dimensions
        }
        // check if result matrix has correct dimensions
        if (pResult->numRows != pMatrixA->numRows || pResult->numCols != pMatrixA->numCols) {
            return -1; // error: result matrix has incorrect dimensions
        }
        // perform matrix subtraction using CMSIS-DSP function
        arm_mat_sub_f32(pMatrixA, pMatrixB, pResult);
        return 0; // success
    }

int matmul(
    arm_matrix_instance_f32 *pMatrixA, 
    arm_matrix_instance_f32 *pMatrixB, 
    arm_matrix_instance_f32 *pResult) {
        // check if inner dimensions match
        if (pMatrixA->numCols != pMatrixB->numRows) {

            // uart_log_msg("Matrix multiplication error: inner dimensions do not match\r\n");
            return -1; // error: inner dimensions do not match
        }
        // check if result matrix has correct dimensions
        if (pResult->numRows != pMatrixA->numRows || pResult->numCols != pMatrixB->numCols) {
            // uart_log_msg("Matrix multiplication error: result matrix has incorrect dimensions\r\n");
            return -1; // error: result matrix has incorrect dimensions
        }
        // perform matrix multiplication using CMSIS-DSP function
        arm_mat_mult_f32(pMatrixA, pMatrixB, pResult);
        return 0; // success
    }   

int mattrans(
    arm_matrix_instance_f32 *pMatrix, 
    arm_matrix_instance_f32 *pResult) {
        // check if result matrix has correct dimensions
        if (pResult->numRows != pMatrix->numCols || pResult->numCols != pMatrix->numRows) {
            return -1; // error: result matrix has incorrect dimensions
        }
        // perform matrix transpose using CMSIS-DSP function
        arm_mat_trans_f32(pMatrix, pResult);
        return 0; // success
    }

int matinv(
    arm_matrix_instance_f32 *pMatrix, 
    arm_matrix_instance_f32 *pResult) {
        // check if input matrix is square
        if (pMatrix->numRows != pMatrix->numCols) {
            // uart_log_msg("Matrix inversion error: input matrix is not square\r\n");
            return -1; // error: input matrix is not square
        }
        // check if result matrix has correct dimensions
        if (pResult->numRows != pMatrix->numRows || pResult->numCols != pMatrix->numCols) {
            // uart_log_msg("Matrix inversion error: result matrix has incorrect dimensions\r\n");
            return -1; // error: result matrix has incorrect dimensions
        }
        // perform matrix inversion using CMSIS-DSP function
        arm_status status = arm_mat_inverse_f32(pMatrix, pResult);
        if (status != ARM_MATH_SUCCESS) {
            // uart_log_msg("Matrix inversion error: matrix inversion failed\r\n");
            return status; // error: matrix inversion failed (e.g. singular matrix)
        }
        return 0; // success
    }
