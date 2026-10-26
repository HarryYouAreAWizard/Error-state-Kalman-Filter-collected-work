



/*
 * 
 * This file implements quaternion operations and conversions, using the 
 * CMSIS-DSP library for basic quaternion math functions. 
 *
*/

// link to the CMSIS-DSP library for quaternion math functions
#include "arm_math.h"
#include "linalg.h"

// most quaternion math is already implemented in the CMSIS-DSP library, 
// but some functions are missing. This file is for implementing
// rotation vector to quaternion and to rotation matrix.

static arm_matrix_instance_f32 helper4x1_1;
static arm_matrix_instance_f32 helper4x1_2;
static arm_matrix_instance_f32 helper4x1_3;
static float32_t helper4x1_1_data[4];
static float32_t helper4x1_2_data[4];
static float32_t helper4x1_3_data[4];

static arm_matrix_instance_f32 helper_rotation_product_1;
static arm_matrix_instance_f32 helper_rotation_product_2;
static float32_t helper_rotation_product_data_1[4];
static float32_t helper_rotation_product_data_2[4];


void initialize_quaternion_helpers(void) {
    // initialize any helper matrices here
    init_mat_zeros(&helper4x1_1, 4, 1, helper4x1_1_data); // util
    init_mat_zeros(&helper4x1_2, 4, 1, helper4x1_2_data); // util
    init_mat_zeros(&helper4x1_3, 4, 1, helper4x1_3_data); // util
    init_mat_zeros(&helper_rotation_product_1, 4, 1, helper_rotation_product_data_1); // for rotation product only
    init_mat_zeros(&helper_rotation_product_2, 4, 1, helper_rotation_product_data_2); // for rotation product only
}

// basic quaternion operations
void hamiltonian_product(        
    arm_matrix_instance_f32 *q1, 
    arm_matrix_instance_f32 *q2, 
    arm_matrix_instance_f32 *result) {
        // initialize placeholder matrix once per call (TODO: remove when unused)
        // wrapper for cmsis-dsp hamiltonian product
        arm_quaternion_product_single_f32(q1->pData, q2->pData, result->pData);
    }

void conjugate_quaternion(
    arm_matrix_instance_f32 *q,
    arm_matrix_instance_f32 *result) {
        // the conjugate of a quaternion q = [w, x, y, z] is given by q* = [w, -x, -y, -z]
        result->pData[0] = q->pData[0]; // w
        result->pData[1] = -q->pData[1]; // x
        result->pData[2] = -q->pData[2]; // y
        result->pData[3] = -q->pData[3]; // z
}
void quaternion_inverse(
    arm_matrix_instance_f32 *q,
    arm_matrix_instance_f32 *result) {
        // wrapper for cmsis-dsp quaternion inverse
        // third argument: number of quaternions. Always called with one.
        arm_quaternion_inverse_f32(q->pData, result->pData, 1);
    }

// rotation operations
int rotation_product(
    arm_matrix_instance_f32 *pQuatTarget, 
    arm_matrix_instance_f32 *pQuatRotator, 
    arm_matrix_instance_f32 *pResult) {
        // rotate quaternion QuatTarget with quaternion QuatRotator: 
        // QuatTarget' = QuatRotator * QuatTarget * QuatRotator^-1
        // function expects pointers to quaternions
        quaternion_inverse(pQuatRotator, &helper_rotation_product_1);
        hamiltonian_product(pQuatRotator, pQuatTarget, &helper_rotation_product_2);
        hamiltonian_product(&helper_rotation_product_2, &helper_rotation_product_1, pResult);
        return 0;
    }

int rotate_vector(
    arm_matrix_instance_f32 *pVector, 
    arm_matrix_instance_f32 *pQuatRotator, 
    arm_matrix_instance_f32 *pResult) {
    // rotate a vector by treating it as a quaternion with zero scalar part, and applying the same formula as rotation_product
    // v' = q * v * q^-1, where v is treated as a quaternion with w=0
    set_element(&helper4x1_1, 0, 0, 0.0f); // w component is zero
    set_element(&helper4x1_1, 1, 0, pVector->pData[0]); // x
    set_element(&helper4x1_1, 2, 0, pVector->pData[1]); // y
    set_element(&helper4x1_1, 3, 0, pVector->pData[2]); // z

    rotation_product(&helper4x1_1, pQuatRotator, &helper4x1_2);
    // Copy the result back to the output vector
    set_element(pResult, 0, 0, helper4x1_2.pData[1]);
    set_element(pResult, 1, 0, helper4x1_2.pData[2]);
    set_element(pResult, 2, 0, helper4x1_2.pData[3]);
    return 0;
}


// quaternion <-> rotation matrix
int rotation_matrix2quaternion(
    arm_matrix_instance_f32 *pRotation_matrix, 
    arm_matrix_instance_f32 *pQuaternion) {
        // check sizes of input matrix and quaternion
        if (pRotation_matrix->numRows != 3 || pRotation_matrix->numCols != 3) {
            return -1; // error: input rotation matrix must be 3x3
        }
        if (pQuaternion->numRows != 4 || pQuaternion->numCols != 1) {
            return -1; // error: output quaternion must be 4x1
        }

        // wrapper for cmsis-dsp rotation matrix to quaternion function
        arm_rotation2quaternion_f32(pRotation_matrix->pData, pQuaternion->pData, 1);
        return 0;
    }

int quaternion2rotation_matrix(
    arm_matrix_instance_f32 *pQuaternion, 
    arm_matrix_instance_f32 *pRotation_matrix) {
        // check sizes of input quaternion and output matrix
        if (pQuaternion->numRows != 4 || pQuaternion->numCols != 1) {
            return -1; // error: input quaternion must be 4x1
        }
        if (pRotation_matrix->numRows != 3 || pRotation_matrix->numCols != 3) {
            return -1; // error: output rotation matrix must be 3x3
        }

        // wrapper for cmsis-dsp quaternion to rotation matrix function
        arm_quaternion2rotation_f32(pQuaternion->pData, pRotation_matrix->pData, 1);
        return 0;
    }


// rotation vector -> 
int rotation_vector2quaternion(
    arm_matrix_instance_f32 *pRotation_vector, 
    arm_matrix_instance_f32 *pQuaternion) {
    // calculate norm
    float32_t dot = 0.0f;
    float32_t phi = 0.0f;
    arm_dot_prod_f32(pRotation_vector->pData, pRotation_vector->pData, 3, &dot);
    arm_sqrt_f32(dot, &phi);
    if (phi < 1e-6) {
        // If the angle is very small, return the identity quaternion
        pQuaternion->pData[0] = 1.0f; // w
        pQuaternion->pData[1] = 0.0f; // x
        pQuaternion->pData[2] = 0.0f; // y
        pQuaternion->pData[3] = 0.0f; // z
    } else {
        float32_t half_angle = phi / 2.0f;
        float32_t sin_half_angle = sinf(half_angle);
        float32_t cos_half_angle = cosf(half_angle);
        
        pQuaternion->pData[0] = cos_half_angle; // w
        pQuaternion->pData[1] = (pRotation_vector->pData[0] / phi) * sin_half_angle; // x
        pQuaternion->pData[2] = (pRotation_vector->pData[1] / phi) * sin_half_angle; // y
        pQuaternion->pData[3] = (pRotation_vector->pData[2] / phi) * sin_half_angle; // z
    }
    return 0;
}

int rotation_vector2rotation_matrix(
    arm_matrix_instance_f32 *pRotation_vector, 
    arm_matrix_instance_f32 *pRotation_matrix) {

    /*
    this is a different implementation, than the one seen in the python-c version. 
    This one is simpler, but untested
    python version:
    phi = np.linalg.norm(rotation_vector)
    u = rotation_vector/phi

    skew_matrix = construct_skew_matrix(u)
            
    R = np.eye(3) + sin(phi) * skew_matrix + (1 - cos(phi)) * skew_matrix@skew_matrix
    */

    // calculate norm
    float32_t phi = 0.0f;
    float32_t dot = 0.0f;
    arm_dot_prod_f32(pRotation_vector->pData, pRotation_vector->pData, 3, &dot);
    arm_sqrt_f32(dot, &phi);
    if (phi < 1e-6) {
        // If the angle is very small, return the identity matrix
        for (int i = 0; i < 9; i++) {
            pRotation_matrix->pData[i] = (i % 4 == 0) ? 1.0f : 0.0f; // Identity matrix
        }
    } else {
        float32_t x = pRotation_vector->pData[0];
        float32_t y = pRotation_vector->pData[1];
        float32_t z = pRotation_vector->pData[2];
        
        float32_t c = cosf(phi);
        float32_t s = sinf(phi);
        float32_t t = 1.0f - c;

        // Normalize the rotation vector
        float32_t norm = sqrtf(x*x + y*y + z*z);
        x /= norm;
        y /= norm;
        z /= norm;

        // Compute the rotation matrix elements
        pRotation_matrix->pData[0] = t*x*x + c;
        pRotation_matrix->pData[1] = t*x*y - s*z;
        pRotation_matrix->pData[2] = t*x*z + s*y;
        
        pRotation_matrix->pData[3] = t*x*y + s*z;
        pRotation_matrix->pData[4] = t*y*y + c;
        pRotation_matrix->pData[5] = t*y*z - s*x;
        
        pRotation_matrix->pData[6] = t*x*z - s*y;
        pRotation_matrix->pData[7] = t*y*z + s*x;
        pRotation_matrix->pData[8] = t*z*z + c;
    }
    return 0;
}


// utility
int construct_skew_matrix(
    arm_matrix_instance_f32 *pVector3x3, 
    arm_matrix_instance_f32 *pSkew_matrix) {
    // u is a 3x1 vector, skew_matrix is a 3x3 matrix
    float32_t u_x = pVector3x3->pData[0];
    float32_t u_y = pVector3x3->pData[1];    
    float32_t u_z = pVector3x3->pData[2];

    pSkew_matrix->pData[0] = 0.0f;
    pSkew_matrix->pData[1] = -u_z;
    pSkew_matrix->pData[2] = u_y;
    
    pSkew_matrix->pData[3] = u_z;
    pSkew_matrix->pData[4] = 0.0f;
    pSkew_matrix->pData[5] = -u_x;
    
    pSkew_matrix->pData[6] = -u_y;
    pSkew_matrix->pData[7] = u_x;
    pSkew_matrix->pData[8] = 0.0f;

    return 0;
}


int quaternion2eulerangles(
    arm_matrix_instance_f32 *pQuat, 
    arm_matrix_instance_f32 *pEuler_angles) {
    // convert quaternion to euler angles (roll, pitch, yaw)
        float32_t qw = pQuat->pData[0];
        float32_t qx = pQuat->pData[1];
        float32_t qy = pQuat->pData[2];
        float32_t qz = pQuat->pData[3];
    
        float32_t roll = pEuler_angles->pData[0];
        float32_t pitch = pEuler_angles->pData[1];
        float32_t yaw = pEuler_angles->pData[2];

        // # roll (x-axis rotation)
        // sinr_cosp = 2 * (qw * qx + qy * qz)
        // cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        // roll = np.arctan2(sinr_cosp, cosr_cosp)
        float32_t sinr_cosp = 2 * (qw * qx + qy * qz);
        float32_t cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        roll = atan2f(sinr_cosp, cosr_cosp); 
        

        // # pitch (y-axis rotation)
        // sinp = 2 * (qw * qy - qz * qx)
        // if abs(sinp) >= 1:
        //         pitch = np.sign(sinp) * (np.pi / 2)  # use 90 degrees if out of range
        //     else:
        //         pitch = np.arcsin(sinp)
        float32_t sinp = 2 * (qw * qy - qz * qx);
        if (fabsf(sinp) >= 1) {
            pitch = copysignf(PI / 2, sinp); // use 90 degrees if out of range
        } else {
            pitch = asinf(sinp);
        }

        // # yaw (z-axis rotation)
        // siny_cosp = 2 * (qw * qz + qx * qy)
        // cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        // yaw = np.arctan2(siny_cosp, cosy_cosp)
        float32_t siny_cosp = 2 * (qw * qz + qx * qy);
        float32_t cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        yaw = atan2f(siny_cosp, cosy_cosp);

        pEuler_angles->pData[0] = roll;
        pEuler_angles->pData[1] = pitch;
        pEuler_angles->pData[2] = yaw;
        
        return 0;
}

