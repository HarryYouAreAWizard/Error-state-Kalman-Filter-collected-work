#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include "arm_math.h"
#include "linalg.h"


void initialize_quaternion_helpers(void);

// basic quaternion operations
void hamiltonian_product(        
    arm_matrix_instance_f32 *pQuat1, 
    arm_matrix_instance_f32 *pQuat2, 
    arm_matrix_instance_f32 *pResult);

void conjugate_quaternion(
    arm_matrix_instance_f32 *pQuat,
    arm_matrix_instance_f32 *pResult);

void quaternion_inverse(
    arm_matrix_instance_f32 *pQuat,
    arm_matrix_instance_f32 *pResult);

// rotation operations
int rotation_product(
    arm_matrix_instance_f32 *pQuatTarget, 
    arm_matrix_instance_f32 *pQuatRotator, 
    arm_matrix_instance_f32 *pResult);

int rotate_vector(
    arm_matrix_instance_f32 *pVector, 
    arm_matrix_instance_f32 *pQuatRotator, 
    arm_matrix_instance_f32 *pResult);


// quaternion <-> rotation matrix
int rotation_matrix2quaternion(
    arm_matrix_instance_f32 *pRotation_matrix, 
    arm_matrix_instance_f32 *pQuaternion);

int quaternion2rotation_matrix(
    arm_matrix_instance_f32 *pQuaternion, 
    arm_matrix_instance_f32 *pRotation_matrix);

// rotation vector ->
int rotation_vector2quaternion(
    arm_matrix_instance_f32 *pRotation_vector, 
    arm_matrix_instance_f32 *pQuaternion);

int rotation_vector2rotation_matrix(
    arm_matrix_instance_f32 *pRotation_vector, 
    arm_matrix_instance_f32 *pRotation_matrix);

// utility
int construct_skew_matrix(
    arm_matrix_instance_f32 *pVector3x3, 
    arm_matrix_instance_f32 *pSkew_matrix);

int quaternion2eulerangles(
    arm_matrix_instance_f32 *pQuat, 
    arm_matrix_instance_f32 *pEuler_angles);


#endif
