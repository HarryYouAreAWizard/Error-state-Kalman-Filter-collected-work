


"""wrap the quaternion c code"""

import ctypes
from ctypes import (
    POINTER as p, 
    c_float as f, 
    c_uint16 as u
)

from math import sqrt
from wrap_linalg import load_library, ArmMatrix, mat_init


lib = load_library()
#-------------declarations-------------
# lib.initialize_quaternion_helpers.argtypes   = ctypes.c_void_p;
lib.hamiltonian_product.argtypes             = (p(ArmMatrix), p(ArmMatrix), p(ArmMatrix));
lib.conjugate_quaternion.argtypes            = (p(ArmMatrix), p(ArmMatrix) );
lib.quaternion_inverse.argtypes              = (p(ArmMatrix), p(ArmMatrix));
lib.rotation_product.argtypes                = (p(ArmMatrix), p(ArmMatrix), p(ArmMatrix));
lib.rotate_vector.argtypes                   = (p(ArmMatrix), p(ArmMatrix), p(ArmMatrix));
lib.rotation_matrix2quaternion.argtypes      = (p(ArmMatrix), p(ArmMatrix));
lib.quaternion2rotation_matrix.argtypes      = (p(ArmMatrix), p(ArmMatrix));
lib.rotation_vector2quaternion.argtypes      = (p(ArmMatrix), p(ArmMatrix));
lib.rotation_vector2rotation_matrix.argtypes = (p(ArmMatrix), p(ArmMatrix));
lib.construct_skew_matrix.argtypes           = (p(ArmMatrix), p(ArmMatrix));
lib.quaternion2eulerangles.argtypes          = (p(ArmMatrix), p(ArmMatrix));

#-------------definitions-------------
def initialize_quaternion_helpers():                                     lib.initialize_quaternion_helpers()
def hamiltonian_product(pQuat1, pQuat2, pResult):                        lib.hamiltonian_product(pQuat1, pQuat2, pResult)
def conjugate_quaternion(pQuat, pResult):                                lib.conjugate_quaternion(pQuat, pResult)
def quaternion_inverse(pQuat, pResult):                                  lib.quaternion_inverse(pQuat, pResult)
def rotation_product(pQuatTarget, pQuatRotator, pResult):                lib.rotation_product(pQuatTarget, pQuatRotator, pResult)
def rotate_vector(pVector, pQuatRotator, pResult):                       lib.rotate_vector(pVector, pQuatRotator, pResult)
def rotation_matrix2quaternion(pRotation_matrix, pQuaternion):           lib.rotation_matrix2quaternion(pRotation_matrix, pQuaternion)
def quaternion2rotation_matrix(pQuaternion, pRotation_matrix):           lib.quaternion2rotation_matrix(pQuaternion, pRotation_matrix)
def rotation_vector2quaternion(pRotation_vector, pQuaternion):           lib.rotation_vector2quaternion(pRotation_vector, pQuaternion)
def rotation_vector2rotation_matrix(pRotation_vector, pRotation_matrix): lib.rotation_vector2rotation_matrix(pRotation_vector, pRotation_matrix)
def construct_skew_matrix(pVector3x3, pSkew_matrix):                     lib.construct_skew_matrix(pVector3x3, pSkew_matrix)
def quaternion2eulerangles(pQuat, pEuler_angles):                        lib.quaternion2eulerangles(pQuat, pEuler_angles)


#-------------aliases for ESKF-------------
quaternion_from_rotation_matrix = rotation_matrix2quaternion
rotation_matrix_from_quaternion = quaternion2rotation_matrix
inverse_quaternion = quaternion_inverse


#-------------util-------------
def printquat(quat):
    for i in range(quat.numRows):
        for j in range(quat.numCols):
            print(f"{quat.pData[i * quat.numCols + j]}", end = " ")
        print()
def test_lib():

    # initialize helpers. must be done before any use of quaternion math
    initialize_quaternion_helpers()

    quat1_data = (ctypes.c_float * 4)(1.0, 0.0, 0.0, 0.0)
    quat2_data = (ctypes.c_float * 4)(1.0/sqrt(2), 0.0, 1.0/sqrt(2), 0.0)
    mat_data = (ctypes.c_float * 9)()
    vector_data = (ctypes.c_float * 3)(1.0, 0.0, 0.0)
    vector_result_data = (ctypes.c_float * 3)()
    quat1 = ArmMatrix()
    quat2 = ArmMatrix()
    mat = ArmMatrix()
    vector = ArmMatrix()
    vector_result = ArmMatrix()

    mat_init(quat1, 4, 1, quat1_data)
    mat_init(quat2, 4, 1, quat2_data)
    mat_init(mat, 3, 3, mat_data)
    mat_init(vector, 3, 1, vector_data)
    mat_init(vector_result, 3, 1, vector_result_data)

    print(f"quat2 = ")
    printquat(quat2)

    print(f"vector = ")
    printquat(vector)

    print(f"vector_result = ")
    printquat(vector_result)

    print("rotating")
    rotate_vector(vector, quat2, vector_result)

    print(f"vector_result = ")
    printquat(vector_result)

    quaternion2rotation_matrix(quat2, mat)
    rotation_matrix2quaternion(mat, quat1)
    print("rotation matrix")
    printquat(mat)
    print("quat1")
    printquat(quat1)

# test_lib()
    