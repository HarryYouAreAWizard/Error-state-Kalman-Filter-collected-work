


"""wrapper for the linalg functions in the libmain shared object"""


import ctypes
from ctypes import (
    POINTER as p, 
    c_float as f, 
    c_uint16 as u
)

def load_library():
	# lib_path = "/home/noah/NEXT/estimator_stand_alone_builder/Error-State-Kalman-Filter-standalone/libmain.so"
    # lib_path = r"Error-State-Kalman-Filter-standalone/libmain.so"
    lib_path = "../ESKF/libeskf.so"
    return ctypes.CDLL(lib_path)

lib = load_library()

# define the arm_matrix_instance_f32 matrix struct
class ArmMatrix(ctypes.Structure):
	# parameters needed to initialize the struct
	_fields_ = [
		("numRows", ctypes.c_uint16),
		("numCols", ctypes.c_uint16),
		("pData", ctypes.POINTER(ctypes.c_float))
	]


#---------------example---------------
# set the argument types. Is something like a header decleration
lib.matscale.argtypes = [
	ctypes.POINTER(ArmMatrix), 
	ctypes.c_float,
	ctypes.POINTER(ArmMatrix)]

# define how the function works, like the .c file
def matscale(matrix:ArmMatrix, scalar:float, result:ArmMatrix):
	lib.matscale(matrix, scalar, result)


#---------------declerations----------------
lib.get_element.argtypes =    [p(ArmMatrix), u, u]
lib.set_element.argtypes =    [p(ArmMatrix), u, u, f]
lib.mat_init.argtypes =       [p(ArmMatrix), u, u, p(f)]
lib.set_zero.argtypes =       [p(ArmMatrix)]
lib.set_identity.argtypes =   [p(ArmMatrix)]
lib.init_mat_zeros.argtypes = [p(ArmMatrix), u, u, p(f)]
lib.mat_resize.argtypes =     [p(ArmMatrix), u, u]
lib.intmedsetup.argtypes =    [p(ArmMatrix), u, u]
lib.matscale.argtypes =       [p(ArmMatrix), f, p(ArmMatrix)]
lib.matadd.argtypes =         [p(ArmMatrix), p(ArmMatrix), p(ArmMatrix)]
lib.matsub.argtypes =         [p(ArmMatrix), p(ArmMatrix), p(ArmMatrix)]
lib.matmul.argtypes =         [p(ArmMatrix), p(ArmMatrix), p(ArmMatrix)]
lib.mattrans.argtypes =       [p(ArmMatrix), p(ArmMatrix)]
lib.matinv.argtypes =         [p(ArmMatrix), p(ArmMatrix)]

#---------------function wrappers----------------
# // getset
def get_element(Matrix, row, col):
    """currently does not work. Returns garbage values
    a substitute implementation is provided for now"""
    return Matrix.pData[row * Matrix.numCols + col]
    return lib.get_element(pMatrix, row, col)

def set_element(Matrix, row, col, value):           lib.set_element(Matrix, row, col, value)
def mat_init(Matrix, numRows, numCols, data):       lib.mat_init(Matrix, numRows, numCols, data)
def set_zero(Matrix):                               lib.set_zero(Matrix)
def set_identity(Matrix):                           lib.set_identity(Matrix)
def init_mat_zeros(Matrix, numRows, numCols, data): lib.init_mat_zeros(Matrix, numRows, numCols, data) 
def mat_resize(Matrix, newNumRows, newNumCols):     lib.mat_resize(Matrix, newNumRows, newNumCols) 
def intmedsetup(Matrix, numRows, numCols):          lib.intmedsetup(Matrix, numRows, numCols)
def matscale(Matrix, scalar, Result):               lib.matscale(Matrix, scalar, Result)
def matadd(MatrixA, MatrixB, Result):               lib.matadd(MatrixA, MatrixB, Result)
def matsub(MatrixA, MatrixB, Result):               lib.matsub(MatrixA, MatrixB, Result)
def matmul(MatrixA, MatrixB, Result):               lib.matmul(MatrixA, MatrixB, Result)
def mattrans(Matrix, Result):                       lib.mattrans(Matrix, Result)
def matinv(Matrix, Result):                         lib.matinv(Matrix, Result)



#--------------------------util------------------------------

def print_matrix(Matrix):
    for i in range(Matrix.numRows):
        for j in range(Matrix.numCols):
            print(get_element(Matrix, i, j), end=" ")
        print()
    print()

def test_lib():
    
    # create some matrices
    # data buffers for the matrices
    A_data = (ctypes.c_float * 4)(1.0, 2.0, 3.0, 4.0)
    B_data = (ctypes.c_float * 4)(5.0, 6.0, 7.0, 8.0)
    Result_data = (ctypes.c_float * 4)()
    # initialize the ArmMatrix structs
    A = ArmMatrix(2, 2, A_data)
    B = ArmMatrix(2, 2, B_data)
    Result = ArmMatrix(2, 2, Result_data)
    # use arm_mat_init_f32 to initialize the matrices
    mat_init(A, 2, 2, A_data)
    mat_init(B, 2, 2, B_data)
    mat_init(Result, 2, 2, Result_data)

    print("A =")
    print_matrix(A)

    set_element(B, 0, 0, 5.0)
    print("B =")
    print_matrix(B)

    matsub(A, B, Result)
    print("A - B =")
    print_matrix(Result)

# test_lib()