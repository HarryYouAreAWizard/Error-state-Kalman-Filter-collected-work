

"""c wrapper usage example"""

import ctypes

def load_library():
	lib_path = "/home/noah/NEXT/estimator_stand_alone_builder/estimator_stand_alone/libmain.so"
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

# set the argument types. Is something like a header decleration
lib.matscale.argtypes = [
	ctypes.POINTER(ArmMatrix), 
	ctypes.c_float,
	ctypes.POINTER(ArmMatrix)]

# define how the function works, like the .c file
def matscale(matrix:ArmMatrix, scalar:float, result:ArmMatrix):
	lib.matscale(matrix, scalar, result)


mat_data = [1.0, 0.0, 0.0, 1.0]
res_data = [1.0, 0.0, 0.0, 1.0]
mat_buf = (ctypes.c_float * (2 * 2))(*mat_data)
res_buf = (ctypes.c_float * (2 * 2))(*res_data)
mat = ArmMatrix(2, 2, mat_buf)
res = ArmMatrix(2, 2, res_buf)
a = 2.0
matscale(mat, a, res)

for i in range(mat.numRows * mat.numCols):
	print(f"{mat.pData[i] = }")

for i in range(res.numRows * mat.numCols):
	print(f"{res.pData[i] = }")




# LIB_RELATIVE_PATH = os.path.join("estimator_stand_alone", "libmain.so")


# def _load_library():
# 	# here = os.path.dirname(os.path.abspath(__file__))
# 	# lib_path = os.path.join(here, LIB_RELATIVE_PATH)
# 	lib_path = "/home/noah/NEXT/estimator_stand_alone_builder/estimator_stand_alone/libmain.so"
# 	if not os.path.exists(lib_path):
# 		raise FileNotFoundError(f"Shared library not found: {lib_path}")
# 	return ctypes.CDLL(lib_path)


# class ArmMatrixF32(ctypes.Structure):
# 	_fields_ = [
# 		("numRows", ctypes.c_uint16),
# 		("numCols", ctypes.c_uint16),
# 		("pData", ctypes.POINTER(ctypes.c_float)),
# 	]


# class MatrixF32:
# 	def __init__(self, rows, cols, data=None):
# 		self.rows = int(rows)
# 		self.cols = int(cols)
# 		if data is None:
# 			data = [0.0] * (self.rows * self.cols)
# 		if len(data) != self.rows * self.cols:
# 			raise ValueError("data length must be rows*cols")
# 		self._buffer = (ctypes.c_float * (self.rows * self.cols))(*data)
# 		self._mat = ArmMatrixF32(self.rows, self.cols, self._buffer)

# 	@property
# 	def ptr(self):
# 		return ctypes.byref(self._mat)

# 	def to_list(self):
# 		return [float(x) for x in self._buffer]


# def _check(status, name):
# 	if status != 0:
# 		raise RuntimeError(f"{name} failed: {status}")


# # --- linalg prototypes ---
# _lib.get_element.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.c_int, ctypes.c_int]
# _lib.get_element.restype = ctypes.c_float

# _lib.set_element.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.c_int, ctypes.c_int, ctypes.c_float]
# _lib.set_element.restype = None

# _lib.init_mat_zeros.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
# _lib.init_mat_zeros.restype = None

# _lib.mat_resize.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.c_int, ctypes.c_int]
# _lib.mat_resize.restype = ctypes.c_int

# _lib.matscale.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.c_float, ctypes.POINTER(ArmMatrixF32)]
# _lib.matscale.restype = ctypes.c_int

# _lib.matadd.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.matadd.restype = ctypes.c_int

# _lib.matsub.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.matsub.restype = ctypes.c_int

# _lib.matmul.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.matmul.restype = ctypes.c_int

# _lib.mattrans.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.mattrans.restype = ctypes.c_int

# _lib.matinv.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.matinv.restype = ctypes.c_int


# # --- quaternion prototypes ---
# _lib.initialize_quaternion_helpers.argtypes = []
# _lib.initialize_quaternion_helpers.restype = None

# _lib.hamiltonian_product.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.hamiltonian_product.restype = None

# _lib.conjugate_quaternion.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.conjugate_quaternion.restype = None

# _lib.quaternion_inverse.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.quaternion_inverse.restype = None

# _lib.rotation_product.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.rotation_product.restype = ctypes.c_int

# _lib.rotate_vector.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.rotate_vector.restype = ctypes.c_int

# _lib.rotation_matrix2quaternion.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.rotation_matrix2quaternion.restype = ctypes.c_int

# _lib.quaternion2rotation_matrix.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.quaternion2rotation_matrix.restype = ctypes.c_int

# _lib.rotation_vector2quaternion.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.rotation_vector2quaternion.restype = ctypes.c_int

# _lib.rotation_vector2rotation_matrix.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.rotation_vector2rotation_matrix.restype = ctypes.c_int

# _lib.construct_skew_matrix.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.construct_skew_matrix.restype = ctypes.c_int

# _lib.quaternion2eulerangles.argtypes = [ctypes.POINTER(ArmMatrixF32), ctypes.POINTER(ArmMatrixF32)]
# _lib.quaternion2eulerangles.restype = ctypes.c_int


# # initialize helpers once
# _lib.initialize_quaternion_helpers()


# # --- linalg wrappers ---
# def get_element(mat, row, col):
# 	return float(_lib.get_element(mat.ptr, int(row), int(col)))


# def set_element(mat, row, col, value):
# 	_lib.set_element(mat.ptr, int(row), int(col), ctypes.c_float(value))


# def init_mat_zeros(mat):
# 	_lib.init_mat_zeros(mat.ptr, mat.rows, mat.cols, mat._buffer)


# def mat_resize(mat, new_rows, new_cols):
# 	return _lib.mat_resize(mat.ptr, int(new_rows), int(new_cols))


# def matscale(mat, scalar):
# 	result = MatrixF32(mat.rows, mat.cols)
# 	_check(_lib.matscale(mat.ptr, ctypes.c_float(scalar), result.ptr), "matscale")
# 	return result


# def matadd(a, b):
# 	result = MatrixF32(a.rows, a.cols)
# 	_check(_lib.matadd(a.ptr, b.ptr, result.ptr), "matadd")
# 	return result


# def matsub(a, b):
# 	result = MatrixF32(a.rows, a.cols)
# 	_check(_lib.matsub(a.ptr, b.ptr, result.ptr), "matsub")
# 	return result


# def matmul(a, b):
# 	result = MatrixF32(a.rows, b.cols)
# 	_check(_lib.matmul(a.ptr, b.ptr, result.ptr), "matmul")
# 	return result


# def mattrans(a):
# 	result = MatrixF32(a.cols, a.rows)
# 	_check(_lib.mattrans(a.ptr, result.ptr), "mattrans")
# 	return result


# def matinv(a):
# 	result = MatrixF32(a.rows, a.cols)
# 	_check(_lib.matinv(a.ptr, result.ptr), "matinv")
# 	return result


# # --- quaternion wrappers ---
# def hamiltonian_product(q1, q2):
# 	result = MatrixF32(4, 1)
# 	_lib.hamiltonian_product(q1.ptr, q2.ptr, result.ptr)
# 	return result


# def conjugate_quaternion(q):
# 	result = MatrixF32(4, 1)
# 	_lib.conjugate_quaternion(q.ptr, result.ptr)
# 	return result


# def quaternion_inverse(q):
# 	result = MatrixF32(4, 1)
# 	_lib.quaternion_inverse(q.ptr, result.ptr)
# 	return result


# def rotation_product(q_target, q_rotator):
# 	result = MatrixF32(4, 1)
# 	_check(_lib.rotation_product(q_target.ptr, q_rotator.ptr, result.ptr), "rotation_product")
# 	return result


# def rotate_vector(vec3, q_rotator):
# 	result = MatrixF32(3, 1)
# 	_check(_lib.rotate_vector(vec3.ptr, q_rotator.ptr, result.ptr), "rotate_vector")
# 	return result


# def rotation_matrix2quaternion(rot_mat):
# 	result = MatrixF32(4, 1)
# 	_check(_lib.rotation_matrix2quaternion(rot_mat.ptr, result.ptr), "rotation_matrix2quaternion")
# 	return result


# def quaternion2rotation_matrix(q):
# 	result = MatrixF32(3, 3)
# 	_check(_lib.quaternion2rotation_matrix(q.ptr, result.ptr), "quaternion2rotation_matrix")
# 	return result


# def rotation_vector2quaternion(rot_vec):
# 	result = MatrixF32(4, 1)
# 	_check(_lib.rotation_vector2quaternion(rot_vec.ptr, result.ptr), "rotation_vector2quaternion")
# 	return result


# def rotation_vector2rotation_matrix(rot_vec):
# 	result = MatrixF32(3, 3)
# 	_check(_lib.rotation_vector2rotation_matrix(rot_vec.ptr, result.ptr), "rotation_vector2rotation_matrix")
# 	return result


# def construct_skew_matrix(vec3):
# 	result = MatrixF32(3, 3)
# 	_check(_lib.construct_skew_matrix(vec3.ptr, result.ptr), "construct_skew_matrix")
# 	return result


# def quaternion2eulerangles(q):
# 	result = MatrixF32(3, 1)
# 	_check(_lib.quaternion2eulerangles(q.ptr, result.ptr), "quaternion2eulerangles")
# 	return result

