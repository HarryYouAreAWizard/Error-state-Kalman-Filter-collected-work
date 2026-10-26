




import ctypes
import numpy as np

def load_library(path=None):
    lib_path = "/home/noah/NEXT/estimator_stand_alone_builder/Error-State-Kalman-Filter-standalone/libmain.so"
    lib_path = "/home/noah/NEXT/ESKF_shockwave_simulations/Error-State-Kalman-Filter-standalone/libmain.so"
    lib_path = "/home/noah/NEXT/Error-State-Kalman-Filter-standalone/libmain.so"
    if path is not None:
        lib_path = path

    return ctypes.CDLL(lib_path)


library = load_library()


class arm_matrix_instance_f32(ctypes.Structure):
    _fields_ = [
        ("numRows", ctypes.c_uint16),
        ("numCols", ctypes.c_uint16),
        ("pData", ctypes.POINTER(ctypes.c_float)),
    ]

MAXROWS = 10
MAXCOLS = 10

class ErrorStateKalmanFilter(ctypes.Structure):
    _fields_ = [
        ("pos", arm_matrix_instance_f32),
        ("vel", arm_matrix_instance_f32),
        ("att", arm_matrix_instance_f32),
        ("pos_error", arm_matrix_instance_f32),
        ("vel_error", arm_matrix_instance_f32),
        ("att_error", arm_matrix_instance_f32),
        ("nominal_state", arm_matrix_instance_f32),
        ("error_state", arm_matrix_instance_f32),
        ("covariance", arm_matrix_instance_f32),
        ("gravity_vector_unnormalized", arm_matrix_instance_f32),
        ("gravity_vector_normalized", arm_matrix_instance_f32),
        ("expected_gravity_unnormalized", arm_matrix_instance_f32),
        ("expected_gravity_normalized", arm_matrix_instance_f32),
        ("magnetic_vector_unnormalized", arm_matrix_instance_f32),
        ("magnetic_vector_normalized", arm_matrix_instance_f32),
        ("expected_magnetic_field_normalized", arm_matrix_instance_f32),
        ("acc_bias", arm_matrix_instance_f32),
        ("gyr_bias", arm_matrix_instance_f32),
        ("gns_nois", arm_matrix_instance_f32),
        ("bar_nois", arm_matrix_instance_f32),
        ("acc_nois", arm_matrix_instance_f32),
        ("gyr_nois", arm_matrix_instance_f32),
        ("mag_nois", arm_matrix_instance_f32),
        ("acc_bias_nois", arm_matrix_instance_f32),
        ("gyr_bias_nois", arm_matrix_instance_f32),
        ("gns_meas", arm_matrix_instance_f32),
        ("bar_meas", arm_matrix_instance_f32),
        ("acc_meas", arm_matrix_instance_f32),
        ("acc_meas_normalized", arm_matrix_instance_f32),
        ("mag_meas", arm_matrix_instance_f32),
        ("mag_meas_normalized", arm_matrix_instance_f32),
        ("gyr_meas", arm_matrix_instance_f32),
        ("acc_nois_normalized", arm_matrix_instance_f32),
        ("mag_nois_normalized", arm_matrix_instance_f32),
        ("R_att", arm_matrix_instance_f32),
        ("rotation_vector", arm_matrix_instance_f32),
        ("true_acc", arm_matrix_instance_f32),
        ("q_gyr", arm_matrix_instance_f32),
        ("R_gyr", arm_matrix_instance_f32),
        ("R_gyr_T", arm_matrix_instance_f32),
        ("rotated_acceleration_skew_matrix", arm_matrix_instance_f32),
        ("Vi", arm_matrix_instance_f32),
        ("Thetai", arm_matrix_instance_f32),
        ("Fx", arm_matrix_instance_f32),
        ("Fi", arm_matrix_instance_f32),
        ("Qi", arm_matrix_instance_f32),
        ("I3", arm_matrix_instance_f32),
        ("O3", arm_matrix_instance_f32),
        ("OOI", arm_matrix_instance_f32),
        ("O13", arm_matrix_instance_f32),
        ("O14", arm_matrix_instance_f32),
        ("O43", arm_matrix_instance_f32),
        ("I9", arm_matrix_instance_f32),
        ("dadq", arm_matrix_instance_f32),
        ("dmdq", arm_matrix_instance_f32),
        ("V_gns", arm_matrix_instance_f32),
        ("V_bar", arm_matrix_instance_f32),
        ("V_acc", arm_matrix_instance_f32),
        ("V_mag", arm_matrix_instance_f32),
        ("measurement_vector", arm_matrix_instance_f32),
        ("h", arm_matrix_instance_f32),
        ("Hx", arm_matrix_instance_f32),
        ("V", arm_matrix_instance_f32),
        ("Q_angular_error", arm_matrix_instance_f32),
        ("X_angular_error", arm_matrix_instance_f32),
        ("H", arm_matrix_instance_f32),
        ("H_T", arm_matrix_instance_f32),
        ("S", arm_matrix_instance_f32),
        ("Kalman_gain", arm_matrix_instance_f32),
        ("innovation", arm_matrix_instance_f32),
        ("total_error", arm_matrix_instance_f32),
        ("q_error", arm_matrix_instance_f32),
        ("att_skew_matrix", arm_matrix_instance_f32),
        ("G", arm_matrix_instance_f32),
        ("intmed1", arm_matrix_instance_f32),
        ("intmed2", arm_matrix_instance_f32),
        ("intmed3", arm_matrix_instance_f32),
        ("intmed4", arm_matrix_instance_f32),

        # // initialize data buffers
        ("pos_buffer", ctypes.c_float * 3*1),
        ("vel_buffer", ctypes.c_float * 3*1),
        ("att_buffer", ctypes.c_float * 4*1),
        ("pos_error_buffer", ctypes.c_float * 3*1),
        ("vel_error_buffer", ctypes.c_float * 3*1),
        ("att_error_buffer", ctypes.c_float * 3*1),
        ("nominal_state_buffer", ctypes.c_float * 10*1),
        ("error_state_buffer", ctypes.c_float * 10*1),
        ("covariance_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("gravity_vector_unnormalized_buffer", ctypes.c_float * 3*1),
        ("gravity_vector_normalized_buffer", ctypes.c_float * 3*1),
        ("expected_gravity_unnormalized_buffer", ctypes.c_float * 3*1),
        ("expected_gravity_normalized_buffer", ctypes.c_float * 3*1),
        ("magnetic_vector_unnormalized_buffer", ctypes.c_float * 3*1),
        ("magnetic_vector_normalized_buffer", ctypes.c_float * 3*1),
        ("expected_magnetic_field_normalized_buffer", ctypes.c_float * 3*1),
        ("acc_bias_buffer", ctypes.c_float * 3*1),
        ("gyr_bias_buffer", ctypes.c_float * 3*1),
        ("gns_nois_buffer", ctypes.c_float * 3*1),
        ("bar_nois_buffer", ctypes.c_float * 3*1),
        ("acc_nois_buffer", ctypes.c_float * 3*1),
        ("gyr_nois_buffer", ctypes.c_float * 3*1),
        ("mag_nois_buffer", ctypes.c_float * 3*1),
        ("acc_bias_nois_buffer", ctypes.c_float * 3*1),
        ("gyr_bias_nois_buffer", ctypes.c_float * 3*1),
        ("gns_meas_buffer", ctypes.c_float * 3*1),
        ("bar_meas_buffer", ctypes.c_float * 3*1),
        ("acc_meas_buffer", ctypes.c_float * 3*1),
        ("acc_meas_normalized_buffer", ctypes.c_float * 3*1),
        ("mag_meas_buffer", ctypes.c_float * 3*1),
        ("mag_meas_normalized_buffer", ctypes.c_float * 3*1),
        ("gyr_meas_buffer", ctypes.c_float * 3*1),
        ("acc_nois_normalized_buffer", ctypes.c_float * 3*1),
        ("mag_nois_normalized_buffer", ctypes.c_float * 3*1),
        ("R_att_buffer", ctypes.c_float * 3*3),
        ("rotation_vector_buffer", ctypes.c_float * 3*1),
        ("true_acc_buffer", ctypes.c_float * 3*1),
        ("q_gyr_buffer", ctypes.c_float * 4*1),
        ("R_gyr_buffer", ctypes.c_float * 3*3),
        ("R_gyr_T_buffer", ctypes.c_float * 3*3),
        ("rotated_acceleration_skew_matrix_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("Vi_buffer", ctypes.c_float * 3*3),
        ("Thetai_buffer", ctypes.c_float * 3*3),
        ("Fx_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("Fi_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("Qi_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("I3_buffer", ctypes.c_float * 3*3),
        ("O3_buffer", ctypes.c_float * 3*3),
        ("OOI_buffer", ctypes.c_float * 1*3),
        ("O13_buffer", ctypes.c_float * 1*3),
        ("O14_buffer", ctypes.c_float * 1*4),
        ("O43_buffer", ctypes.c_float * 4*3),
        ("I9_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("dadq_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("dmdq_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("V_gns_buffer", ctypes.c_float * 3*3),
        ("V_bar_buffer", ctypes.c_float * 3*3),
        ("V_acc_buffer", ctypes.c_float * 3*3),
        ("V_mag_buffer", ctypes.c_float * 3*3),
        ("measurement_vector_buffer", ctypes.c_float * MAXROWS*1),
        ("h_buffer", ctypes.c_float * MAXROWS*1),
        ("Hx_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("V_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("Q_angular_error_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("X_angular_error_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("H_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("H_T_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("S_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("Kalman_gain_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("innovation_buffer", ctypes.c_float * MAXROWS*1),
        ("total_error_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("q_error_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("att_skew_matrix_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("G_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("intmed1_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("intmed2_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("intmed3_buffer", ctypes.c_float * MAXROWS*MAXCOLS),
        ("intmed4_buffer", ctypes.c_float * MAXROWS*MAXCOLS),

        ("acc_noise_update_prescaler", ctypes.c_float),
    ]


class Estimator(ctypes.Structure):
    _fields_ = [
        ("eskf", ErrorStateKalmanFilter),
        ("gns_new", ctypes.c_uint8),
        ("bar_new", ctypes.c_uint8),
        ("acc_new", ctypes.c_uint8),
        ("gyr_new", ctypes.c_uint8),
        ("mag_new", ctypes.c_uint8),
        ("lat_ref", ctypes.c_float),
        ("lon_ref", ctypes.c_float),
        ("alt_ref", ctypes.c_float),
        ("pressure_ref", ctypes.c_float),
        ("temperature_ref", ctypes.c_float),
        ("gnss_meas_local", ctypes.c_float * 3),
        ("bar_meas_alt_m", ctypes.c_float * 1),
        ("nominal_state_backup", arm_matrix_instance_f32),
        ("covariance_matric_backup", arm_matrix_instance_f32),
        ("nominal_state_backup_buffer", ctypes.c_float * 10*1),
        ("covariance_matric_backup_buffer", ctypes.c_float * 9 * 9),
]


library.estimator_init.argtypes = [
    ctypes.POINTER(Estimator),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float)
]
library.estimator_init.restype = int


library.estimator_set_reference.argtypes = [
    ctypes.POINTER(Estimator),
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
]

library.estimator_set_reference.restype = None

library.estimate.argtypes = [
    ctypes.POINTER(Estimator),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_uint8,
    ctypes.c_uint8,
    ctypes.c_uint8,
    ctypes.c_uint8,
    ctypes.c_uint8,
    ctypes.c_uint8
]
library.estimate.restype = int


def _as_float_array(values):
    return (ctypes.c_float * len(values))(*values)


def _matrix_to_numpy(matrix):
    length = matrix.numRows * matrix.numCols
    L = [matrix.pData[i] for i in range(length)]
    return np.array(L)

# example main
def main():
    estimator = Estimator()

    gnss_noise = _as_float_array([1.0, 1.0, 1.0])
    accelerometer_noise = _as_float_array([1.0, 1.0, 2.0])
    gyroscope_noise = _as_float_array([1.0, 1.0, 1.0])
    magnetometer_noise = _as_float_array([1.0, 1.0, 1.0])
    barometer_noise = _as_float_array([1.0])

    library.estimator_init(
        ctypes.byref(estimator),
        gnss_noise,
        accelerometer_noise,
        gyroscope_noise,
        magnetometer_noise,
        barometer_noise,
    )

    gns_meas = _as_float_array([0.0, 0.0, 0.0])
    bar_meas = _as_float_array([0.0, 0.0, 0.0])
    acc_meas = _as_float_array([1.0, 0.0, 9.82])
    gyr_meas = _as_float_array([0.1, 0.2, 0.1])
    mag_meas = _as_float_array([0.0, 0.0, 0.0])

    library.estimate(
        ctypes.byref(estimator),
        gns_meas,
        bar_meas,
        acc_meas,
        gyr_meas,
        mag_meas,
        ctypes.c_float(0.1),
        ctypes.c_float(0.2),
        ctypes.c_uint8(0),
        ctypes.c_uint8(0),
        ctypes.c_uint8(1),
        ctypes.c_uint8(1),
        ctypes.c_uint8(0),
    )

    print("nominal_state:", _matrix_to_numpy(estimator.eskf.nominal_state)[:5])
    print("covariance shape:", estimator.eskf.covariance.numRows, estimator.eskf.covariance.numCols)


# if __name__ == "__main__":
    # main()
    