


import ctypes
from time import time
import numpy as np
from estimator_wrap import library, Estimator, _as_float_array
from wrap_quat import quaternion2eulerangles, ArmMatrix
from phyphox_reader import read_one_datapoint


IP = "192.168.43.45:8080"
# IP = "172.20.10.1"
host = f"http://{IP}" 


def rotate_sensor_meas(sensor_meas):
    """
    let long axis of phone be x and the short be y
    Is a 90 degree rotation counter clockwise
    """
    x, y, z = sensor_meas
    rotated_90_deg = np.array([y, -x, z])
    return rotated_90_deg

# verify connection
datapoint = read_one_datapoint(host, timeout=1)

# noise in std dev
gns_nois = np.array([1.0]*3) *1e-2  #m
bar_nois = np.array([1.0]) * 1e0# m
acc_nois = np.array([1.0]*3) * 1e0 #m/s^2 
gyr_nois = np.array([1.0]*3) * 1e-3 # m/s
mag_nois = np.array([1.0]*3) *1e-1 # G

# initialize estimator
ofelas = Estimator()
error = library.estimator_init(
    ctypes.byref(ofelas),
    _as_float_array(gns_nois),
    _as_float_array(bar_nois),
    _as_float_array(acc_nois),
    _as_float_array(gyr_nois),
    _as_float_array(mag_nois),
)
if error: print(f"{error = }")

# use a measurment to set references
phyphox_output = read_one_datapoint(host, timeout=0.1)
acc_meas = rotate_sensor_meas(phyphox_output["accelerometer_data"])      #ENU frame
gyr_meas = rotate_sensor_meas(phyphox_output["gyroscope_data"])
mag_meas = rotate_sensor_meas(phyphox_output["magnetometer_data"])       #ENU frame
gns_meas = phyphox_output["gnss_data"]
previous_acc_time = phyphox_output["accelerometer_time"]
acc_time = previous_acc_time

previous_gnss = gns_meas
print(f"{gns_meas = }")

for i in range(3):
    if gns_meas[i] is None:
        gns_meas[i] = 0.0

# use the same reference as the simulator
library.estimator_set_reference(
    ctypes.byref(ofelas),
    ctypes.c_float(gns_meas[2]),
    ctypes.c_float(gns_meas[0]),
    ctypes.c_float(gns_meas[1]),
)
    # ctypes.c_float(0.0),              # reference_height
    # ctypes.c_float(69.6489),      # reference_latitude
    # ctypes.c_float(18.95508),    # reference_longitude

# initialize containers for euler angles
quat_numpy = np.zeros(4)
quat_c = _as_float_array(quat_numpy)
quat_arm_mat = ArmMatrix(4, 1, quat_c)
euler_c = _as_float_array(np.zeros(3))
euler_arm_mat = ArmMatrix(3, 1, euler_c)

# try adding some bias correction
ofelas.eskf.acc_bias.pData[0] = 0.15
ofelas.eskf.acc_bias.pData[1] = 0.2
maxiter = 100
loopcounter = 0
while True:
    # if i == maxiter: break
    
    sensor_before = time()
    phyphox_output = read_one_datapoint(host, timeout=0.1)
    sensor_after = time()

    if phyphox_output is not None:
        acc_meas = rotate_sensor_meas(phyphox_output["accelerometer_data"])      #ENU frame
        gyr_meas = rotate_sensor_meas(phyphox_output["gyroscope_data"])
        mag_meas = rotate_sensor_meas(phyphox_output["magnetometer_data"])       #ENU frame
        gns_meas = phyphox_output["gnss_data"]
        acc_time = phyphox_output["accelerometer_time"]

    delta_t = acc_time - previous_acc_time
    previous_acc_time = acc_time

    # if not np.array_equal(gns_meas, previous_gnss):
    gns_new = 1
        # previous_gnss = gns_meas
    # else:
        # gns_new = 0


    bar_meas = np.array([ofelas.pressure_ref])
    bar_new = 0

    for i in range(3):
        if gns_meas[i] is None:
            print(f"GNSS is none")
            gns_meas[i] = 0.01

    ofelas_reference_c = ctypes.byref(ofelas)
    gns_meas_c = _as_float_array(gns_meas)
    bar_meas_c = _as_float_array(bar_meas)
    acc_meas_c = _as_float_array(acc_meas)
    gyr_meas_c = _as_float_array(gyr_meas)
    mag_meas_c = _as_float_array(mag_meas/ 100) 
    delta_t_acc_c = ctypes.c_float(delta_t)
    delta_t_gyr_c = ctypes.c_float(delta_t)
    gns_new_c = ctypes.c_uint8(gns_new)
    bar_new_c = ctypes.c_uint8(bar_new)
    acc_new_c = ctypes.c_uint8(1)
    gyr_new_c = ctypes.c_uint8(1)
    mag_new_c = ctypes.c_uint8(0)
    enforce_only_predict = ctypes.c_uint8(1) if phyphox_output is None else ctypes.c_uint8(0)

    estimate_before = time()
    library.estimate(
        ofelas_reference_c,
        gns_meas_c,
        bar_meas_c,
        acc_meas_c,
        gyr_meas_c,
        mag_meas_c,
        delta_t_acc_c,
        delta_t_gyr_c,
        gns_new_c,
        bar_new_c, # never any barometer data
        acc_new_c,
        gyr_new_c,
        mag_new_c,
        enforce_only_predict,
    )
    estimate_after = time()

    px = ofelas.eskf.nominal_state.pData[0]
    py = ofelas.eskf.nominal_state.pData[1]
    pz = ofelas.eskf.nominal_state.pData[2]
    vx = ofelas.eskf.nominal_state.pData[3]
    vy = ofelas.eskf.nominal_state.pData[4]
    vz = ofelas.eskf.nominal_state.pData[5]
    qw = ofelas.eskf.nominal_state.pData[6]
    qx = ofelas.eskf.nominal_state.pData[7]
    qy = ofelas.eskf.nominal_state.pData[8]
    qz = ofelas.eskf.nominal_state.pData[9]

    
    quat_arm_mat.pData[0] = qw
    quat_arm_mat.pData[1] = qx
    quat_arm_mat.pData[2] = qy
    quat_arm_mat.pData[3] = qz
    quaternion2eulerangles(quat_arm_mat, euler_arm_mat)

    ro = euler_arm_mat.pData[0]*180/np.pi
    pi = euler_arm_mat.pData[1]*180/np.pi
    ya = euler_arm_mat.pData[2]*180/np.pi

    loopcounter += 1
    if not loopcounter&10==0:
       pass # continue

    print(
        "\033[H\033[J"  # move cursor home + clear screen
        # f"GNS : {gns_meas}\n"
        # f"ACC : {acc_meas}\n"
        # f"GYR : {gyr_meas}\n"
        # f"MAG : {mag_meas}\n"
        f"BAR_eskf : {ofelas.eskf.bar_meas.pData[0]:10.1f}\n"
        f"GNS_eskf : {ofelas.eskf.gns_meas.pData[0]:10.1f}, {ofelas.eskf.gns_meas.pData[1]:10.1f}, {ofelas.eskf.gns_meas.pData[2]:10.1f}\n"
        f"ACC_eskf : {ofelas.eskf.acc_meas.pData[0]:10.1f}, {ofelas.eskf.acc_meas.pData[1]:10.1f}, {ofelas.eskf.acc_meas.pData[2]:10.1f}\n"
        f"GYR_eskf : {ofelas.eskf.gyr_meas.pData[0]:10.1f}, {ofelas.eskf.gyr_meas.pData[1]:10.1f}, {ofelas.eskf.gyr_meas.pData[2]:10.1f}\n"
        f"sensor time   : {(sensor_after-sensor_before)*1e3:10.3f} ms\n"
        f"estimate time : {(estimate_after-estimate_before)*1e3:10.2f} ms\n"
        f"\n"
        f"Position\n"
        f"  East : {px:10.0f}\n"
        f"  North: {py:10.0f}\n"
        f"  Up   : {pz:10.0f}\n"
        f"\n"
        f"Velocity\n"
        f"  East : {vx:10.0f}\n"
        f"  North: {vy:10.0f}\n"
        f"  Up   : {vz:10.0f}\n"
        f"\n"
        f"Attitude\n"
        f"  Roll : {ro:10.0f}°\n"
        f"  Pitch: {pi:10.0f}°\n"
        f"  Yaw  : {ya:10.0f}°\n",
        end="",
        flush=True
    )
