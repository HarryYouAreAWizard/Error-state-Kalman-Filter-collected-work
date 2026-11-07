

import time as time_module # coincides with a simulation variable
import numpy as np
import matplotlib.pyplot as plt

from numpy import pi

from sim_src.quaternion_math import quaternion_to_euler_angles
# from simulator import SPEED_OF_SOUND

figure_folder = r"figures"
data_folder = r"Simulations/data"

GRAVITY = 9.8125

# # noise in std dev
# gns_nois = np.array([5.0]*3) *1e1  #m
# bar_nois = np.array([5.0]) * 1e1# m
# acc_nois = np.array([1.0]*3) * 1e0 #m/s^2 
# gyr_nois = np.array([1.0]*3) * 1e-3 # m/s
# mag_nois = np.array([1.0]*3) *1e-1 # G


# running pure C ESKF
import ctypes
from estimator_wrap import library, Estimator, _as_float_array, _matrix_to_numpy



def calculate_rms_error(
        true_position, 
        true_velocity, 
        true_attitude, 
        estimated_position, 
        estimated_velocity, 
        estimated_attitude
    ):
    """calculate post 'flight' rms error"""
    position_error = np.linalg.norm(true_position - estimated_position, axis=1)
    velocity_error = np.linalg.norm(true_velocity - estimated_velocity, axis=1)
    attitude_error_quat = np.linalg.norm(true_attitude - estimated_attitude, axis=1)
    
    true_euler_angles = np.array([quaternion_to_euler_angles(true_quaternion) for true_quaternion in true_attitude])
    estimated_euler_angles = np.array([quaternion_to_euler_angles(attitude) for attitude in estimated_attitude])
    euler_delta = (true_euler_angles - estimated_euler_angles + np.pi) % (2.0 * np.pi) - np.pi
    attitude_error_angle = np.linalg.norm(euler_delta, axis=1)
    
    rms_position_error = np.sqrt(np.mean(position_error**2))
    rms_velocity_error = np.sqrt(np.mean(velocity_error**2))
    rms_attitude_error_quat = np.sqrt(np.mean(attitude_error_quat**2))
    rms_attitude_error_angle = np.sqrt(np.mean(attitude_error_angle**2))
    
    return {
        "rms": {
            "position": rms_position_error,
            "velocity": rms_velocity_error, 
            "attitude_quat": rms_attitude_error_quat, 
            "attitude_angle": rms_attitude_error_angle,
        },
        "timeseries": {
            "position": attitude_error_angle,
            "velocity": attitude_error_angle,
            "attitude_quat": attitude_error_angle,
            "attitude_angle": attitude_error_angle,
        },
        "units": {
            "position": "m",
            "velocity": "m/s",
            "attitude_quat": "--",
            "attitude_angle": "rad",
        }
    }

def run_ESKF_thorugh_data(data, eskf_config):
    gns_nois = eskf_config["gns_nois"]
    acc_nois = eskf_config["acc_nois"]
    gyr_nois = eskf_config["gyr_nois"]
    mag_nois = eskf_config["mag_nois"]
    bar_nois = eskf_config["bar_nois"]
    # convert the noise arrays to c-compatible arrays
    gns_nois_c = _as_float_array(gns_nois)
    acc_nois_c = _as_float_array(acc_nois)
    gyr_nois_c = _as_float_array(gyr_nois)
    mag_nois_c = _as_float_array(mag_nois)
    bar_nois_c = _as_float_array(bar_nois)

    # initialize estimator
    est = Estimator()
    error = library.estimator_init(
        ctypes.byref(est),
        gns_nois_c,
        acc_nois_c,
        gyr_nois_c,
        mag_nois_c,
        bar_nois_c
    )
    if error: print(f"{error = }")

    # use the same reference as the simulator
    library.estimator_set_reference(
        ctypes.byref(est),
        ctypes.c_float(0.0),              # reference_height
        ctypes.c_float(69.6489),      # reference_latitude
        ctypes.c_float(18.95508),    # reference_longitude
    )
    
    previous_acc_time = 0.0
    previous_gyr_time = 0.0

    # containers
    positions = []
    velocities = []
    attitudes = []
    detected_launch_time = None
    detected_apogee_time = None
    estimate_times = []

    # run
    for i in range(len(data["time"])):

        # get measurements
        time = data["time"][i]
        bar_meas = data["bar_measurements"][i]
        acc_meas = data["acc_measurements"][i]
        gyr_meas = data["gyr_measurements"][i]
        mag_meas = data["mag_measurements"][i]

        # barometer treated as arrays in estimator, even though it is only one number
        bar_meas=np.array([bar_meas])

        # GNSS only available with freq ~ 1 Hz
        if i%100 == 0:
            gns_new = 1
            gns_meas = data["gns_measurements"][i]
        else:
            gns_new = 0

        # realistically, the following sensors will always have a new measurement ready
        bar_new = 1
        acc_new = 1
        gyr_new = 1
        mag_new = 0
        
        delta_t_acc = time - previous_acc_time
        previous_acc_time = time
        delta_t_gyr = time - previous_gyr_time
        previous_gyr_time = time

        time_start = time_module.time()
        ofelas_reference_c = ctypes.byref(est)
        gns_meas_c = _as_float_array(gns_meas)
        bar_meas_c = _as_float_array(bar_meas)
        acc_meas_c = _as_float_array(acc_meas)
        gyr_meas_c = _as_float_array(gyr_meas)
        mag_meas_c = _as_float_array(mag_meas)
        delta_t_acc_c = ctypes.c_float(delta_t_acc)
        delta_t_gyr_c = ctypes.c_float(delta_t_gyr)
        gns_new_c = ctypes.c_uint8(gns_new)
        bar_new_c = ctypes.c_uint8(bar_new)
        acc_new_c = ctypes.c_uint8(acc_new)
        gyr_new_c = ctypes.c_uint8(gyr_new)
        mag_new_c = ctypes.c_uint8(mag_new)
        enforce_only_predict = ctypes.c_uint8(0)

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
        time_end = time_module.time()
        if error: print(f"{error = }")
        delta_time = time_end - time_start

        estimate_times.append(delta_time)

        # launch and apogee detection logic
        if detected_launch_time is None and (np.linalg.norm(acc_meas)/GRAVITY > 5 or est.eskf.nominal_state.pData[3+2] > 50): 
            detected_launch_time = time
        if detected_launch_time is not None:
            if detected_apogee_time is None and time-detected_launch_time>1.0 and est.eskf.nominal_state.pData[3+2] < 1.0:
                detected_apogee_time = time

        position = np.zeros(3)
        velocity = np.zeros(3)
        attitude = np.zeros(4)

        for j in range(3):
            position[j] = est.eskf.nominal_state.pData[j]
            velocity[j] = est.eskf.nominal_state.pData[j+3]

        for j in range(4):
            attitude[j] = est.eskf.nominal_state.pData[j+6]

        positions.append(position)
        velocities.append(velocity)
        attitudes.append(attitude)

        if np.isnan(est.eskf.nominal_state.pData[2]):
            print(f"NaN detected in position estimate, stopping ESKF run. {i=}")
            break
        print(f"{i :<5} / {len(data["time"])}         ", end="\r")


    # ensure "detected_apogee_time" is set if not detected
    if detected_apogee_time is None:
        detected_apogee_time = data["time"][-1]
    if detected_launch_time is None:
        detected_launch_time = data["time"][0]

    # print(f"Average estimation time per step: {np.mean(estimate_times)*1e3:.5f} ms")
    
    true_launch_condition = data["velocity"][:, 2] > 0
    true_launch_index = np.where(true_launch_condition)[0][0]
    true_launch_time = data["time"][true_launch_index]

    true_apogee_condition = data["position"][:, 2] == np.max(data["position"][:, 2])
    true_apogee_index = np.where(true_apogee_condition)[0][0]
    true_apogee_time = data["time"][true_apogee_index]

    detected_launch_time_err = abs(true_launch_time - detected_launch_time)
    detected_apogee_time_err = abs(true_apogee_time - detected_apogee_time)

    positions = np.array(positions)
    velocities = np.array(velocities)
    attitudes = np.array(attitudes)
    return {
        "position":positions, 
        "velocity":velocities, 
        "attitude":attitudes, 
        "detected_launch_time":detected_launch_time, 
        "detected_apogee_time":detected_apogee_time, 
        "detected_launch_time_err":detected_launch_time_err, 
        "detected_apogee_time_err":detected_apogee_time_err,
    }


def plot_true_vs_ESKF(data, states, sim_config, title=""):
    time = data["time"]
    positions_ESKF = states["position"] 
    velocities_ESKF = states["velocity"] 
    attitudes_ESKF = states["attitude"]
    detected_launch_time = states["detected_launch_time"]
    detected_apogee_time = states["detected_apogee_time"]
    euler_angles_ESKF = np.array([quaternion_to_euler_angles(attitude) for attitude in attitudes_ESKF])
    true_quaternions = data["attitude"]
    SPEED_OF_SOUND = sim_config["SPEED_OF_SOUND"]
    
    true_euler_angles = np.array([quaternion_to_euler_angles(true_quaternion) for true_quaternion in true_quaternions])
    true_roll  = true_euler_angles[:, 0]
    true_pitch = true_euler_angles[:, 1]
    true_yaw   = true_euler_angles[:, 2]

    fig, axs=plt.subplots(4,figsize=(10, 18))
    axs[0].set_title("Position")
    axs[0].plot(time, positions_ESKF[:, 0], label="x ESKF")
    axs[0].plot(time, positions_ESKF[:, 1], label="y ESKF")
    axs[0].plot(time, positions_ESKF[:, 2], label="z ESKF")
    axs[0].plot(time, data["position"][:, 0], label="x true", ls="--")
    axs[0].plot(time, data["position"][:, 1], label="y true", ls="--")
    axs[0].plot(time, data["position"][:, 2], label="z true", ls="--")
    axs[0].set_ylabel(r"$p$ [m]")
    axs[0].vlines(detected_launch_time, min(data["position"][:, 2])-1000, max(data["position"][:, 2]))
    axs[0].vlines(detected_apogee_time, min(data["position"][:, 2])-1000, max(data["position"][:, 2]))
    axs[0].text(detected_launch_time+1, min(data["position"][:, 2])-1000, "launch")
    axs[0].text(detected_apogee_time+1, min(data["position"][:, 2])-1000, "apogee")    
    axs[1].set_title("Velocity")
    axs[1].plot(time, velocities_ESKF[:, 0], label="x ESKF")
    axs[1].plot(time, velocities_ESKF[:, 1], label="y ESKF")
    axs[1].plot(time, velocities_ESKF[:, 2], label="z ESKF")
    axs[1].plot(time, data["velocity"][:, 0], label="x true", ls="--")
    axs[1].plot(time, data["velocity"][:, 1], label="y true", ls="--")
    axs[1].plot(time, data["velocity"][:, 2], label="z true", ls="--")
    axs[1].set_ylabel(r"$v$ [$\frac{\text{m}}{\text{s}}$]")
    axs[1].hlines(0, data["time"][0], data["time"][-1], color="k")
    axs[1].hlines(SPEED_OF_SOUND, data["time"][0], data["time"][-1], color="k", ls="--")    
    axs[2].set_title("Attitude (Euler angles)")
    axs[2].plot(time, 180/pi*euler_angles_ESKF[:, 0], label="roll ESKF")
    axs[2].plot(time, 180/pi*euler_angles_ESKF[:, 1], label="pitch ESKF")
    axs[2].plot(time, 180/pi*euler_angles_ESKF[:, 2], label="yaw ESKF")
    axs[2].plot(time, 180/pi*true_roll, label="roll true", ls="--")
    axs[2].plot(time, 180/pi*true_pitch, label="pitch true", ls="--")
    axs[2].plot(time, 180/pi*true_yaw, label="yaw true", ls="--")
    axs[2].set_ylabel(r"$\theta$ [$\degree$]")
    axs[3].set_title("Attitude (quaternions)")
    axs[3].plot(time, attitudes_ESKF[:, 0], label="qw ESKF")
    axs[3].plot(time, attitudes_ESKF[:, 1], label="qx ESKF")
    axs[3].plot(time, attitudes_ESKF[:, 2], label="qy ESKF")
    axs[3].plot(time, attitudes_ESKF[:, 3], label="qz ESKF")
    axs[3].plot(time, true_quaternions[:, 0], label="qw true", ls="--")
    axs[3].plot(time, true_quaternions[:, 1], label="qx true", ls="--")
    axs[3].plot(time, true_quaternions[:, 2], label="qy true", ls="--")
    axs[3].plot(time, true_quaternions[:, 3], label="qz true", ls="--")
    axs[3].set_ylabel(r"$q$")
    for ax in axs.flatten():ax.legend()
    fig.supxlabel("Time [s]")
    fig.suptitle("Latest experiment")
    fig.tight_layout()
    fig.savefig(figure_folder + "/" + "ESKF_true_vs_estimated" + f"{title}" + ".png")
    plt.close()
