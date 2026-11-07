
import os
import numpy as np
import matplotlib.pyplot as plt
from numpy import array, linspace, pi
from matplotlib.pyplot import subplots, close
from pathlib import Path

# from simulator import main as run_sim
from simulator import (
    sim, 
    plot_measurements,
    spiky,
    sinusoidal,
    spiky_sinusoidal,
)
from ESKF_runner import run_ESKF_thorugh_data, plot_true_vs_ESKF, calculate_rms_error

shockwave_amplitudes = linspace(0, 1000, 100)
shockwave_amplitudes = [500.0]
shockwave_amplitudes = [0.0]

run_new_sims = 1

results_folder = Path("Simulations/results")
figures_folder = Path("Simulations/figures")


class Experiement:
    def __init__(self, sim_config, eskf_config):

        self.sim_config = sim_config
        self.eskf_config = eskf_config        

    def run_sim(self, plot=True):
        self.data = sim(self.sim_config)
        if plot:
            plot_measurements(
                time=self.data["time"],
                gns_measurements=self.data["gns_measurements"],
                bar_measurements=self.data["bar_measurements"],
                acc_measurements=self.data["acc_measurements"],
                gyr_measurements=self.data["gyr_measurements"],
                mag_measurements=self.data["mag_measurements"],
                sim_config=self.sim_config
            )

        return self.data
    
    def run_eskf(self, plot=True):

        self.results = run_ESKF_thorugh_data(self.data, eskf_config=self.eskf_config)

        if plot:
            plot_true_vs_ESKF(self.data, self.results, self.sim_config)

        return self.results

    def run_errors(self):
        self.rms_errors = calculate_rms_error(
            true_position=self.data["position"],
            true_velocity=self.data["velocity"],
            true_attitude=self.data["attitude"],
            estimated_position=self.results["position"],
            estimated_velocity=self.results["velocity"],
            estimated_attitude=self.results["attitude"],
        )
        return self.rms_errors
    
sim_config = {
    "TIME_STEP":                    1e-2,
    "MAX_ITERATIONS":               100000,
    "shockwave_amplitude":          0.0,
    "GRAVITY":                      9.82,                                        # m/s2
    "THRUST":                       5e3,                                         # N
    "ROCKET_MASS":                  40.0,                                        # kg
    "DRAG_COEFFICIENCT_REDUCED":    0.01,                                        
    "IDLE_TIME":                    10.0,                                        # s
    "BOOST_TIME":                   5.0,                                         # s
    "ROCKET_INITIAL_QUAT":          array([1.0, 0.0, 0.0, 0.0]),
    "ROCKET_INITIAL_PITCH":         0.0,
    "SPEED_OF_SOUND":               340,                                         # m/s
    "shockwave":                    spiky_sinusoidal,
    "GNSS_VARIANCE":                array([10.0]*3)**2  /100,                   # m^2
    "BAROMETER_VARIANCE":           10**2             /100,                     # Pa^2
    "ACCELEROMETER_VARIANCE":       array([0.5]*3)**2  /1,                      # m/s^2 ^2 
    "GYROSCOPE_VARIANCE":           array([0.03]*3)**2 /1,                      # rad/s ^2
    "MAGNETOMETER_VARIANCE":        array([1.0]*3)**2  /1,                      # µT ^2
    "ACCELEROMETER_BIAS":           array([0.2, 0.1, 0.3])  * 0.0,              # m/s^2
    "GYROSCOPE_BIAS":               array([0.2, 0.0, 0.0]) * np.pi/180  * 0.0,  # rad/s
}

eskf_config = {
    # noise in std dev
    "gns_nois": np.array([5.0]*3) * 1e1,   # m
    "bar_nois": np.array([5.0])   * 1e1,   # m
    "acc_nois": np.array([1.0]*3) * 1e0,   # m/s^2 
    "gyr_nois": np.array([1.0]*3) * 1e-3,  # m/s
    "mag_nois": np.array([1.0]*3) * 1e-1,  # G
}

def __main__():
    experiment = Experiement(
        sim_config=sim_config, 
        eskf_config=eskf_config
    )
    experiment.run_sim(plot=True)
    experiment.run_eskf(plot=True)
    errors = experiment.run_errors()
    print(f"\n\nESKF rms errors: ")
    for key, value in errors["rms"].items():
        unit = errors["units"][key]
        if key=="attitude_angle":
            value = value * 180/np.pi
            unit = "deg"
        print(f"\t{key:<15}: {value:<10.4} {unit:<5}")
    print("\n")
    return 
    ex = Experiement(sim_config, eskf_config)
    collected_errors = []
    for _ in range(100):
        ex.run_sim(plot=False)
        ex.run_eskf(plot=False)
        errors = ex.run_errors()
        collected_errors.append(errors)

    # collected_errors = np.array(collected_errors)
    position_error = np.array([collected_errors[i]["rms"]["position"] for i in range(len(collected_errors))])
    velocity_error = np.array([collected_errors[i]["rms"]["velocity"] for i in range(len(collected_errors))])
    attitude_quat_error = np.array([collected_errors[i]["rms"]["attitude_quat"] for i in range(len(collected_errors))])
    attitude_angle_error = np.array([collected_errors[i]["rms"]["attitude_angle"] for i in range(len(collected_errors))])
    fig, axs=plt.subplots(4, 1, figsize=(10, 10))
    axs[0].hist(position_error)
    axs[1].hist(velocity_error)
    axs[2].hist(attitude_quat_error)
    axs[3].hist(attitude_angle_error * 180/np.pi)
    axs[0].set_title("position error")
    axs[1].set_title("velocity error")
    axs[2].set_title("attitude quat error")
    axs[3].set_title("attitude angle error")
    axs[0].set_ylabel("Error [m]")
    axs[1].set_ylabel("Error [m/s]")
    axs[2].set_ylabel("Error [--]")
    axs[3].set_ylabel("Error [deg]")
    fig.tight_layout()
    fig.savefig(Path("figures") / "rms.png")

if __name__ == "__main__":
    __main__()
