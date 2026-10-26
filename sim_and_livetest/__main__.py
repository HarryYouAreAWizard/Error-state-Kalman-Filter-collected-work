
import os
import numpy as np
import matplotlib.pyplot as plt
from numpy import array, linspace, pi
from matplotlib.pyplot import subplots, close

from simulator import main as run_sim
from ESKF_runner import main as run_ESKF

shockwave_amplitudes = linspace(0, 1000, 100)
shockwave_amplitudes = [500.0]
shockwave_amplitudes = [0.0]

run_new_sims = 1

results_folder = r"Simulations/results"
figures_folder = r"Simulations/figures"


def load_errors(file_name):
    doc = open("results" + "/" + file_name)
    amp = []
    pos_err = []
    vel_err = []
    att_err_q = []
    att_err_a = []
    launch_detection_errs = []
    apogee_detection_errs = []
    for line in doc:
        a, b, c, d, e, f, g = line.split(",")
        amp.append(float(a))
        pos_err.append(float(b))
        vel_err.append(float(c))
        att_err_q.append(float(d))
        att_err_a.append(float(e))
        launch_detection_errs.append(float(f))
        apogee_detection_errs.append(float(g))

    amp = array(amp)
    pos_err = array(pos_err)
    vel_err = array(vel_err)
    att_err_q = array(att_err_q)
    att_err_a = array(att_err_a)
    return (
        amp,
        pos_err,
        vel_err,
        att_err_q,
        att_err_a,
        launch_detection_errs,
        apogee_detection_errs,
    )

def plot_errors():
    data = load_errors("results.txt")
    amp, pos_err, vel_err, att_err_q, att_err_a, lau_dec_err, apo_dec_err = data

    fig, axs=subplots(4, 1, figsize=(8, 8))
    axs[0].set_title("pos error")
    axs[0].set_ylabel(r"RMS $\Delta p$ [m]")
    axs[0].scatter(amp, pos_err)
    axs[1].set_title("vel error")
    axs[1].set_ylabel(r"RMS $\Delta v$ [m/s]")
    axs[1].scatter(amp, vel_err)
    axs[2].set_ylabel(r"RMS $\Delta \theta$ [deg]")
        # axs[2].set_ylabel(r"$\sqrt{\left( q^{-1} \otimes \hat q \right)^2}$ [$\degree$]")
    axs[2].set_title("att error")
    axs[2].scatter(amp, att_err_a*180/pi)
    axs[3].set_title("Launch and apogee detection error")
    axs[3].set_ylabel(r"$\Delta t$ [s]")
    axs[3].scatter(amp, lau_dec_err, label="launch")
    axs[3].scatter(amp, apo_dec_err, label="apogee")
    axs[3].legend()
    fig.suptitle("RMS errors")
    fig.supxlabel("Pressure perturbation amplitude [Pa]")
    fig.tight_layout()
    fig.savefig("figures" + "/" + "Shockwave simulations.png")
    close()

def create_data(live_plot=False):
    
    doc = open("results" + "/" + "results.txt", mode="w")
    
    pos_errors = []
    vel_errors = []
    att_errors_q = []
    att_errors_a = []
    detected_launche_times = []
    detected_apogee_times = []
    for amp in shockwave_amplitudes:
        data_file = f"sim_data-{amp}"
        if run_new_sims:
            sim_data = run_sim(shockwave_amplitude=amp)
        print(f"Running ESKF on {data_file}...")
        pos_error, vel_error, att_error_q, att_error_a, dlt, dat = run_ESKF(sim_data, data_file)
        pos_errors.append(pos_error)
        vel_errors.append(vel_error)
        att_errors_q.append(att_error_q)
        att_errors_a.append(att_error_a)
        detected_launche_times.append(dlt)
        detected_apogee_times.append(dat)

        
        doc.write(
            f"{amp}"         + "," +
            f"{pos_error}"   + "," + 
            f"{vel_error}"   + "," + 
            f"{att_error_q}" + "," +
            f"{att_error_q}" + "," +
            f"{dlt}" + "," +
            f"{dat}" + "\n"

        )
        if live_plot:
            doc.close()
            plot_errors()
            doc = open("results" + "/" + "results.txt", mode="a")
    doc.close()
    del pos_errors
    del vel_errors
    del att_errors_q
    del att_errors_a
    del detected_launche_times
    del detected_apogee_times    


def __main__():
    create_data(live_plot=True)
    plot_errors()
    plt.show()
if __name__ == "__main__":
    __main__()
