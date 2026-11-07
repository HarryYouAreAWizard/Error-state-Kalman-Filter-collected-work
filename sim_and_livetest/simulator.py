
import numpy as np
# import matplotlib.pyplot as plt
from matplotlib.pyplot import subplots, close

from numpy import array, pi, sqrt, cos, sin

from sim_src.util import euler_angles_to_quaternion, force_components_world, quaternion_derivative
from sim_src.quaternion_math import quaternion_to_euler_angles, rotate_vector, inverse_quaternion


from sim_src.sensor_models import GNSS, Barometer, Accelerometer, Gyroscope, Magnetometer
from sim_src.shockwave_models import sinusoidal, spiky, spiky_sinusoidal

figure_folder = r"figures"
data_folder = r"Simulations\data"
# figure_folder = r"C:\Users\NoahH\OneDrive - UiT Office 365\Personal OneDrive copy\Tromsø\UiT Arctic University of Norway\Ignite\NEXT\Simulations\figures"
# data_folder = r"C:\Users\NoahH\OneDrive - UiT Office 365\Personal OneDrive copy\Tromsø\UiT Arctic University of Norway\Ignite\NEXT\Simulations\data"

# core simulation solver
def rungekutta_step(f:'function', tn:float, Qn:array, sim_config):
    '''One iteration of the Runge-Kutta method
    
    Parameters:
    f (function): The derivative of y. f = dydt(t, y) 
    tn (float): current time step
    Qn (numpy array): an array of variables in the format (x, dxdt, y, dydt, z, dzdt). Can include more variables'''
    time_step=sim_config["TIME_STEP"]
    k1 = f(tn, Qn, sim_config)
    k2 = f(tn + time_step/2, Qn + time_step*k1/2, sim_config)
    k3 = f(tn + time_step/2, Qn + time_step*k2/2, sim_config)
    k4 = f(tn + time_step, Qn +   time_step*k3, sim_config)
    
    tnext = tn + time_step
    Qnext = Qn + time_step/6 * (k1+ 2*k2 + 2*k3 + k4)

    return tnext, Qnext



# ODEs for the flight
def flight_model(t:float, Q:np.array, sim_config:dict):
    """
    Defines the system of ODEs for free fall with quadratic drag.

    Parameters:
    t (float): Current time.
    Q (numpy array): State vector [Q, dQdt].
    u (float): voltage on air brakes [0, 1]

    Returns:
    Derivatives of the variables stated in Q.
    """
    # declare global parameters
    # global GRAVITY, THRUST, ROCKET_MASS, DRAG_COEFFICIENCT_REDUCED, IDLE_TIME, BOOST_TIME
    DRAG_COEFFICIENCT_REDUCED = sim_config["DRAG_COEFFICIENCT_REDUCED"]
    IDLE_TIME = sim_config["IDLE_TIME"]
    BOOST_TIME = sim_config["BOOST_TIME"]
    THRUST = sim_config["THRUST"]
    GRAVITY = sim_config["GRAVITY"]
    ROCKET_MASS = sim_config["ROCKET_MASS"]


    # Extract variables
    x,y,z,vx,vy,vz,qw,qx,qy,qz,_,_,_ = Q # same structure as Kalman Filter
    q = (qw,qx,qy,qz)

    # Calculate drag force (proportional to v^2, opposing motion)
    v = np.sqrt(vx**2 + vy**2 + vz**2) # total speed
    FD_common = DRAG_COEFFICIENCT_REDUCED * v/2 # The common term in each direction

    drag_forcex = - FD_common * vx 
    drag_forcey = - FD_common * vy 
    drag_forcez = - FD_common * vz 

    # Trust
    # thust in body frame
    if IDLE_TIME < t < IDLE_TIME+BOOST_TIME:
        T  = array([THRUST, 0.0, 0.0])
    else:
        T = array([0.0]*3)
    # body -> world frame
    Thrustx, Thrusty, Thrustz = rotate_vector(T, q)
    
    # weight
    weight = GRAVITY * ROCKET_MASS
    # ensure rocket does not pass thorugh ground while idle
    if t<IDLE_TIME:
        weight = 0.0
    else:
        weight = GRAVITY * ROCKET_MASS

    # define some angular movement of the rocket
    if t < IDLE_TIME:
        omega = array([
             0.0*pi/180,
            -87.5/IDLE_TIME*pi/180,
             0.0*pi/180
        ])
    elif IDLE_TIME < t < IDLE_TIME+BOOST_TIME:
        omega = array([
             0.1*pi/180,
             0.5*pi/180,
             0.0*pi/180
        ])
    else:
        omega = array([
             0.5*pi/180 * sin(2*t),
             1.0*pi/180,
            -3.0*pi/180
        ])
    dqdt = quaternion_derivative(q, omega)
    # q = q + dqdt * dt      <- solved by runge kutta
    q = q / np.linalg.norm(q)  # Normalize

    dxdt = vx
    dydt = vy
    dzdt = vz
    dvxdt = drag_forcex / ROCKET_MASS + Thrustx / ROCKET_MASS
    dvydt = drag_forcey / ROCKET_MASS + Thrusty / ROCKET_MASS
    dvzdt = drag_forcez / ROCKET_MASS + Thrustz / ROCKET_MASS - weight / ROCKET_MASS
    dqwdt = dqdt[0]
    dqxdt = dqdt[1]
    dqydt = dqdt[2]
    dqzdt = dqdt[3]
    
    return np.array([dxdt, dydt, dzdt, dvxdt, dvydt, dvzdt, dqwdt, dqxdt, dqydt, dqzdt, *omega])

# run the simulation
def sim(sim_config):
    # simulation parameters
    TIME_STEP= sim_config["TIME_STEP"] #,
    shockwave_amplitude= sim_config["shockwave_amplitude"] #,
    GNSS_VARIANCE= sim_config["GNSS_VARIANCE"] #,
    BAROMETER_VARIANCE= sim_config["BAROMETER_VARIANCE"] #,
    ACCELEROMETER_VARIANCE= sim_config["ACCELEROMETER_VARIANCE"] #,
    GYROSCOPE_VARIANCE= sim_config["GYROSCOPE_VARIANCE"] #,
    MAGNETOMETER_VARIANCE= sim_config["MAGNETOMETER_VARIANCE"] #,
    ACCELEROMETER_BIAS= sim_config["ACCELEROMETER_BIAS"] #,
    GYROSCOPE_BIAS= sim_config["GYROSCOPE_BIAS"] #
    GRAVITY= sim_config["GRAVITY"] #     = 9.82 # m/s2
    ROCKET_INITIAL_QUAT= sim_config["ROCKET_INITIAL_QUAT"] # = array([1.0, 0.0, 0.0, 0.0])
    MAX_ITERATIONS= sim_config["MAX_ITERATIONS"] # = 100000
    SPEED_OF_SOUND= sim_config["SPEED_OF_SOUND"] # = 340 # m/s
    shockwave= sim_config["shockwave"] # = spiky_sinusoidal
    # SHOCKWAVE_AMPLITUDE = 500.0


    # run sim
    # initialize sensors
    gns = GNSS(
        variance=GNSS_VARIANCE,
        reference_altitude=0,
        reference_latitude=69.6489,
        reference_longitude=18.95508
    )
    bar = Barometer(
        variance=BAROMETER_VARIANCE,
        reference_pressure = 101325.0, # Pa
        reference_temperature = 288.15 # K
    )
    acc = Accelerometer(
        variance=ACCELEROMETER_VARIANCE, # m/s^2
        bias=ACCELEROMETER_BIAS
        # bias=array([0.0]*3)**2
    )
    gyr = Gyroscope(
        variance=GYROSCOPE_VARIANCE,
        bias=GYROSCOPE_BIAS
        # bias=array([0.0]*3)**2
    )
    magnetic_dip = 69*pi/180
    mag = Magnetometer(
        variance=MAGNETOMETER_VARIANCE * 1e-2,
        # reference_magnetic_vector=array([2.0, 18.0, -47.0]) # µT
        reference_magnetic_vector=array([2.0, 18.0, -47.0]) * 1e-2 # G 

    )

    # initial conditions
    tn = 0.0
    Qn = array([
        0.0,    # position
        0.0,
        0.0,
        0.0,    # velocity
        0.0,
        0.0,
        ROCKET_INITIAL_QUAT[0],    # orientation in euler angles
        ROCKET_INITIAL_QUAT[1],
        ROCKET_INITIAL_QUAT[2],
        ROCKET_INITIAL_QUAT[3],
        0.0,                      # omega placeholder  
        0.0,
        0.0
    ])
    # containers
    time                    = []
    position                = []
    velocity                = []
    attitude                = []
    gns_measurements        = []
    bar_measurements        = []
    acc_measurements        = []
    gyr_measurements        = []
    mag_measurements        = []
    i = 0
    while True:
        if i > MAX_ITERATIONS:
            break  
        # run until rocket hits the ground
        if Qn[2] < 0 and tn > 10.0:
            break
        tn, Qn = rungekutta_step(flight_model, tn, Qn, sim_config)
        dxdt, dydt, dzdt, dvxdt, dvydt, dvzdt, dqwdt, dqxdt, dqydt, dqzdt, *omega = flight_model(tn, Qn, sim_config)

        # all vectors are rotated the opposite way as the orientation quaterion
        # attitude_quaternion = euler_angles_to_quaternion(roll=Qn[-3], pitch=Qn[-2], yaw=Qn[-1])
        attitude_quaternion = Qn[6:10]
        acceleration_body = rotate_vector(array([dvxdt, dvydt, dvzdt]) + array([0.0, 0.0, GRAVITY]), inverse_quaternion(attitude_quaternion))
        angular_rate_body = rotate_vector(omega, inverse_quaternion(attitude_quaternion))
        # acceleration_body = rotate_vector(array([dvxdt, dvydt, dvzdt]) + array([0.0, 0.0, GRAVITY]), attitude_quaternion)
        # angular_rate_body = rotate_vector(omega, attitude_quaternion)

        # construct measurements
        gns_meas = gns.generate_measurement(Qn[:3])
        bar_meas = bar.generate_measurement(Qn[2])
        acc_meas = acc.generate_measurement(acceleration_body)
        gyr_meas = gyr.generate_measurement(angular_rate_body)
        mag_meas = mag.generate_measurement(attitude_quaternion)

        # TODO add more shockwave effects
        speed = np.linalg.norm(([dxdt, dydt, dzdt]))
        if 0.8*SPEED_OF_SOUND < speed: # < 1.*SPEED_OF_SOUND:
            bar_meas += shockwave(tn, amplitude=shockwave_amplitude, frequency=1.0, phase_shift=0)


        # the pressure-altitude conversion is done here, mostly for convenience. It is not yet implemented in the estimator
        # bar_meas = bar.pressure_to_altitude(bar_meas)

        # append in containers
        time.append(tn)
        position.append(Qn[0:3])
        velocity.append(Qn[3:6])
        attitude.append(Qn[6:10])
        gns_measurements.append(gns_meas)
        bar_measurements.append(bar_meas)
        acc_measurements.append(acc_meas)
        gyr_measurements.append(gyr_meas)
        mag_measurements.append(mag_meas)
   
        i += 1
        print(f"{i = :<5}, {Qn[2] = :>5.6}      ", end="\r")
    gns_measurements = array(gns_measurements)
    bar_measurements = array(bar_measurements)
    acc_measurements = array(acc_measurements)
    gyr_measurements = array(gyr_measurements)
    mag_measurements = array(mag_measurements)
    time = array(time)
    position = array(position)
    velocity = array(velocity)
    attitude = array(attitude)

    return {
        "time":time,
        "position":position,
        "velocity":velocity,
        "attitude":attitude,
        "gns_measurements":gns_measurements,
        "bar_measurements":bar_measurements,
        "acc_measurements":acc_measurements,
        "gyr_measurements":gyr_measurements,
        "mag_measurements":mag_measurements,
    }

def plot_measurements(
        time,
        gns_measurements,
        bar_measurements,
        acc_measurements,
        gyr_measurements,
        mag_measurements,
        sim_config,
    ):
    GRAVITY = sim_config["GRAVITY"]
    fig, axs = subplots(5, figsize=(10, 18))

    axs[0].set_title("GNSS")
    axs[3].set_ylabel("Magnitude")
    axs[0].plot(time, gns_measurements[:, 0], label="lat")
    axs[0].plot(time, gns_measurements[:, 1], label="lon")
    ax_ = axs[0].twinx()
    ax_.plot(time, gns_measurements[:, 2], label="alt", color="forestgreen")
    ax_.set_ylabel("Altitude [m]", color="forestgreen")
    axs[1].set_title("Barometer")
    # axs[1].set_ylabel("Pressure[Pa]")
    axs[1].set_ylabel("Altitude [m]")
    axs[1].plot(time, bar_measurements)
    axs[2].set_title("Accelerometer")
    axs[2].set_ylabel("Magnitude [g]")
    axs[2].plot(time, acc_measurements[:, 0]/GRAVITY, label="x")
    axs[2].plot(time, acc_measurements[:, 1]/GRAVITY, label="y")
    axs[2].plot(time, acc_measurements[:, 2]/GRAVITY, label="z")
    axs[3].set_title("Gyroscope")
    axs[3].set_ylabel("Magnitude [dps]")
    axs[3].plot(time, gyr_measurements[:, 0]*180/pi, label="x")
    axs[3].plot(time, gyr_measurements[:, 1]*180/pi, label="y")
    axs[3].plot(time, gyr_measurements[:, 2]*180/pi, label="z")
    axs[4].set_title("Magnetometer")
    axs[4].set_ylabel("Magnitude [µT]")
    axs[4].plot(time, mag_measurements[:, 0], label="x")
    axs[4].plot(time, mag_measurements[:, 1], label="y")
    axs[4].plot(time, mag_measurements[:, 2], label="z")

    for ax in axs.flatten(): ax.legend()
    fig.suptitle("Lastest measurements")
    fig.tight_layout()
    fig.savefig(figure_folder + "/" + "measurements.png")
    print(f"Saving measurement plot")
    # fig.savefig(r"C:\Users\NoahH\OneDrive - UiT Office 365\Personal OneDrive copy\Tromsø\UiT Arctic University of Norway\Ignite\NEXT\Simulations\figures\measurements.png")
    close()

