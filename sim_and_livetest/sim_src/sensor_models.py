


import numpy as np

from .quaternion_math import rotation_matrix_from_quaternion

# noise adding
rng = np.random.default_rng(42)  # reproducible
def add_gaussian_noise(true_value, sigma):
    true_value = np.asarray(true_value, dtype=float)
    noise = rng.normal(loc=0.0, scale=sigma, size=true_value.shape)
    return true_value + noise

class GNSS:
    def __init__(self, variance, reference_longitude, reference_latitude, reference_altitude):
        
        self.variance = variance

        # WGS84
        self.A = 6378137.0                  # semi-major axis [m]
        self.F = 1 / 298.257223563
        self.E2 = self.F * (2 - self.F)               # first eccentricity squared
        self.reference_longitude = reference_longitude
        self.reference_latitude = reference_latitude
        self.reference_altitude = reference_altitude

    
    def geodetic_to_ecef(self, lat, lon, h):
        # lat/lon in radians
        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        sin_lon = np.sin(lon)
        cos_lon = np.cos(lon)

        N = self.A / np.sqrt(1 - self.E2 * sin_lat**2)
        x = (N + h) * cos_lat * cos_lon
        y = (N + h) * cos_lat * sin_lon
        z = (N * (1 - self.E2) + h) * sin_lat
        return np.array([x, y, z])
    
    
    def ecef_to_geodetic(self, x, y, z, max_iter=10):
        lon = np.arctan2(y, x)
        p = np.sqrt(x*x + y*y)
        lat = np.arctan2(z, p * (1 - self.E2))  # initial guess

        for _ in range(max_iter):
            sin_lat = np.sin(lat)
            N =self.A/ np.sqrt(1 - self.E2 * sin_lat**2)
            h = p / np.cos(lat) - N
            lat_new = np.arctan2(z, p * (1 - self.E2 * N / (N + h)))
            if np.abs(lat_new - lat) < 1e-12:
                lat = lat_new
                break
            lat = lat_new

        sin_lat = np.sin(lat)
        N =self.A/ np.sqrt(1 - self.E2 * sin_lat**2)
        h = p / np.cos(lat) - N
        return lat, lon, h

    def enu_to_latlonh(self, e, n, u):
        lat0 = np.deg2rad(self.reference_latitude)
        lon0 = np.deg2rad(self.reference_longitude)

        # Reference ECEF
        r0 = self.geodetic_to_ecef(lat0, lon0, self.reference_altitude)

        # ENU -> ECEF rotation
        slat, clat = np.sin(lat0), np.cos(lat0)
        slon, clon = np.sin(lon0), np.cos(lon0)
        R_enu_to_ecef = np.array([
            [-slon,          -slat*clon,   clat*clon],
            [ clon,          -slat*slon,   clat*slon],
            [ 0.0,                 clat,        slat ]
        ])

        d_ecef = R_enu_to_ecef @ np.array([e, n, u])
        x, y, z = r0 + d_ecef

        lat, lon, h = self.ecef_to_geodetic(x, y, z)
        return np.rad2deg(lat), np.rad2deg(lon), h

    def generate_measurement(self, xyz):
        # add the noise to the gnss measurement before converting
        xyz_noisy = add_gaussian_noise(xyz, self.variance)
        x,y,z = xyz_noisy
        measurement_noisy = self.enu_to_latlonh(x, y, z)
        return measurement_noisy
    
class Barometer:
    def __init__(self, variance, reference_pressure, reference_temperature):
        self.variance = variance
        # ISA constants (troposphere, valid approx. 0-11 km)
        self.P0 = reference_pressure      # Pa
        self.T0 = reference_temperature        # K
        self.G = 9.80665        # m/s^2
        self.R = 287.05         # J/(kg*K)
        self.L = 0.0065         # K/m

    def altitude_to_pressure(self, altitude_m: float) -> float:
        """
        Convert altitude [m] to pressure [Pa] using ISA troposphere model.
        Valid approximately for 0 <= altitude <= 11000 m.
        """
        return self.P0 * (1.0 - (self.L * altitude_m) / self.T0) ** (self.G / (self.R * self.L))

    def pressure_to_altitude(self, pressure_pa: float) -> float:
        """
        Convert pressure [Pa] to altitude [m] using ISA troposphere inverse model.
        Valid approximately for pressures corresponding to 0-11 km altitude.
        """

        result =  (self.T0 / self.L) * (1.0 - (pressure_pa / self.P0) ** (self.R * self.L / self.G))
        if np.isnan(result):
            return -1000
        return result
    def generate_measurement(self, altitude):
        # add noise before converting
        altitude_noisy = add_gaussian_noise(altitude, self.variance)
        measurement_noisy = self.altitude_to_pressure(altitude_m=altitude_noisy)
        # measurement_noisy = add_gaussian_noise(measurement_clean, self.variance)
        return measurement_noisy
    
class Accelerometer:

    def __init__(self, bias, variance):
        self.bias = bias
        self.variance = variance
        
    def generate_measurement(self, acceleration):
        measurement_clean = acceleration
        measurement_noisy = add_gaussian_noise(measurement_clean+self.bias, self.variance)
        return measurement_noisy

class Gyroscope:
    def __init__(self, bias, variance):
        self.bias = bias
        self.variance = variance

    def generate_measurement(self, angular_rotation):
        # roll_rate, pitch_rate, yaw_rate = angular_rotation
        measurement_clean = angular_rotation
        measurement_noisy = add_gaussian_noise(measurement_clean+self.bias, self.variance)
        return measurement_noisy
    
class Magnetometer:
    def __init__(self, variance, reference_magnetic_vector):
        
        self.variance = variance
        self.reference_magnetic_vector = reference_magnetic_vector

    def magnetic_vector_from_quaternion(
        self,
        q_bn,                    # quaternion nav->body (w,x,y,z)
        # m_nav,                   # magnetic field in nav/world frame [uT]
        bias=np.zeros(3),        # hard-iron bias [uT]
        noise_std=0.0,           # scalar or 3-vector std [uT]
        rng=np.random.default_rng()
    ):
        R_bn = rotation_matrix_from_quaternion(np.asarray(q_bn, dtype=float))
        # m_body_ideal = R_bn @ np.asarray(self.reference_magnetic_vector, dtype=float)
        # wrong:
        # m_body_ideal = R_bn @ m_nav

        # correct:
        m_body_ideal = R_bn.T @ np.asarray(self.reference_magnetic_vector, dtype=float)
        return m_body_ideal
        
    def generate_measurement(self, oritentation_quaternion):
        measurement_clean = self.magnetic_vector_from_quaternion(oritentation_quaternion)
        measurement_noisy = add_gaussian_noise(measurement_clean, self.variance)
        # print(f"{measurement_clean = }")
        return measurement_noisy
    
        # noise = rng.normal(0.0, noise_std, size=3)
        # return m_body_ideal + np.asarray(bias, dtype=float) + noise
        
