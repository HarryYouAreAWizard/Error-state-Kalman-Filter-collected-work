

import urllib.request
import json
import numpy as np


def read_one_datapoint(host, timeout):
    url = f"{host}/get?accX&accY&accZ&acc_time&gyrX&gyrY&gyrZ&gyr_time&magX&magY&magZ&mag_time&locLat&locLon&locZ&loc_time"
    # url = f"{host}/get?accX&accY&accZ&acc_time&gyroX&gyroY&gyroZ&gyro_time&magX&magY&magZ&mag_time&gpsLat&gpsLon&gpsZ&gps_time"

    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            data = json.loads(r.read())
            buffer = data["buffer"]

    except Exception as e:
        # print("Error:", e)
        # print(f"STOPPED HERE")
        return None
    
    return {"accelerometer_data":   np.array([buffer["accX"]["buffer"][0], buffer["accY"]["buffer"][0], buffer["accZ"]["buffer"][0]]),
            "gyroscope_data":       np.array([buffer["gyrX"]["buffer"][0], buffer["gyrY"]["buffer"][0], buffer["gyrZ"]["buffer"][0]]),
            # "gyroscope_data":       np.array([buffer["gyroX"]["buffer"][0], buffer["gyroY"]["buffer"][0], buffer["gyroZ"]["buffer"][0]]),
            "magnetometer_data":    np.array([buffer["magX"]["buffer"][0], buffer["magY"]["buffer"][0], buffer["magZ"]["buffer"][0]]),
            "gnss_data":            np.array([buffer["locLat"]["buffer"][0], buffer["locLon"]["buffer"][0], buffer["locZ"]["buffer"][0]]),
            # "gnss_data":            np.array([buffer["gpsLat"]["buffer"][0], buffer["gpsLon"]["buffer"][0], buffer["gpsZ"]["buffer"][0]]),
            "accelerometer_time":   buffer["acc_time"]["buffer"][0],
            "gyroscope_time":       buffer["gyr_time"]["buffer"][0],
            "magnetometer_time":    buffer["mag_time"]["buffer"][0],
            "gnss_time":            buffer["loc_time"]["buffer"][0]
            }



def get_host():
    # host_path = r"C:\Users\NoahH\OneDrive\Tromsø\UiT Arctic University of Norway\Ignite\NEXT\error_state_kalman_filter\phone_data_acquisition\host.txt"
    # host_path = r"C:\Users\NoahH\OneDrive - UiT Office 365\Personal OneDrive copy\Tromsø\UiT Arctic University of Norway\Ignite\NEXT\error_state_kalman_filter\phone_data_acquisition\host.txt"
    host_path = r"phone_data_acquisition\host.txt"
    doc = open(host_path, "r")
    host = doc.read().strip()
    doc.close()
    return host

# # Example usage:
# host = "http://192.168.43.207:8080"

# output = read_one_datapoint(host)

# print(output)






