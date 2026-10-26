


/*
sensor handing functions


a bit misleading name, since this module only contain the sensor measurement handling, not the actual sensor drivers
*/
#ifndef MEASUREMENT_HANDLING_H
#define MEASUREMENT_HANDLING_H

#include "linalg.h"
#include "arm_math.h"


#define STANDARD_ATMOSPHERIC_PRESSURE 101325.0f
#define STANDARD_ATMOSPHERIC_TEMPERATURE 288.15f



// GNSS
int latlonalt_to_xyz_m(float32_t *pGNSS_latlonalt, 
    float32_t lat_ref,
    float32_t lon_ref,
    float32_t alt_ref,
    float32_t *pResult);

// Barometer
float pressure_to_alt(float32_t pressure, float32_t pressure_ref, float32_t temp_ref);

// Accelerometer

// Gyroscope

// Magnetometer


#endif