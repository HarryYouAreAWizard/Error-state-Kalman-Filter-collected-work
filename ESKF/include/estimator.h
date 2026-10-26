
/*
The estimator module handles 
 - sensor conversion and references
    - GNSS and barometer calculations
 - sensor checks
    - validity checks

This module might also handle the determination of whether or not a sensor measurement is new
*/


#ifndef ESTIMATOR_H
#define ESTIMATOR_H


#include "ESKF.h"


typedef struct {
    ErrorStateKalmanFilter eskf;
    uint8_t gns_new;
    uint8_t bar_new;
    uint8_t acc_new;
    uint8_t gyr_new;
    uint8_t mag_new;

    // gns measurement references
    float32_t lat_ref;
    float32_t lon_ref;
    float32_t alt_ref;

    // barometer measurement references
    float32_t pressure_ref;
    float32_t temperature_ref;
    
    // gnss converted measurement container
    float32_t gns_meas_local_m[3];
    float32_t bar_meas_alt_m[1];

    // backup, in case ESKF must be reset due to bad measurement
    arm_matrix_instance_f32 nominal_state_backup;
    arm_matrix_instance_f32 covariance_matric_backup;
    float32_t nominal_state_backup_buffer[10*1];
    float32_t covariance_matric_backup_buffer[9*9];
    

} Estimator;


int estimator_init(
    Estimator *self,
    float32_t *pGNSS_noise,
    float32_t *paccelerometer_noise,
    float32_t *pgyroscope_noise,
    float32_t *pmagnetometer_noise,
    float32_t *pbarometer_noise);

void estimator_set_reference(
    Estimator *self,
    float32_t reference_height,
    float32_t reference_latitude,
    float32_t reference_longitude);

// ------------------measurement checks------------------
int check_gnss(float32_t *gns_meas, uint8_t gns_new);
int check_barometer(float32_t *bar_meas, uint8_t bar_new);
int check_accelerometer(float32_t *acc_meas, uint8_t acc_new);
int check_gyroscope(float32_t *gyr_meas, uint8_t gyr_new);
int check_magnetometer(float32_t *mag_meas, uint8_t mag_new);

int estimate(
    Estimator *self,
    float32_t *gns_meas,
    float32_t *bar_meas,
    float32_t *acc_meas,
    float32_t *gyr_meas,
    float32_t *mag_meas,
    float32_t delta_t_acc,
    float32_t delta_t_gyr,
    uint8_t gns_new,
    uint8_t bar_new,
    uint8_t acc_new,
    uint8_t gyr_new,
    uint8_t mag_new,
    uint8_t enfore_only_predict);

#endif 