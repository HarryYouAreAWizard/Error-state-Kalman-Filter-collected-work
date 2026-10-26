

#include "estimator.h"
#include "measurement_handling.h"
#include "ESKF.h"
// #include <stdio.h>

int estimator_init(
    Estimator *self,
    float32_t *pGNSS_noise,
    float32_t *paccelerometer_noise,
    float32_t *pgyroscope_noise,
    float32_t *pmagnetometer_noise,
    float32_t *pbarometer_noise) {
    /*
    initialize the estimator
    */
    int error;

    // the quaternion calculations require helper matrices, which must be initialized
    initialize_quaternion_helpers();

    // initialize backups
    init_mat_zeros(&self->nominal_state_backup, 10, 1, self->nominal_state_backup_buffer);
    init_mat_zeros(&self->covariance_matric_backup, 9, 9, self->covariance_matric_backup_buffer);


    error = ESKF_init(
        &self->eskf, 
        pGNSS_noise, 
        paccelerometer_noise, 
        pgyroscope_noise, 
        pmagnetometer_noise,
        pbarometer_noise);
    if(error){return error;}
        
    return 0;       
}
        
void estimator_set_reference(
    Estimator *self,
    float32_t reference_height,
    float32_t reference_latitude,
    float32_t reference_longitude) {
        /*
        set the references for the GNSS and barometer
        */
        self->lat_ref = reference_latitude;
        self->lon_ref = reference_longitude;
        self->alt_ref = reference_height;

        // fixed constant for barometer
        self->pressure_ref = STANDARD_ATMOSPHERIC_PRESSURE; // standard atmospheric pressure at sea level in Pa
        self->temperature_ref = STANDARD_ATMOSPHERIC_TEMPERATURE; // standard temperature at sea level in K

}

// ------------------measurement checks------------------
int check_gnss(float32_t *gns_meas, uint8_t gns_new) {

    // if already invalid, return invalid
    if (!gns_new) {
        return 0;
    }

    // we should never move 100 km from initial position
    float upperboundxy =  100.0 * 1000.0; // m 
    float lowerboundxy = -100.0 * 1000.0; // m

    // we should never be below 1 km under sea level or above 100 km
    float upperboundz  =  100.0 * 1000.0; // m
    float lowerboundz  = -1.0   * 1000.0; // m

    if (gns_meas[0] < lowerboundxy || upperboundxy < gns_meas[0]) {
        return 0;
    }
    if (gns_meas[1] < lowerboundxy || upperboundxy < gns_meas[1]) {
        return 0;
    }
    if (gns_meas[2] < lowerboundz || upperboundz < gns_meas[2]) {
        return 0;
    }
    
    // check against 0.0 measurements

    if (gns_meas[0] == 0.0 || gns_meas[1] == 0.0 || gns_meas[2] == 0.0) {
        return 0;
    }

    return 1;
    }

int check_barometer(float32_t *bar_meas, uint8_t bar_new) {

    if (!bar_new) {
        return 0;
    }

    // the atmospheric pressure from 0 to 10 000 km falls from ~100 kPa to 20 kPa. 
    // https://en.wikipedia.org/wiki/Atmospheric_pressure#/media/File:Atmospheric_Pressure_vs._Altitude.png

    float upperbound = 1000.0 * 1000.0; // Pa
    float lowerbound = 1.0    * 1000.0; // Pa
    // check bounds for pressure (in Pa)
    if (bar_meas[0] < lowerbound || bar_meas[0] > upperbound) { 
        return 0;
    }

    return 1;
}

int check_accelerometer(float32_t *acc_meas, uint8_t acc_new) {

    if (!acc_new) {
        return 0;
    }

    float upperbound =  100.0 * GRAVITY; // m/s^2
    float lowerbound = -100.0 * GRAVITY; // m/s^2
    for (int i=0; i<3; i++) {
        if (acc_meas[i] < lowerbound || acc_meas[i] > upperbound) { 
            return 0;
        }
    }
    return 1;
}

int check_gyroscope(float32_t *gyr_meas, uint8_t gyr_new) {
    /*
    TODO check the upper and lower bound
    */
    if (!gyr_new) {
        return 0;
    }

    // maximum 10 revolutions per second
    float upperbound =  360.0 * 10 * PI/180.0; // rad/s
    float lowerbound = -360.0 * 10 * PI/180.0; // rad/s
    // check bounds for angular velocity (in rad/s)
    for (int i=0; i<3; i++) {
        if (gyr_meas[i] < lowerbound || gyr_meas[i] > upperbound) { 
            return 0;
        }
    }
    return 1;
}

int check_magnetometer(float32_t *mag_meas, uint8_t mag_new) {
    /*
    Depends heavily on the magnetometer used. 
    It is not clear what the MMC5983MA uses, although the datasheet hints at using Gauss
    */
    if (!mag_new) {
        return 0;
    }

    // Earth magnetic field seems to not exeed 1.0 Gauss at the surface 
    // https://en.wikipedia.org/wiki/Earth%27s_magnetic_field

    float upperbound =  1.5; // G
    float lowerbound = -1.5; // G

    // check bounds for magnetic field (in Tesla)
    for (int i=0; i<3; i++) {
        if (mag_meas[i] < lowerbound || mag_meas[i] > upperbound) {
            return 0;
        }
    }
    
    // check the norm. It should stay relatively constant. If not, then there are other disturbances
    float mag_norm_sq = mag_meas[0]*mag_meas[0] + mag_meas[1]*mag_meas[1] + mag_meas[2]*mag_meas[2];
    if (mag_norm_sq > upperbound*upperbound) { 
        return 0;
    }

    // in case of 0-measurement 
    if (mag_norm_sq < 0.1) {
        return 0;
    }

    return 1;
}


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
    uint8_t enforce_only_predict) {

        
        int error;
        // copy the content in nominal state and covariance in case the ESKF needs a reset
        copy_matrix(&self->eskf.nominal_state, &self->nominal_state_backup);
        copy_matrix(&self->eskf.covariance, &self->covariance_matric_backup);

        // check acc and int before the predict step
        acc_new = check_accelerometer(acc_meas, acc_new);
        gyr_new = check_gyroscope(gyr_meas, gyr_new);

        error = predict(&self->eskf, acc_meas, gyr_meas, delta_t_acc, delta_t_gyr, acc_new, gyr_new);        
        if(error){return error;}

        // stop early if only to run predict step
        if (enforce_only_predict) {
            return 0;
        }

        // continue with update, and check measurements beforehand
        gns_new = check_gnss(gns_meas, gns_new);
        bar_new = check_barometer(bar_meas, bar_new);
        // mag_new = check_magnetometer(mag_meas, mag_new);

        // convert GNSS
        if (gns_new) {
            latlonalt_to_xyz_m(gns_meas, self->lat_ref, self->lon_ref, self->alt_ref, self->gns_meas_local_m);
        }

        // convert barometer
        if (bar_new) {
            self->bar_meas_alt_m[0] = pressure_to_alt(bar_meas[0], self->pressure_ref, self->temperature_ref);
        }

        error = update(&self->eskf, self->gns_meas_local_m, self->bar_meas_alt_m, acc_meas, mag_meas, gns_new, bar_new, acc_new, mag_new);
        if(error){return error;}

        return 0;
    }




