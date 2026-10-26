

/*
    Error-state Kalman filter
*/


#ifndef ESKF_H
#define ESKF_H

// include math here, so it propagates into submodules
#include "arm_math.h"
#include "linalg.h"
#include "quaternions.h"

#define ACC_REJECTION_THRESHOLD 0.1f // m/s^2, threshold for rejecting accelerometer measurements based on magnitude
#define GRAVITY 9.825f // m/s^2

typedef struct{
        // initialize instances
        arm_matrix_instance_f32 pos;
        arm_matrix_instance_f32 vel;
        arm_matrix_instance_f32 att;
        arm_matrix_instance_f32 pos_error;
        arm_matrix_instance_f32 vel_error;
        arm_matrix_instance_f32 att_error;
        arm_matrix_instance_f32 nominal_state;
        arm_matrix_instance_f32 error_state;
        arm_matrix_instance_f32 covariance;
        arm_matrix_instance_f32 gravity_vector_unnormalized;
        arm_matrix_instance_f32 gravity_vector_normalized;
        arm_matrix_instance_f32 expected_gravity_unnormalized;
        arm_matrix_instance_f32 expected_gravity_normalized;
        arm_matrix_instance_f32 magnetic_vector_unnormalized;
        arm_matrix_instance_f32 magnetic_vector_normalized;
        arm_matrix_instance_f32 expected_magnetic_field_normalized;
        arm_matrix_instance_f32 acc_bias;
        arm_matrix_instance_f32 gyr_bias;
        arm_matrix_instance_f32 gns_nois;
        arm_matrix_instance_f32 bar_nois;
        arm_matrix_instance_f32 acc_nois;
        arm_matrix_instance_f32 gyr_nois;
        arm_matrix_instance_f32 mag_nois;
        arm_matrix_instance_f32 acc_bias_nois;
        arm_matrix_instance_f32 gyr_bias_nois;
        arm_matrix_instance_f32 gns_meas;
        arm_matrix_instance_f32 bar_meas;
        arm_matrix_instance_f32 acc_meas;
        arm_matrix_instance_f32 acc_meas_normalized;
        arm_matrix_instance_f32 mag_meas;
        arm_matrix_instance_f32 mag_meas_normalized;
        arm_matrix_instance_f32 gyr_meas;
        arm_matrix_instance_f32 acc_nois_normalized;
        arm_matrix_instance_f32 mag_nois_normalized;
        arm_matrix_instance_f32 R_att;
        arm_matrix_instance_f32 rotation_vector;
        arm_matrix_instance_f32 true_acc;
        arm_matrix_instance_f32 q_gyr;
        arm_matrix_instance_f32 R_gyr;
        arm_matrix_instance_f32 R_gyr_T;
        arm_matrix_instance_f32 rotated_acceleration_skew_matrix;
        arm_matrix_instance_f32 Vi;
        arm_matrix_instance_f32 Thetai;
        arm_matrix_instance_f32 Fx;
        arm_matrix_instance_f32 Fi;
        arm_matrix_instance_f32 Qi;
        arm_matrix_instance_f32 I3;
        arm_matrix_instance_f32 O3;
        arm_matrix_instance_f32 OOI;
        arm_matrix_instance_f32 O13;
        arm_matrix_instance_f32 O14;
        arm_matrix_instance_f32 O43;
        arm_matrix_instance_f32 I9;
        arm_matrix_instance_f32 dadq;
        arm_matrix_instance_f32 dmdq;
        arm_matrix_instance_f32 V_gns;
        arm_matrix_instance_f32 V_bar;
        arm_matrix_instance_f32 V_acc;
        arm_matrix_instance_f32 V_mag;
        arm_matrix_instance_f32 measurement_vector;
        arm_matrix_instance_f32 h;        
        arm_matrix_instance_f32 Hx;        
        arm_matrix_instance_f32 V;          
        arm_matrix_instance_f32 Q_angular_error;
        arm_matrix_instance_f32 X_angular_error;
        arm_matrix_instance_f32 H;
        arm_matrix_instance_f32 H_T;
        arm_matrix_instance_f32 S;
        arm_matrix_instance_f32 Kalman_gain;
        arm_matrix_instance_f32 innovation;
        arm_matrix_instance_f32 total_error;
        arm_matrix_instance_f32 q_error;
        arm_matrix_instance_f32 att_skew_matrix;
        arm_matrix_instance_f32 G;
        arm_matrix_instance_f32 intmed1;
        arm_matrix_instance_f32 intmed2;
        arm_matrix_instance_f32 intmed3;
        arm_matrix_instance_f32 intmed4;

        // initialize data buffers
        float32_t pos_buffer[3*1];
        float32_t vel_buffer[3*1];
        float32_t att_buffer[4*1];
        float32_t pos_error_buffer[3*1];
        float32_t vel_error_buffer[3*1];
        float32_t att_error_buffer[3*1];
        float32_t nominal_state_buffer[10*1];
        float32_t error_state_buffer[10*1];
        float32_t covariance_buffer[MAXROWS*MAXCOLS];
        float32_t gravity_vector_unnormalized_buffer[3*1];
        float32_t gravity_vector_normalized_buffer[3*1];
        float32_t expected_gravity_unnormalized_buffer[3*1];
        float32_t expected_gravity_normalized_buffer[3*1];
        float32_t magnetic_vector_unnormalized_buffer[3*1];
        float32_t magnetic_vector_normalized_buffer[3*1];
        float32_t expected_magnetic_field_normalized_buffer[3*1];
        float32_t acc_bias_buffer[3*1];
        float32_t gyr_bias_buffer[3*1];
        float32_t gns_nois_buffer[3*1];
        float32_t bar_nois_buffer[3*1];
        float32_t acc_nois_buffer[3*1];
        float32_t gyr_nois_buffer[3*1];
        float32_t mag_nois_buffer[3*1];
        float32_t acc_bias_nois_buffer[3*1];
        float32_t gyr_bias_nois_buffer[3*1];
        float32_t gns_meas_buffer[3*1];
        float32_t bar_meas_buffer[3*1];
        float32_t acc_meas_buffer[3*1];
        float32_t acc_meas_normalized_buffer[3*1];
        float32_t mag_meas_buffer[3*1];
        float32_t mag_meas_normalized_buffer[3*1];
        float32_t gyr_meas_buffer[3*1];
        float32_t acc_nois_normalized_buffer[3*1];
        float32_t mag_nois_normalized_buffer[3*1];
        float32_t R_att_buffer[3*3];
        float32_t rotation_vector_buffer[3*1];
        float32_t true_acc_buffer[3*1];
        float32_t q_gyr_buffer[4*1];
        float32_t R_gyr_buffer[3*3];
        float32_t R_gyr_T_buffer[3*3];
        float32_t rotated_acceleration_skew_matrix_buffer[MAXROWS*MAXCOLS];
        float32_t Vi_buffer[3*3];
        float32_t Thetai_buffer[3*3];
        float32_t Fx_buffer[MAXROWS*MAXCOLS];
        float32_t Fi_buffer[MAXROWS*MAXCOLS];
        float32_t Qi_buffer[MAXROWS*MAXCOLS];
        float32_t I3_buffer[3*3];
        float32_t O3_buffer[3*3];
        float32_t OOI_buffer[1*3];
        float32_t O13_buffer[1*3];
        float32_t O14_buffer[1*4];
        float32_t O43_buffer[4*3];
        float32_t I9_buffer[MAXROWS*MAXCOLS];
        float32_t dadq_buffer[MAXROWS*MAXCOLS];
        float32_t dmdq_buffer[MAXROWS*MAXCOLS];
        float32_t V_gns_buffer[3*3];
        float32_t V_bar_buffer[3*3];
        float32_t V_acc_buffer[3*3];
        float32_t V_mag_buffer[3*3];
        float32_t measurement_vector_buffer[MAXROWS*1];
        float32_t h_buffer[MAXROWS*1];
        float32_t Hx_buffer[MAXROWS*MAXCOLS];
        float32_t V_buffer[MAXROWS*MAXCOLS];
        float32_t Q_angular_error_buffer[MAXROWS*MAXCOLS];
        float32_t X_angular_error_buffer[MAXROWS*MAXCOLS];
        float32_t H_buffer[MAXROWS*MAXCOLS];
        float32_t H_T_buffer[MAXROWS*MAXCOLS];
        float32_t S_buffer[MAXROWS*MAXCOLS];
        float32_t Kalman_gain_buffer[MAXROWS*MAXCOLS];
        float32_t innovation_buffer[MAXROWS*1];
        float32_t total_error_buffer[MAXROWS*MAXCOLS];
        float32_t q_error_buffer[MAXROWS*MAXCOLS];
        float32_t att_skew_matrix_buffer[MAXROWS*MAXCOLS];
        float32_t G_buffer[MAXROWS*MAXCOLS];
        float32_t intmed1_buffer[MAXROWS*MAXCOLS];
        float32_t intmed2_buffer[MAXROWS*MAXCOLS];
        float32_t intmed3_buffer[MAXROWS*MAXCOLS];
        float32_t intmed4_buffer[MAXROWS*MAXCOLS];

        // char gravity_reference_frame[3];

        float32_t acc_noise_update_prescaler;

} ErrorStateKalmanFilter;

// helper states <-> state vectors
void nominal_state2helpers(ErrorStateKalmanFilter *self);
void helpers2nominal_state(ErrorStateKalmanFilter *self);
void error_state2helpers(ErrorStateKalmanFilter *self);
void helpers2error_state(ErrorStateKalmanFilter *self);

int ESKF_init(
    ErrorStateKalmanFilter *self,
    float32_t *pGNSS_noise, 
    float32_t *paccelerometer_noise,
    float32_t *pgyroscope_noise,
    float32_t *pmagnetometer_noise,
    float32_t *pbarometer_noise);

int predict(ErrorStateKalmanFilter *self,
    float32_t *pacc_meas, float32_t *pgyr_meas, 
    float32_t delta_t_acc, float32_t delta_t_gyr,
    uint8_t acc_new, uint8_t gyr_new);


int update(ErrorStateKalmanFilter *self,
    float32_t *pgns_meas, float32_t *pbar_meas, 
    float32_t *pacc_meas, float32_t *pmag_meas,
    uint8_t gns_new, uint8_t bar_new, 
    uint8_t acc_new, uint8_t mag_new);

#endif