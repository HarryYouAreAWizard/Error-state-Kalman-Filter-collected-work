

/*
    Source file for the Error-State Kalman Filter initialization
*/

#include "ESKF.h"

static void init_initialize_matrices_as_zero(ErrorStateKalmanFilter *self) {
    /*
    initializes all matrices with the proper size. All values are set to zero

    matrices set to size 1x1 change shape and dimensions at runtime
    */
    init_mat_zeros(&self->pos, 3, 1, self->pos_buffer);
    init_mat_zeros(&self->vel, 3, 1, self->vel_buffer);
    init_mat_zeros(&self->att, 4, 1, self->att_buffer);
    init_mat_zeros(&self->pos_error, 3, 1, self->pos_error_buffer);
    init_mat_zeros(&self->vel_error, 3, 1, self->vel_error_buffer);
    init_mat_zeros(&self->att_error, 3, 1, self->att_error_buffer);
    init_mat_zeros(&self->nominal_state, 10, 1, self->nominal_state_buffer);
    init_mat_zeros(&self->error_state, 9, 1, self->error_state_buffer);
    init_mat_zeros(&self->gravity_vector_unnormalized, 3, 1, self->gravity_vector_unnormalized_buffer);
    init_mat_zeros(&self->gravity_vector_normalized, 3, 1, self->gravity_vector_normalized_buffer);
    init_mat_zeros(&self->expected_gravity_normalized, 3, 1, self->expected_gravity_normalized_buffer);
    init_mat_zeros(&self->magnetic_vector_unnormalized, 3, 1, self->magnetic_vector_unnormalized_buffer);
    init_mat_zeros(&self->magnetic_vector_normalized, 3, 1, self->magnetic_vector_normalized_buffer);
    init_mat_zeros(&self->expected_magnetic_field_normalized, 3, 1, self->expected_magnetic_field_normalized_buffer);
    init_mat_zeros(&self->covariance, 9, 9, self->covariance_buffer);
    init_mat_zeros(&self->acc_bias, 3, 1, self->acc_bias_buffer);
    init_mat_zeros(&self->gyr_bias, 3, 1, self->gyr_bias_buffer);
    init_mat_zeros(&self->gns_nois, 3, 1, self->gns_nois_buffer);
    init_mat_zeros(&self->bar_nois, 1, 1, self->bar_nois_buffer);
    init_mat_zeros(&self->acc_nois, 3, 1, self->acc_nois_buffer);
    init_mat_zeros(&self->gyr_nois, 3, 1, self->gyr_nois_buffer);
    init_mat_zeros(&self->mag_nois, 3, 1, self->mag_nois_buffer);
    init_mat_zeros(&self->acc_bias_nois, 3, 1, self->acc_bias_nois_buffer);
    init_mat_zeros(&self->gyr_bias_nois, 3, 1, self->gyr_bias_nois_buffer);
    init_mat_zeros(&self->gns_meas, 3, 1, self->gns_meas_buffer);
    init_mat_zeros(&self->bar_meas, 1, 1, self->bar_meas_buffer);
    init_mat_zeros(&self->acc_meas, 3, 1, self->acc_meas_buffer);
    init_mat_zeros(&self->acc_meas_normalized, 3, 1, self->acc_meas_normalized_buffer);
    init_mat_zeros(&self->mag_meas, 3, 1, self->mag_meas_buffer);
    init_mat_zeros(&self->mag_meas_normalized, 3, 1, self->mag_meas_normalized_buffer);
    init_mat_zeros(&self->gyr_meas, 3, 1, self->gyr_meas_buffer);
    init_mat_zeros(&self->acc_nois_normalized, 3, 1, self->acc_nois_normalized_buffer);
    init_mat_zeros(&self->mag_nois_normalized, 3, 1, self->mag_nois_normalized_buffer);
    init_mat_zeros(&self->R_att, 3, 3, self->R_att_buffer);
    init_mat_zeros(&self->rotation_vector, 3, 1, self->rotation_vector_buffer);
    init_mat_zeros(&self->true_acc, 3, 1, self->true_acc_buffer);
    init_mat_zeros(&self->q_gyr, 4, 1, self->q_gyr_buffer);
    init_mat_zeros(&self->R_gyr, 3, 3, self->R_gyr_buffer);
    init_mat_zeros(&self->R_gyr_T, 3, 3, self->R_gyr_T_buffer);
    init_mat_zeros(&self->rotated_acceleration_skew_matrix, 3, 3, self->rotated_acceleration_skew_matrix_buffer);
    init_mat_zeros(&self->Vi, 3, 3, self->Vi_buffer);
    init_mat_zeros(&self->Thetai, 3, 3, self->Thetai_buffer);
    init_mat_zeros(&self->Fx, 9, 9, self->Fx_buffer);
    init_mat_zeros(&self->Fi, 9, 6, self->Fi_buffer); // size changed
    init_mat_zeros(&self->Qi, 6, 6, self->Qi_buffer); // size changed
    init_mat_zeros(&self->I3, 3, 3, self->I3_buffer);
    init_mat_zeros(&self->O3, 3, 3, self->O3_buffer);
    init_mat_zeros(&self->OOI, 1, 3, self->OOI_buffer);
    init_mat_zeros(&self->O13, 1, 3, self->O13_buffer);
    init_mat_zeros(&self->O14, 1, 4, self->O14_buffer);
    init_mat_zeros(&self->O43, 4, 3, self->O43_buffer);
    init_mat_zeros(&self->I9, 9, 9, self->I9_buffer);
    init_mat_zeros(&self->dadq, 3, 4, self->dadq_buffer);
    init_mat_zeros(&self->dmdq, 3, 4, self->dmdq_buffer);
    init_mat_zeros(&self->V_gns, 3, 3, self->V_gns_buffer);
    init_mat_zeros(&self->V_bar, 1, 1, self->V_bar_buffer);
    init_mat_zeros(&self->V_acc, 3, 3, self->V_acc_buffer);
    init_mat_zeros(&self->V_mag, 3, 3, self->V_mag_buffer);
    init_mat_zeros(&self->measurement_vector, 1, 1, self->measurement_vector_buffer);
    init_mat_zeros(&self->h , 1, 1, self->h_buffer);
    init_mat_zeros(&self->Hx, 1, 1, self->Hx_buffer);
    init_mat_zeros(&self->V, 1, 1, self->V_buffer);
    init_mat_zeros(&self->Q_angular_error, 4, 3, self->Q_angular_error_buffer);
    init_mat_zeros(&self->X_angular_error, 10, 9, self->X_angular_error_buffer);
    init_mat_zeros(&self->H, 1, 1, self->H_buffer);
    init_mat_zeros(&self->H_T, 1, 1, self->H_T_buffer);
    init_mat_zeros(&self->S, 1, 1, self->S_buffer);
    init_mat_zeros(&self->Kalman_gain, 1, 1, self->Kalman_gain_buffer);
    init_mat_zeros(&self->innovation, 1, 1, self->innovation_buffer);
    init_mat_zeros(&self->total_error, 9, 1, self->total_error_buffer);
    init_mat_zeros(&self->q_error, 4, 1, self->q_error_buffer);
    init_mat_zeros(&self->att_skew_matrix, 3, 3, self->att_skew_matrix_buffer);
    init_mat_zeros(&self->G, 9, 9, self->G_buffer);
    init_mat_zeros(&self->intmed1, 1, 1, self->intmed1_buffer);
    init_mat_zeros(&self->intmed2, 1, 1, self->intmed2_buffer);
    init_mat_zeros(&self->intmed3, 1, 1, self->intmed3_buffer);
    init_mat_zeros(&self->intmed4, 1, 1, self->intmed4_buffer);
}
static int init_set_reference_vectors(ErrorStateKalmanFilter *self) {
    /*
    Set the gravity and magnetic reference vectors in the ENU reference frame

    magnetic vector only defined from magnetic dip. Inclination not taken into account
    */
    int error;
    // g = (0, 0, GRAVITY)
    self->gravity_vector_unnormalized.pData[2] = GRAVITY;
    // self->gravity_vector_unnormalized.pData[2] = -GRAVITY;
    float32_t norm_gravity = vecnorm(&self->gravity_vector_unnormalized);
    // check zero division
    if (norm_gravity < DIVISION_BY_ZERO_THRESHOLD) {return ERROR_DIVISION_BY_ZERO;}
    error = matscale(&self->gravity_vector_unnormalized, 1.0f / norm_gravity, &self->gravity_vector_normalized);
    if(error){return error;}
    
    // m = (0, cos(theta), -sin(theta))
    float32_t magnetic_dip = 69.0f * PI / 180.0f;
    self->magnetic_vector_unnormalized.pData[1] =  cosf(magnetic_dip);
    self->magnetic_vector_unnormalized.pData[2] = -sinf(magnetic_dip);
    float32_t norm_magnetic_vector = vecnorm(&self->magnetic_vector_unnormalized);
    if (norm_magnetic_vector < DIVISION_BY_ZERO_THRESHOLD) {return ERROR_DIVISION_BY_ZERO;}

    error = matscale(&self->magnetic_vector_unnormalized, 1.0f / norm_magnetic_vector, &self->magnetic_vector_normalized);
    if(error){return error;}
    return 0;

}

static void init_transfer_noise_values(
    ErrorStateKalmanFilter *self,
    float32_t *pGNSS_noise, 
    float32_t *paccelerometer_noise,
    float32_t *pgyroscope_noise,
    float32_t *pmagnetometer_noise,
    float32_t *pbarometer_noise) {
    /*
    transfer noise values to internal containers
    */
    for (int i=0; i<3; i++) {
        self->gns_nois.pData[i] = pGNSS_noise[i];
        self->acc_nois.pData[i] = paccelerometer_noise[i];
        self->gyr_nois.pData[i] = pgyroscope_noise[i];
        self->mag_nois.pData[i] = pmagnetometer_noise[i];
    }
    self->bar_nois.pData[0] = pbarometer_noise[0];
}
static void init_populate_matrices(ErrorStateKalmanFilter *self) {
    /*
    Populate the matrices used in the ESKF.
    Only entries that do not change at runtime are populated here.
    All entries that do not change at runtime are populated here.
    */

    // attitude quaternion must be unit quaternion
    self->nominal_state.pData[6] = 1.0f; 

    // set initial covariance to 10*identity
    for (int i=0; i<6; i++) {
        set_element(&self->covariance, i, i, 0.1f); // some choice of initial covariance.
    }
    for (int i=6; i<10; i++) {
        set_element(&self->covariance, i, i, 0.0001f); // some choice of initial covariance.
    }
    
    // predict step
    // Fx and Fi identity blocks
    for (int i=0;i<3;i++) {
        // Fx
        // position propagation
        set_element(&self->Fx, i, i, 1.0f);
        // velocity propagation
        set_element(&self->Fx, i+3, i+3, 1.0f);

        // Fi
        set_element(&self->Fi, i+3, i, 1.0f);
        set_element(&self->Fi, i+6, i+3, 1.0f);
    }
    
    // update step
    set_identity(&self->I3);
    set_element(&self->OOI, 0, 2, 1.0f);
    set_identity(&self->I9);
    set_identity(&self->G); 
    for (int i=0; i<3; i++) {
        set_element(&self->X_angular_error, i,   i, 1.0f);
        set_element(&self->X_angular_error, i+3, i+3, 1.0f);
        set_element(&self->G, i,   i, 1.0f);
        set_element(&self->G, i+3, i+3, 1.0f);

        // identity on position and velocity diagagonal entries
        set_element(&self->X_angular_error, i, i, 1.0f);
    }

    // no longer in use
    self->acc_noise_update_prescaler = 1.0;
}    

int ESKF_init(
    ErrorStateKalmanFilter *self,
    float32_t *pGNSS_noise, 
    float32_t *paccelerometer_noise, 
    float32_t *pgyroscope_noise, 
    float32_t *pmagnetometer_noise, 
    float32_t *pbarometer_noise) {
    /*
        Initialize and set up the ESKF internal matrices

        limitations
        - only ENU frame implemented
        - bias not implemented
    */ 
    int error;

            init_initialize_matrices_as_zero(self); 
            error = init_set_reference_vectors(self); if(error){return error;}
            init_transfer_noise_values(
                self,
                pGNSS_noise, 
                paccelerometer_noise, 
                pgyroscope_noise, 
                pmagnetometer_noise, 
                pbarometer_noise);
            init_populate_matrices(self);
    for (int i=0; i<3; i++) {
            self->gyr_nois.pData[i] = 0.01f;
            self->gns_nois.pData[i] = 1.0f;
            self->acc_nois.pData[i] = 0.1f;
            self->mag_nois.pData[i] = 100.0f;
        }
    self->bar_nois.pData[0] = 1.0f; 
    return 0;
}


