

/*
    Source file for the Error-State Kalman Filter update step
*/
#include "ESKF.h"

// update step functions
static void update_setup_meas_GNSS(ErrorStateKalmanFilter *self, float32_t *pgns_meas) {
    /*
    transfer measurement to internal container
    
    transfer noise values into matrix
    */
    for (int i=0;i<3;i++) {
        set_element(&self->gns_meas, i, 0, pgns_meas[i]);
        set_element(&self->V_gns, i, i, self->gns_nois.pData[i] * self->gns_nois.pData[i]);
    }
}
static void update_setup_meas_barometer(ErrorStateKalmanFilter *self, float32_t *pbar_meas) {
    /*
    transfer measurement to internal container
    
    transfer noise values into matrix
    */
    for (int i=0;i<1;i++) {
        set_element(&self->bar_meas, i, 0, pbar_meas[i]);
        set_element(&self->V_bar, i, i, self->bar_nois.pData[i] * self->bar_nois.pData[i]);
    }
}
static void update_setup_meas_accelerometer_jacobian(ErrorStateKalmanFilter *self) {
    /*
    Jacobian construction
    gx, gy, gz = self.gravity_vector_normalized
    dadq = 2 * array([
        [gx*qw + gy*qz - gz*qy,  gx*qx + gy*qy + gz*qz, -gx*qy + gy*qx - gz*qw, -gx*qz + gy*qw + gz*qx],
        [-gx*qz + gy*qw + gz*qx, gx*qy - gy*qx + gz*qw,  gx*qx + gy*qy + gz*qz, -gx*qw - gy*qz + gz*qy],
        [gx*qy - gy*qx + gz*qw,  gx*qz - gy*qw - gz*qx,  gx*qw + gy*qz - gz*qy,  gx*qx + gy*qy + gz*qz]
    */
    float32_t qw = self->nominal_state.pData[6+0];
    float32_t qx = self->nominal_state.pData[6+1];
    float32_t qy = self->nominal_state.pData[6+2];
    float32_t qz = self->nominal_state.pData[6+3];

    float32_t gx = self->gravity_vector_normalized.pData[0];
    float32_t gy = self->gravity_vector_normalized.pData[1];
    float32_t gz = self->gravity_vector_normalized.pData[2];
    set_element(&self->dadq, 0, 0, 2.0f * ( gx*qw + gy*qz - gz*qy) );
    set_element(&self->dadq, 0, 1, 2.0f * ( gx*qx + gy*qy + gz*qz) );   
    set_element(&self->dadq, 0, 2, 2.0f * (-gx*qy + gy*qx - gz*qw) );
    set_element(&self->dadq, 0, 3, 2.0f * (-gx*qz + gy*qw + gz*qx) );
    set_element(&self->dadq, 1, 0, 2.0f * (-gx*qz + gy*qw + gz*qx) );
    set_element(&self->dadq, 1, 1, 2.0f * ( gx*qy - gy*qx + gz*qw) );
    set_element(&self->dadq, 1, 2, 2.0f * ( gx*qx + gy*qy + gz*qz) );
    set_element(&self->dadq, 1, 3, 2.0f * (-gx*qw - gy*qz + gz*qy) );
    set_element(&self->dadq, 2, 0, 2.0f * ( gx*qy - gy*qx + gz*qw) );
    set_element(&self->dadq, 2, 1, 2.0f * ( gx*qz - gy*qw - gz*qx) );
    set_element(&self->dadq, 2, 2, 2.0f * ( gx*qw + gy*qz - gz*qy) );
    set_element(&self->dadq, 2, 3, 2.0f * ( gx*qx + gy*qy + gz*qz) );
}
static void update_setup_meas_magnetometer_jacobian(ErrorStateKalmanFilter *self) {
    /*
    rx, ry, rz = self.magnetic_vector_normalized
    self.dmdq = 2 * array([
        [rx*qw + ry*qz - rz*qy,  rx*qx + ry*qy + rz*qz, -rx*qy + ry*qx - rz*qw, -rx*qz + ry*qw + rz*qx], 
        [-rx*qz + ry*qw + rz*qx, rx*qy - ry*qx + rz*qw,  rx*qx + ry*qy + rz*qz, -rx*qw - ry*qz + rz*qy],    
        [rx*qy - ry*qx + rz*qw,  rx*qz - ry*qw - rz*qx,  rx*qw + ry*qz - rz*qy,  rx*qx + ry*qy + rz*qz]      
    */
    float32_t qw = self->nominal_state.pData[6+0];
    float32_t qx = self->nominal_state.pData[6+1];
    float32_t qy = self->nominal_state.pData[6+2];
    float32_t qz = self->nominal_state.pData[6+3];

    float32_t rx = self->magnetic_vector_normalized.pData[0];
    float32_t ry = self->magnetic_vector_normalized.pData[1];
    float32_t rz = self->magnetic_vector_normalized.pData[2];
    set_element(&self->dmdq, 0, 0, 2.0f * ( rx*qw + ry*qz - rz*qy));
    set_element(&self->dmdq, 0, 1, 2.0f * ( rx*qx + ry*qy + rz*qz));
    set_element(&self->dmdq, 0, 2, 2.0f * (-rx*qy + ry*qx - rz*qw));
    set_element(&self->dmdq, 0, 3, 2.0f * (-rx*qz + ry*qw + rz*qx));
    set_element(&self->dmdq, 1, 0, 2.0f * (-rx*qz + ry*qw + rz*qx));
    set_element(&self->dmdq, 1, 1, 2.0f * ( rx*qy - ry*qx + rz*qw));
    set_element(&self->dmdq, 1, 2, 2.0f * ( rx*qx + ry*qy + rz*qz));
    set_element(&self->dmdq, 1, 3, 2.0f * (-rx*qw - ry*qz + rz*qy));
    set_element(&self->dmdq, 2, 0, 2.0f * ( rx*qy - ry*qx + rz*qw));
    set_element(&self->dmdq, 2, 1, 2.0f * ( rx*qz - ry*qw - rz*qx));
    set_element(&self->dmdq, 2, 2, 2.0f * ( rx*qw + ry*qz - rz*qy));
    set_element(&self->dmdq, 2, 3, 2.0f * ( rx*qx + ry*qy + rz*qz));
}
static int update_setup_meas_accelerometer(ErrorStateKalmanFilter *self, float32_t *pacc_meas) {
    /*
    transfer measurement to internal container
    transfer noise values into matrix
    normalize measurement
    normalize noise
    construct jacobian
    */

    int error;
    for (int i=0;i<3;i++) {
        set_element(&self->acc_meas, i, 0, pacc_meas[i]);
    }

    // how to handle the accelerometer noise duing update? Currenly done by dividing with the magnitude of the acceleration

    // bias correction
    self->acc_meas.pData[0] -= self->acc_bias.pData[0];
    self->acc_meas.pData[1] -= self->acc_bias.pData[1];
    self->acc_meas.pData[2] -= self->acc_bias.pData[2];

    // normalize measured vector and noise
    float32_t acc_norm = vecnorm(&self->acc_meas);
    if (acc_norm < DIVISION_BY_ZERO_THRESHOLD) {return ERROR_DIVISION_BY_ZERO;}
    error = arm_mat_scale_f32(&self->acc_meas, 1.0f / acc_norm, &self->acc_meas_normalized);
    if(error){return error;}
    // error = arm_mat_scale_f32(&self->acc_nois, 1.0 / acc_norm, &self->acc_nois_normalized);
    if(error){return error;}
    for (int i=0; i<3; i++) {
        // set_element(&self->V_acc, i, i, self->acc_noise_update_prescaler * self->acc_nois_normalized.pData[i]*self->acc_nois_normalized.pData[i]);
        // self->V_acc.pData[i*self->V_acc.numCols + i] = self->acc_noise_update_prescaler * self->acc_nois_normalized.pData[i]*self->acc_nois_normalized.pData[i];
        set_element(&self->V_acc, i, i, 0.1f * self->acc_nois.pData[i]*self->acc_nois.pData[i]);
    }
    
    // find the normalized expected gravity vector by rotating with inverse attitude
    intmedsetup(&self->intmed1, 4, 1);
    quaternion_inverse(&self->att, &self->intmed1);
    error = rotate_vector(
        &self->gravity_vector_normalized, 
        &self->intmed1, // inverse of attitude
        &self->expected_gravity_normalized);
    if(error){return error;}
    
    // setup H jacobian
    update_setup_meas_accelerometer_jacobian(self);

    return 0;
 
}
static int update_setup_meas_magnetometer(ErrorStateKalmanFilter *self, float32_t *pmag_meas) {
    /*
    transfer measurement to internal container
    transfer noise values into matrix
    normalize measurement
    normalize noise
    construct jacobian
    */
    int error;

    for (int i=0; i<3; i++) {
        set_element(&self->mag_meas, i, 0, pmag_meas[i]);
    }

    // normalize measured vector and noise
    float32_t mag_norm = vecnorm(&self->mag_meas);
    if (mag_norm < DIVISION_BY_ZERO_THRESHOLD){return ERROR_DIVISION_BY_ZERO;}
    error = arm_mat_scale_f32(&self->mag_meas, 1.0f / mag_norm, &self->mag_meas_normalized);
    if(error){return error;}
    error = arm_mat_scale_f32(&self->mag_nois, 1.0f / mag_norm, &self->mag_nois_normalized);
    if(error){return error;}
    for (int i=0; i<3; i++) {
        // set_element(&self->V_mag, i, i, self->mag_nois_normalized.pData[i]*self->mag_nois_normalized.pData[i]);
        // self->V_mag.pData[i*self->V_mag.numCols + i] = self->mag_nois_normalized.pData[i]*self->mag_nois_normalized.pData[i];
        set_element(&self->V_mag, i, i, self->mag_nois_normalized.pData[i]*self->mag_nois_normalized.pData[i]);
    }
    // find normalized expected magnetic field vector by rotating with inverse attitude
    intmedsetup(&self->intmed1, 4, 1);
    quaternion_inverse(&self->att, &self->intmed1);
    error = rotate_vector(
        &self->magnetic_vector_normalized,
        &self->intmed1, // inverse of attitude
        &self->expected_magnetic_field_normalized);
    if(error){return error;}

    // setup H jacobian
   update_setup_meas_magnetometer_jacobian(self);
    
   return 0;
}
static int update_setup_meas(ErrorStateKalmanFilter *self,
    float32_t *pgns_meas, float32_t *pbar_meas, 
    float32_t *pacc_meas, float32_t *pmag_meas,
    uint8_t gns_new, uint8_t bar_new, 
    uint8_t acc_new, uint8_t mag_new) {

    int error;


    if (gns_new) {
        update_setup_meas_GNSS(self, pgns_meas);
    }
    if (bar_new) {
        update_setup_meas_barometer(self, pbar_meas);
    }
    if (acc_new) {
        error = update_setup_meas_accelerometer(self, pacc_meas);
        if(error){return error;}
    }            
    if (mag_new) {
        error = update_setup_meas_magnetometer(self, pmag_meas);
        if(error){return error;}
    }

    return 0;
}
static int update_setup_jacobian_resize_and_zeroize(ErrorStateKalmanFilter *self, uint8_t gns_new, uint8_t bar_new, uint8_t acc_new, uint8_t mag_new) {

    int error;
    // # if no measurements: observation dimension = 0
    int obs_dim = 0;
    
    // available sensor measurements determine matrix dimensions
    if (gns_new) {
        obs_dim += 3; //# x, y, z
        // bar_new = 0; //# GNSS overrules barometer <- this could be changed. it is unnessecary, but simple
    } 
    if (bar_new) {obs_dim += 1;} //# only z
    if (acc_new) {obs_dim += 3;} //# ax, ay, az
    if (mag_new) {obs_dim += 3;} //# mx, my, mz
    
    if (obs_dim == 0) {
        // # no update no problem
        return 0;   
    }
    
    // # initialize self.measurement_vector, h, Hx depending on available measurements
    error = mat_resize(&self->measurement_vector, obs_dim, 1);                  if(error){return error;}
    error = mat_resize(&self->h, obs_dim, 1);                                   if(error){return error;}
    error = mat_resize(&self->Hx, obs_dim, 10);                                 if(error){return error;}
    error = mat_resize(&self->V, obs_dim, obs_dim);                             if(error){return error;}
    error = mat_resize(&self->H, self->Hx.numRows, 9);                          if(error){return error;}
    error = mat_resize(&self->H_T, self->H.numCols, self->H.numRows);           if(error){return error;}
    error = mat_resize(&self->Kalman_gain, 9, self->h.numRows);                 if(error){return error;}
    error = mat_resize(&self->S, self->V.numRows, self->V.numCols);             if(error){return error;}
    error = mat_resize(&self->innovation, self->h.numRows, self->h.numCols);    if(error){return error;}
    
    // zeroize
    set_zero(&self->measurement_vector);
    set_zero(&self->h);
    set_zero(&self->Hx);
    set_zero(&self->V);
    set_zero(&self->H);       
    set_zero(&self->H_T);
    set_zero(&self->Kalman_gain);
    set_zero(&self->S);
    set_zero(&self->innovation);

    return 0;
}
static void update_setup_jacobian_place_measurements_and_noise(ErrorStateKalmanFilter *self, uint8_t gns_new, uint8_t bar_new, uint8_t acc_new, uint8_t mag_new) {
    // fill in the measurement vector, h, Hx and V
    // # selector
    int idx_dim = 0;
    nominal_state2helpers(self);

    if (gns_new) {
        for (int i=0;i<3;i++) {
            set_element(&self->measurement_vector, idx_dim, 0, self->gns_meas.pData[i]);
            set_element(&self->h, idx_dim, 0, self->pos.pData[i]);
            set_element(&self->V, idx_dim, idx_dim, self->V_gns.pData[i*self->V_gns.numCols + i]);
            set_element(&self->Hx, idx_dim, i, 1.0f);
            idx_dim += 1;
        }
    }
    if (bar_new) {
        for (int i=0; i<1; i++) { // pseudo loop for selector. barometer only holds one value
            set_element(&self->measurement_vector, idx_dim, 0, get_element(&self->bar_meas, 0, 0));
            set_element(&self->h, idx_dim, 0, get_element(&self->pos, 2, 0));
            set_element(&self->V, idx_dim, idx_dim, get_element(&self->V_bar, 0, 0));
            set_element(&self->Hx, idx_dim, 2, 1.0f);
            idx_dim += 1;
        }
    }
    if (acc_new) {
        for (int i=0; i<3; i++) {
            set_element(&self->measurement_vector, idx_dim, 0, get_element(&self->acc_meas_normalized, i, 0));
            set_element(&self->h, idx_dim, 0, get_element(&self->expected_gravity_normalized, i, 0));
            set_element(&self->V, idx_dim, idx_dim, get_element(&self->V_acc, i, i));
            for (int j=0; j<4; j++) {
                set_element(&self->Hx, idx_dim, j+6, get_element(&self->dadq, i, j));
            }
            idx_dim += 1;
        }
    }
    if (mag_new) {
        for (int i=0; i<3; i++) {
            set_element(&self->measurement_vector, idx_dim, 0, get_element(&self->mag_meas_normalized, i, 0));
            set_element(&self->h, idx_dim, 0, get_element(&self->expected_magnetic_field_normalized, i, 0));
            set_element(&self->V, idx_dim, idx_dim, get_element(&self->V_mag, i, i));
            for (int j=0; j<4; j++) {
                set_element(&self->Hx, idx_dim, j+6, get_element(&self->dmdq, i, j));
            }
            idx_dim += 1;
        }
    }
    helpers2nominal_state(self);
}
static int update_setup_jacobian_X_Q_angular_error(ErrorStateKalmanFilter *self) {
    /*
    // # The derivative of the true state quaternion w/ resp. to the error state angular error is presented in the 
    // # litterature (Sola, 2017)

    Q_angular_error = 1/2 * array([
        [-qx, -qy, -qz],    
        [ qw, -qz,  qy],
        [ qz,  qw, -qx],
        [-qy,  qx,  qw]
        ]) 

    X_angluar_error = np.block([
        [I3,  O3,  O3,               ],
        [O3,  I3,  O3,               ],
        [O43, O43, Q_angular_error, ],
        ])
    */
    
    int error;
    nominal_state2helpers(self);

            // # some building blocks
    // # qw, qx, qy, qz = self.nominal_state.attitude
    float32_t qw = self->nominal_state.pData[6+0];
    float32_t qx = self->nominal_state.pData[6+1];
    float32_t qy = self->nominal_state.pData[6+2];
    float32_t qz = self->nominal_state.pData[6+3];

    set_element(&self->Q_angular_error, 0, 0, -qx * 0.5f);
    set_element(&self->Q_angular_error, 0, 1, -qy * 0.5f);
    set_element(&self->Q_angular_error, 0, 2, -qz * 0.5f);
    set_element(&self->Q_angular_error, 1, 0,  qw * 0.5f); 
    set_element(&self->Q_angular_error, 1, 1, -qz * 0.5f); 
    set_element(&self->Q_angular_error, 1, 2,  qy * 0.5f);
    set_element(&self->Q_angular_error, 2, 0,  qz * 0.5f);  
    set_element(&self->Q_angular_error, 2, 1,  qw * 0.5f);
    set_element(&self->Q_angular_error, 2, 2, -qx * 0.5f);
    set_element(&self->Q_angular_error, 3, 0, -qy * 0.5f);  
    set_element(&self->Q_angular_error, 3, 1,  qx * 0.5f);  
    set_element(&self->Q_angular_error, 3, 2,  qw * 0.5f);

    // identity is set during initialization
    // non-identity block
    for (int i = 0; i < 4; i++) {
        set_element(&self->X_angular_error, i,   i, 1.0f);
        set_element(&self->X_angular_error, i+3, i+3, 1.0f);
        for (int j = 0; j < 3; j++) {
            set_element(&self->X_angular_error, i+6, j+6, get_element(&self->Q_angular_error, i, j));
        }
    }

    // # self.H = self.Hx @ self.X_angular_error
    error = arm_mat_mult_f32(&self->Hx, &self->X_angular_error, &self->H);
    if(error){return error;}

    helpers2nominal_state(self);
    
    return 0;
}
static int update_calculate_error_state(ErrorStateKalmanFilter *self) {
    /*
    Calculate innovation covariance, Kalman gain, innovation and error state and update the covariance matrix 

    Two different but mathematically equivalent forms of the covariance update are possible. The 
    first one is more efficient, but the second one is more numerically stable. 
    Depending on the application, one might be preferred over the other.
    
    */
    nominal_state2helpers(self);
    error_state2helpers(self);


    int error;
    // innovation covariance = S = H @ P @ H.T + V
    error = arm_mat_trans_f32(&self->H, &self->H_T);
    if (error){return error;}
    intmedsetup(&self->intmed1, self->covariance.numRows, self->H_T.numCols);   
    intmedsetup(&self->intmed2, self->H.numRows, self->intmed1.numCols);
    error = arm_mat_mult_f32(&self->covariance, &self->H_T, &self->intmed1);                      // P @ H.T       <-- save this value to be reused later in Kalman gain calculation
    if (error){return error;}
    error = arm_mat_mult_f32(&self->H, &self->intmed1, &self->intmed2);                           // H @ P @ H.T
    if (error){return error;}
    error = arm_mat_add_f32(&self->intmed2, &self->V, &self->S);                                 // S = H @ P @ H.T + V
    if (error){return error;}
    
    
    // compute Kalman gain K = P @ H.T (H @ P @ H.T + V)^-1 = (P @ H.T) @ S^-1
    // this has recently been changed to a different 
    // inversion method. It is less readable, but faster (roughly half the time)
    intmedsetup(&self->intmed3, self->S.numCols, self->S.numRows);
    intmedsetup(&self->intmed4, self->S.numRows, self->S.numCols);
    intmedsetup(&self->intmed2, self->S.numRows, self->S.numCols);
    error = matinv(&self->S, &self->intmed3);                              // S^-1
    if (error){return error;}
    intmedsetup(&self->intmed2, self->H_T.numRows, self->intmed3.numCols);
    error = arm_mat_mult_f32(&self->intmed1, &self->intmed3, &self->Kalman_gain);                 // K = (P @ H.T) @ S^-1             <-- intmed1 was saved from before, stored P @ H.T
    if (error){return error;}

    // innovation = measurement_vector - h
    error = arm_mat_sub_f32(&self->measurement_vector, &self->h, &self->innovation);             // innovation = z - h(x) = measurement_vector - h
    if (error){return error;}
    
    // reveal errors
    error = arm_mat_mult_f32(&self->Kalman_gain, &self->innovation, &self->error_state); //# directly write to error state
    if (error){return error;}

    // Do not transfer helpers to error state here


    return 0;
}
static int update_calculate_covariance(ErrorStateKalmanFilter *self) {
    /*
    fast form of the covariance calculation
    */

    int error;
    nominal_state2helpers(self);
    error_state2helpers(self);

    // # self.covariance = (np.eye(self.covariance.numRows) - self.K @ self.H) @ self.covariance
    intmedsetup(&self->intmed1, self->Kalman_gain.numRows, self->H.numCols);
    intmedsetup(&self->intmed2, self->I9.numRows, self->intmed1.numCols);
    error = arm_mat_mult_f32(&self->Kalman_gain, &self->H, &self->intmed1);                               // K @ H
    if (error){return error;}
    error = arm_mat_sub_f32(&self->I9, &self->intmed1, &self->intmed2);                                  // I - K @ H
    if (error){return error;}
    intmedsetup(&self->intmed1, self->covariance.numCols, self->covariance.numRows);
    error = arm_mat_mult_f32(&self->intmed2, &self->covariance, &self->intmed1);                          // P = (I - K @ H) @ P
    if (error){return error;}
    error = copy_matrix(&self->intmed1, &self->covariance);
    if (error){return error;}

    helpers2error_state(self);
    helpers2nominal_state(self);
    return 0;
}
static int update_calculate_covariance_Joseph_form(ErrorStateKalmanFilter *self) {
    /*
    more numerically stable form (Joseph form)

    can be chosen instead of the fast version, but should not be nessecary
    */

    int error;
    nominal_state2helpers(self);
    error_state2helpers(self);

    // # self.covariance = (self.I9 - Kalman_gain @ H) @ self.covariance @ (self.I9 - Kalman_gain @ H).T + Kalman_gain @ V @ Kalman_gain.T
    intmedsetup(&self->intmed1, self->Kalman_gain.numRows, self->H.numCols);
    error = arm_mat_mult_f32(&self->Kalman_gain, &self->H, &self->intmed1);
    if (error){return error;}
    intmedsetup(&self->intmed2, self->I9.numRows, self->intmed1.numCols);
    error = arm_mat_sub_f32(&self->I9, &self->intmed1, &self->intmed2);
    if (error){return error;}
    intmedsetup(&self->intmed1, self->intmed2.numCols, self->intmed2.numRows);
    error = arm_mat_trans_f32(&self->intmed2, &self->intmed1);                   //# right sandwich
    if (error){return error;}
    intmedsetup(&self->intmed3, self->covariance.numRows, self->intmed1.numCols);
    error = arm_mat_mult_f32(&self->covariance, &self->intmed1, &self->intmed3);
    if (error){return error;}
    intmedsetup(&self->intmed1, self->intmed2.numRows, self->intmed3.numCols);
    error = arm_mat_mult_f32(&self->intmed2, &self->intmed3, &self->intmed1);     //# first term
    if (error){return error;}
    intmedsetup(&self->intmed2, self->Kalman_gain.numCols, self->Kalman_gain.numRows);
    error = arm_mat_trans_f32(&self->Kalman_gain, &self->intmed2);
    if (error){return error;}
    intmedsetup(&self->intmed3, self->V.numRows, self->intmed2.numCols);
    error = arm_mat_mult_f32(&self->V, &self->intmed2, &self->intmed3);
    if (error){return error;}
    intmedsetup(&self->intmed2, self->Kalman_gain.numRows, self->intmed3.numCols);
    error = arm_mat_mult_f32(&self->Kalman_gain, &self->intmed3, &self->intmed2); //# second term
    if (error){return error;}
    error = arm_mat_add_f32(&self->intmed1, &self->intmed2, &self->covariance);
    if (error){return error;}

    helpers2error_state(self);
    helpers2nominal_state(self);
    return 0;

}
static int update_injection(ErrorStateKalmanFilter *self){
       
    int error;
    nominal_state2helpers(self);
    error_state2helpers(self);

    // debug
    // self->att_error.pData[0] = 0.0f;
    // self->att_error.pData[1] = 0.0f;
    // self->att_error.pData[2] = 0.0f;

    // # pos = pos + pos_error
    self->pos.pData[0] += self->pos_error.pData[0];
    self->pos.pData[1] += self->pos_error.pData[1];
    self->pos.pData[2] += self->pos_error.pData[2];
    
    // # vel = vel + vel_error 
    self->vel.pData[0] += self->vel_error.pData[0];
    self->vel.pData[1] += self->vel_error.pData[1];
    self->vel.pData[2] += self->vel_error.pData[2];
    
    // # att = att OX att_error
    // # convert angular error to quaternion
    error = rotation_vector2quaternion(&self->att_error, &self->q_error);
    if(error){return error;}
    // apply attitude errors using hamiltonian product
    intmedsetup(&self->intmed1, self->att.numRows, self->att.numCols);
    hamiltonian_product(&self->att, &self->q_error, &self->intmed1);
    error = copy_matrix(&self->intmed1, &self->att);
    if(error){return error;}
    // renormalize quaternion
    float32_t att_norm = vecnorm(&self->att);
    if (att_norm < DIVISION_BY_ZERO_THRESHOLD) {return ERROR_DIVISION_BY_ZERO;}
    for (int i=0; i<4; i++){
        self->att.pData[i] /= att_norm;
    }

    helpers2nominal_state(self);
    helpers2error_state(self);

    return 0;
}
static void update_reset_fast(ErrorStateKalmanFilter *self) {
    set_zero(&self->error_state);
}
static int update_reset_slow_and_precise(ErrorStateKalmanFilter *self) {

    /*
    Two ways of doing the reset exist. Fast versus accurate.
    
    The fast way just sets the error state to zero and does nothing to the covariance
    If extra accuracy is needed, the covariance matrix can be updated to reflect the small change in 
    attitude error. However, the change is so small and the calculations
    require relatively substantial matrix multiplications, so the fast variant is likely
    to result in better estimates over many iterations, simply because iteration speed
    improves the accuracy


    self.G = np.block([
            [I3, O3, O3,             ],
            [O3, I3, O3,             ],
            [O3, O3, non_zero_entry, ],
            ])
    
            most of the above G stays constant for every update, so we only change the lower right corner
    */

    int error;
    error_state2helpers(self);

    // # we need the error to update the covariance, so we reset the state last
    // # self.att_skew_matrix = construct_skew_matrix(self.error_state.attitude)
    error = construct_skew_matrix(&self->att_error, &self->att_skew_matrix);
    if(error){return error;}
    // # non_zero_entry = I3 - 1/2*skew_matrix
    intmedsetup(&self->intmed1, self->att_skew_matrix.numRows, self->att_skew_matrix.numCols);
    error = arm_mat_scale_f32(&self->att_skew_matrix, 0.5, &self->intmed1);
    if(error){return error;}
    
    // # intmed2 += self.I3 - intmed1 # is the non-zero entry
    intmedsetup(&self->intmed2, self->I3.numRows, self->intmed1.numCols);
    error = arm_mat_sub_f32(&self->I3, &self->intmed1, &self->intmed2);
    if(error){return error;}
    
    // place the non zero entry
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            set_element(&self->G, i+6, j+6, get_element(&self->intmed2, i, j));
        }
    }
    
    // # self.covariance = G @ self.covariance @ G.T
    intmedsetup(&self->intmed1, self->G.numCols, self->G.numRows);
    error = arm_mat_trans_f32(&self->G, &self->intmed1);                                             // G.T
    if(error){return error;}
    intmedsetup(&self->intmed2, self->covariance.numRows, self->intmed1.numCols);
    error = arm_mat_mult_f32(&self->covariance, &self->intmed1, &self->intmed2);                      // P @ G.T
    if(error){return error;}
    
    // # self.covariance = self.G @ intmed2
    error = arm_mat_mult_f32(&self->G, &self->intmed2, &self->covariance);
    if(error){return error;}                                                                // P = G @ P @ G.T
    set_zero(&self->error_state);

    
    error_state2helpers(self);

    return 0;
}
int update(ErrorStateKalmanFilter *self,
    float32_t *pgns_meas, float32_t *pbar_meas, 
    float32_t *pacc_meas, float32_t *pmag_meas,
    uint8_t gns_new, uint8_t bar_new, 
    uint8_t acc_new, uint8_t mag_new){

    int error;

    // no need to run if no measurements available
    if (!gns_new && !bar_new && !acc_new && !mag_new){
        return 0;
    }

    // only use accelerometer if stationary
    if (acc_new) {
        // Calculate magnitude squared: x^2 + y^2 + z^2
        float32_t acc_mag_sq = (pacc_meas[0] * pacc_meas[0]) + 
                               (pacc_meas[1] * pacc_meas[1]) + 
                               (pacc_meas[2] * pacc_meas[2]);
        float32_t acc_mag;
        error = arm_sqrt_f32(acc_mag_sq, &acc_mag);
        if(error){return error;}
        if (fabsf(acc_mag - GRAVITY) > ACC_REJECTION_THRESHOLD) {
            acc_new = 0;
        }

        float32_t speed_sq = 0.0f;
        for (int i=0; i<3; i++){
            speed_sq += self->nominal_state.pData[3+i]*self->nominal_state.pData[3+i];
        }
        if (speed_sq > 100) {
            acc_new = 0;
        }

    }
    // do the update sequence
    // if (gns_new){bar_new = 0;}
    
    error = update_setup_meas(self, pgns_meas, pbar_meas, pacc_meas, pmag_meas, gns_new, bar_new, acc_new, mag_new); if(error){return error;}
    error = update_setup_jacobian_resize_and_zeroize(self, gns_new, bar_new, acc_new, mag_new); if(error){return error;}
            update_setup_jacobian_place_measurements_and_noise(self, gns_new, bar_new, acc_new, mag_new);
    error = update_setup_jacobian_X_Q_angular_error(self); if(error){return error;}
    error = update_calculate_error_state(self); if(error){return error;}
    error = update_calculate_covariance(self); if(error){return error;}
    // error = update_calculate_covariance_Joseph_form(self); if(error){return error;}
    error = update_injection(self); if(error){return error;}
            update_reset_fast(self);

    return 0;
}

