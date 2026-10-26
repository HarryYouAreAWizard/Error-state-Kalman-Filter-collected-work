

/*
    Source file for the Error-State Kalman Filter predict step
*/


#include "ESKF.h"

// predict step function
static int predict_nominal_transform_accelerometer(ErrorStateKalmanFilter *self, float32_t *pacc_meas, float32_t delta_t_acc, uint8_t acc_new) {
    /*
    the transfer to pos, vel and att must happen before the function

    if someting should be wrong with the accelerometer data, we do nothing
    */

    int error;
    nominal_state2helpers(self);

    // set acc to zero if no measurment is available
    if (!acc_new) {
        for (int i = 0; i < 3; i++) {
            set_element(&self->acc_meas, i, 0, 0.0f);
        }        
    }
    else {
        // # transfer measurement to internal variable
        for (int i = 0; i < 3; i++) {
            set_element(&self->acc_meas, i, 0, pacc_meas[i]);
        }
    }
    
    // subtract bias
    self->acc_meas.pData[0] -= self->acc_bias.pData[0];
    self->acc_meas.pData[1] -= self->acc_bias.pData[1];
    self->acc_meas.pData[2] -= self->acc_bias.pData[2];

    // # self.R_att = rotation_matrix_from_quaternion(att)
    error = quaternion2rotation_matrix(&self->att, &self->R_att); if(error){return error;}

    // # true_acceleration = R_att @ accelerometer_measurement - self.gravity_vector_unnormalized
    intmedsetup(&self->intmed1, self->R_att.numRows, self->acc_meas.numCols);
    error = arm_mat_mult_f32(&self->R_att, &self->acc_meas, &self->intmed1); if(error){return error;}

    error = arm_mat_sub_f32(&self->intmed1, &self->gravity_vector_unnormalized, &self->true_acc); if(error){return error;}
    // error = arm_mat_add_f32(&self->intmed1, &self->gravity_vector_unnormalized, &self->true_acc); if(error){return error;}

    helpers2nominal_state(self);
    
    return 0;
}
static int predict_nominal_position(ErrorStateKalmanFilter *self, float32_t delta_t_acc) {
    /*
    pos = pos + vel * delta_t + 1/2 * true_acceleration * delta_t**2

    true acceleration found the transfer accelerometer function
    */
    int error;
    nominal_state2helpers(self);
    
    intmedsetup(&self->intmed1, self->true_acc.numRows, self->true_acc.numCols);
    intmedsetup(&self->intmed2, self->vel.numRows, self->vel.numCols);
    intmedsetup(&self->intmed3, self->pos.numRows, self->pos.numCols);
    error = arm_mat_scale_f32(&self->true_acc, 0.5 * delta_t_acc * delta_t_acc, &self->intmed1);     // 1/2 * true_acceleration * delta_t_acc**2
    if(error){return error;}
    error = arm_mat_scale_f32(&self->vel, delta_t_acc, &self->intmed2);                              // vel * delta_t_acc
    if(error){return error;}
    error = arm_mat_add_f32(&self->pos, &self->intmed1, &self->intmed3);                             // pos + 1/2 * true_acceleration * delta_t_acc**2
    if(error){return error;}
    error = arm_mat_add_f32(&self->intmed3, &self->intmed2, &self->pos);                             // pos = pos + vel * delta_t_acc + 1/2 * true_acceleration * delta_t_acc**2
    if(error){return error;}

    helpers2nominal_state(self);
    return 0;

}
static int predict_nominal_velocity(ErrorStateKalmanFilter *self, float32_t delta_t_acc) {
    /*
    vel = vel + true_acceleration * delta_t

    true acceleration found the transfer accelerometer function
    */
    int error;
    nominal_state2helpers(self);

    intmedsetup(&self->intmed1, self->true_acc.numRows, self->true_acc.numCols);
    intmedsetup(&self->intmed2, self->vel.numRows, self->vel.numCols);
    error = arm_mat_scale_f32(&self->true_acc, delta_t_acc, &self->intmed1);                         // true_acceleration * delta_t_acc
    if(error){return error;}
    error = arm_mat_add_f32(&self->vel, &self->intmed1, &self->intmed2);                             // vel + true_acceleration * delta_t_acc
    if(error){return error;}
    copy_matrix(&self->intmed2, &self->vel);                                        // vel = vel + true_acceleration * delta_t_acc
    
    helpers2nominal_state(self);
    return 0;
}
static int predict_nominal_transform_gyroscope(ErrorStateKalmanFilter *self, float32_t *pgyr_meas, float32_t delta_t_gyr, uint8_t gyr_new) {
    /*
    if something should be wrong with the gyroscope measurement (gyr_new = 0) then we still scale the old measurement with the 
    new delta_t_gyr
    */
    int error;
    nominal_state2helpers(self);
    if (gyr_new) {
        // # transfer measurement to internal variable
        for (int i = 0; i < 3; i++) {
            set_element(&self->gyr_meas, i, 0, pgyr_meas[i]);
        }   
        // # gyroscope_measurement = gyroscope_measurement - self.gyr_bias
        self->gyr_meas.pData[0] -= self->gyr_bias.pData[0];
        self->gyr_meas.pData[1] -= self->gyr_bias.pData[1];
        self->gyr_meas.pData[2] -= self->gyr_bias.pData[2];
    }
    // construct quaternion from gyroscope measurement
    error = arm_mat_scale_f32(&self->gyr_meas, delta_t_gyr, &self->rotation_vector);
    if(error){return error;}
    error = rotation_vector2quaternion(&self->rotation_vector, &self->q_gyr);
    if(error){return error;}
    
    helpers2nominal_state(self);
    return 0;
}
static int predict_nominal_attitude(ErrorStateKalmanFilter *self) {
    /*
    att = hamiltonian_product(att, self.q_gyr)
    */
    int error;
    nominal_state2helpers(self);

    intmedsetup(&self->intmed1, self->att.numRows, self->att.numCols);
    hamiltonian_product(&self->att, &self->q_gyr, &self->intmed1);
    float32_t att_norm = vecnorm(&self->intmed1);
    if (att_norm < DIVISION_BY_ZERO_THRESHOLD){return ERROR_DIVISION_BY_ZERO;}
    error = arm_mat_scale_f32(&self->intmed1, 1.0f / att_norm, &self->att);
    if(error){return error;}

    helpers2nominal_state(self);
    return 0;
}
static int predict_error_construct_rotated_acceleration_skew_matrix(ErrorStateKalmanFilter *self, float32_t delta_t_acc) {
    /*
    construct rotated acc skew matrix
    
    self.rotated_acceleration_skew_matrix = -self.R_att @ construct_skew_matrix(accelerometer_measurement)*delta_t
    
    */
    
    int error;
    nominal_state2helpers(self);

    intmedsetup(&self->intmed1, self->acc_meas.numRows, self->acc_meas.numRows);
    error = construct_skew_matrix(&self->acc_meas, &self->intmed1);                         // skew matrix
    if(error){return error;}
    intmedsetup(&self->intmed2, self->intmed1.numRows, self->intmed1.numCols);
    error = arm_mat_scale_f32(&self->intmed1, delta_t_acc, &self->intmed2);                          // delta_t_acc * skew matrix
    if(error){return error;}
    // intmedsetup(&self->intmed1, self->R_att.numRows, self->intmed2.numCols);
    error = arm_mat_mult_f32(&self->R_att, &self->intmed2, &self->rotated_acceleration_skew_matrix);  // delta_t_acc * R_att * skew matrix
    if(error){return error;}
    
    helpers2nominal_state(self);
    return 0;
}
static int predict_error_construct_gyroscope_rotation_matrix(ErrorStateKalmanFilter *self, float32_t delta_t_gyr) {
    /*
    use gyroscope measurement and time span to find a rotation matrix
    first a rotation vector is found, then converted to a rotation matrix
    */
    int error;
    nominal_state2helpers(self);

    error = arm_mat_scale_f32(&self->gyr_meas, delta_t_gyr, &self->rotation_vector);         if(error){return error;}
    error = rotation_vector2rotation_matrix(&self->rotation_vector, &self->R_gyr);  if(error){return error;}
    error = arm_mat_trans_f32(&self->R_gyr, &self->R_gyr_T);                                 if(error){return error;}

    helpers2nominal_state(self);
    return 0;
}
static void predict_error_populate_matrices(ErrorStateKalmanFilter *self, float32_t delta_t_acc, float32_t delta_t_gyr) {
    /*
    set matrices in one loop

        Construct the jacobians needed for the error prediction step

    everyting is fixed size
    everything is set to run regardless of whether a measurement is new

    Fx, Vi and Thetai are constructed from accelerometer and gyroscope

    Qi depends on Vi and Thetai

    Fx: propagation of covariance depending on nominal state, acc measurement and gyr measurement  
    Fi: propagation of covariance depending on process noise
    Qi: process noise 

    
    // matrices in python syntax, following Joan Sola
    Fx = np.block([
        [I3, delta_t_acc*I3, O3],
        [O3, I3,            -self.R_att @ construct_skew_matrix(accelerometer_measurement)*delta_t_acc],
        [O3, O3,             R_gyr_T]
        
    Fi = np.blocks([
        [O3, O3],
        [I3, O3],
        [O3, I3]
        ])
        
    Qi = np.blocks([
        [Vi, O3],
        [O3, Thetai]
        ])

    */
    nominal_state2helpers(self);

    for (int i=0;i<3;i++) {
        // position - velocity coupling
        set_element(&self->Fx, i,   i+3, delta_t_acc);

        // two blocks in Fx 
        for (int j=0;j<3;j++) {
            set_element(&self->Fx, i+3, j+6, -self->rotated_acceleration_skew_matrix.pData[i*self->rotated_acceleration_skew_matrix.numCols + j]);
                                            //^notice the sign here
            set_element(&self->Fx, i+6, j+6, self->R_gyr_T.pData[i*self->R_gyr_T.numCols + j]);
        }

        // Process noise
        // sorta funcky. Why put the noise in a matrix before putting it in another matrix?
        //  - to follow notation from (Sola)
        // Vi
        set_element(&self->Vi, i, i, self->acc_nois.pData[i]*self->acc_nois.pData[i] * delta_t_acc*delta_t_acc);
        // Thetai
        set_element(&self->Thetai, i, i, self->gyr_nois.pData[i]*self->gyr_nois.pData[i] * delta_t_gyr*delta_t_gyr); 

        // construct Qi from Vi and Thetai
        set_element(&self->Qi, i, i, self->Vi.pData[i*self->Vi.numCols + i]);
        set_element(&self->Qi, i+3, i+3, self->Thetai.pData[i*self->Thetai.numCols + i]);
    }

    helpers2nominal_state(self);
}
static int predict_error_populate_Fx_second_order(ErrorStateKalmanFilter *self, float32_t delta_t_acc, float32_t delta_t_gyr) {
    /*
    Optional
    For higher precision the second order Fx can be used instead
    */
    // second order Fx

    int error;
    nominal_state2helpers(self);
    // - R @ skew(acc) @ (I3 - 1/2 skew(omega*delta_t)) delta_t
    intmedsetup(&self->intmed1, 3, 3);
    intmedsetup(&self->intmed2, 3, 3);
    error = construct_skew_matrix(&self->rotation_vector, &self->intmed1);                      // skew (omega*delta_t)
    if(error){return error;}
    error = arm_mat_scale_f32(&self->intmed1, 1/2, &self->intmed2);                                      // 1/2 * skew (omega*delta_t)
    if(error){return error;}
    intmedsetup(&self->intmed1, 3, 3);
    error = arm_mat_sub_f32(&self->I3, &self->intmed2, &self->intmed1);                                  // (I3 - 1/2 skew(omega*delta_t))
    if(error){return error;}
    intmedsetup(&self->intmed2, 3, 3);
    error = arm_mat_mult_f32(&self->rotated_acceleration_skew_matrix, &self->intmed1, &self->intmed2);    // R @ skew(acc) @ (I3 - 1/2 skew(omega*delta_t))
    if(error){return error;}
    intmedsetup(&self->intmed1, 3, 3);
    error = arm_mat_scale_f32(&self->intmed2, -delta_t_gyr, &self->intmed1);                             // - R @ skew(acc) @ (I3 - 1/2 skew(omega*delta_t)) delta_t
    if(error){return error;}

    for (int i=0;i<3;i++){
        // identity on pos and vel diag
        set_element(&self->Fx, i, i, 1.0f);
        set_element(&self->Fx, i+3, i+3, 1.0f);
        // pos vel coupling
        set_element(&self->Fx, i, i+3, delta_t_acc);

        for (int j=0;j<3;j++){
            // gyroscope
            set_element(&self->Fx, i+6, j+6, self->R_gyr_T.pData[i*self->R_gyr.numCols + j]);

            // second order effects
            set_element(&self->Fx, i,   j+6, -1/2 * delta_t_acc * delta_t_acc * self->rotated_acceleration_skew_matrix.pData[i*self->rotated_acceleration_skew_matrix.numCols + j]);
            set_element(&self->Fx, i+3, j+6, self->intmed1.pData[i*self->intmed1.numCols + j]);
        }
    }

    helpers2nominal_state(self);
    return 0;
}
static int predict_error_covariance_propagation(ErrorStateKalmanFilter *self) {
    /*
    use the constructed jacobians to propagate the covariance matrix
    error_state = Fx @ error_state       <- not actually run
    covariance = Fx @ covariance @ Fx.T + Fi @ Qi @ Fi.T

    // this function is very slow (~5 ms on the microcontroller)    

    */
    
    int error;
    nominal_state2helpers(self);

    // first term
    intmedsetup(&self->intmed1, self->Fx.numCols, self->Fx.numRows);                
    intmedsetup(&self->intmed2, self->covariance.numRows, self->intmed1.numCols);
    intmedsetup(&self->intmed3, self->Fx.numRows, self->intmed2.numCols);
    error = arm_mat_trans_f32(&self->Fx, &self->intmed1);                                              //# Fx.T
    if(error){return error;}
    error = arm_mat_mult_f32(&self->covariance, &self->intmed1, &self->intmed2);                        //# covariance @ Fx.T
    if(error){return error;}
    error = arm_mat_mult_f32(&self->Fx, &self->intmed2, &self->intmed3);                                //# Fx @ covariance @ Fx.T
    if(error){return error;}
    
    // second term
    intmedsetup(&self->intmed1, self->Fi.numCols, self->Fi.numRows);
    intmedsetup(&self->intmed2, self->Qi.numRows, self->intmed1.numCols);
    intmedsetup(&self->intmed4, self->Fi.numRows, self->intmed2.numCols);
    error = arm_mat_trans_f32(&self->Fi, &self->intmed1);                                              //# Fi.T
    if(error){return error;}
    error = arm_mat_mult_f32(&self->Qi, &self->intmed1, &self->intmed2);                                //# Qi @ Fi.T
    if(error){return error;}
    error = arm_mat_mult_f32(&self->Fi, &self->intmed2, &self->intmed4);                                //# Fi @ Qi @ Fi.T
    if(error){return error;}
    
    // add the two terms
    error = arm_mat_add_f32(&self->intmed3, &self->intmed4, &self->covariance);                        //# covariance = Fx @ covariance @ Fx.T + Fi @ Qi @ Fi.T  
    if(error){return error;}

    helpers2nominal_state(self);
    return 0;
}

int predict(ErrorStateKalmanFilter *self,
    float32_t *pacc_meas, float32_t *pgyr_meas, 
    float32_t delta_t_acc, float32_t delta_t_gyr,
    uint8_t acc_new, uint8_t gyr_new) {

    int error;

    error = predict_nominal_transform_accelerometer(self, pacc_meas, delta_t_acc, acc_new);     if(error){return error;}
    error = predict_nominal_transform_gyroscope(self, pgyr_meas, delta_t_gyr, gyr_new);         if(error){return error;}
    error = predict_nominal_position(self, delta_t_acc);                                        if(error){return error;}
    error = predict_nominal_velocity(self, delta_t_acc);                                        if(error){return error;}
    error = predict_nominal_attitude(self);                                                     if(error){return error;}
    error = predict_error_construct_rotated_acceleration_skew_matrix(self, delta_t_acc);        if(error){return error;}
    error = predict_error_construct_gyroscope_rotation_matrix(self, delta_t_gyr);               if(error){return error;}
            predict_error_populate_matrices(self, delta_t_acc, delta_t_gyr);
            // predict_error_populate_Fx_second_order(self, delta_t_acc, delta_t_gyr);
    error = predict_error_covariance_propagation(self);                                         if(error){return error;}

    return 0;
}

