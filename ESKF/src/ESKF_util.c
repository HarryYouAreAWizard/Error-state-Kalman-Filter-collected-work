


/*
Error-State Kalman Filter utility

previously stored the entire filter. Now the main body is split in multiple files

the keyword "self" is chosen for the ESKF in the functions, such 
that the transition from python to C is easier 

*/

#include "ESKF.h"

// #include <stdio.h>
// #include "user/logging.h"

// helper states <-> state vectors
void nominal_state2helpers(ErrorStateKalmanFilter *self) {
    /*transfer data from nominal state vector into helpers*/
    self->pos.pData[0] = self->nominal_state.pData[0];
    self->pos.pData[1] = self->nominal_state.pData[1];
    self->pos.pData[2] = self->nominal_state.pData[2];
    self->vel.pData[0] = self->nominal_state.pData[3];
    self->vel.pData[1] = self->nominal_state.pData[4];
    self->vel.pData[2] = self->nominal_state.pData[5];
    self->att.pData[0] = self->nominal_state.pData[6];
    self->att.pData[1] = self->nominal_state.pData[7];
    self->att.pData[2] = self->nominal_state.pData[8];
    self->att.pData[3] = self->nominal_state.pData[9];
}
void helpers2nominal_state(ErrorStateKalmanFilter *self) {
    /*transfer data from helpers into nominal state vector*/
    self->nominal_state.pData[0] = self->pos.pData[0];
    self->nominal_state.pData[1] = self->pos.pData[1];
    self->nominal_state.pData[2] = self->pos.pData[2];
    self->nominal_state.pData[3] = self->vel.pData[0];
    self->nominal_state.pData[4] = self->vel.pData[1];
    self->nominal_state.pData[5] = self->vel.pData[2];
    self->nominal_state.pData[6] = self->att.pData[0];
    self->nominal_state.pData[7] = self->att.pData[1];
    self->nominal_state.pData[8] = self->att.pData[2];
    self->nominal_state.pData[9] = self->att.pData[3];
}
void error_state2helpers(ErrorStateKalmanFilter *self) {
    self->pos_error.pData[0] = self->error_state.pData[0];
    self->pos_error.pData[1] = self->error_state.pData[1];
    self->pos_error.pData[2] = self->error_state.pData[2]; 
    self->vel_error.pData[0] = self->error_state.pData[3];
    self->vel_error.pData[1] = self->error_state.pData[4];
    self->vel_error.pData[2] = self->error_state.pData[5];
    self->att_error.pData[0] = self->error_state.pData[6];
    self->att_error.pData[1] = self->error_state.pData[7];
    self->att_error.pData[2] = self->error_state.pData[8];
}
void helpers2error_state(ErrorStateKalmanFilter *self) {
    self->error_state.pData[0] = self->pos_error.pData[0];
    self->error_state.pData[1] = self->pos_error.pData[1];
    self->error_state.pData[2] = self->pos_error.pData[2]; 
    self->error_state.pData[3] = self->vel_error.pData[0];
    self->error_state.pData[4] = self->vel_error.pData[1];
    self->error_state.pData[5] = self->vel_error.pData[2];
    self->error_state.pData[6] = self->att_error.pData[0];
    self->error_state.pData[7] = self->att_error.pData[1];
    self->error_state.pData[8] = self->att_error.pData[2];
}
