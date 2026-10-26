


// /*
// primary purpose is testing of the Error-state Kalman Filter
// */


// #include <stdio.h>

// // #include "arm_math.h"
// // #include "linalg.h"
// // #include "quaternions.h"
// // #include "dsp/quaternion_math_functions.h"
// // #include "ESKF.h"
// #include "estimator.h"
// #include "ESKF.h"
// #include "linalg.h"
// #include "quaternions.h"
// #include "measurement_handling.h"



// static Estimator est;

// void printmatrix(arm_matrix_instance_f32 *pMatrix) {
// 	for (int i = 0; i<pMatrix->numRows; i++) {
// 		for (int j=0; j<pMatrix->numCols; j++) {
// 			printf("%f, ", pMatrix->pData[i * pMatrix->numCols + j]);
// 		}	
// 		printf("\n");
// 	}	

// }

// int main(void) {

// 	// initialize some artificial noises and measurements
// 	float32_t GNSS_noise[3]	 = {1.0, 1.0, 1.0};
// 	float32_t barometer_noise[1] = {1.0};
// 	float32_t accelerometer_noise[3] = {1.0, 1.0, 2.0};
// 	float32_t gyroscope_noise[3] = {1.0, 1.0, 1.0};
// 	float32_t magnetometer_noise[3] = {10.0, 10.0, 10.0};
// 	float32_t gns_meas[3] = {0.0, 0.0, 0.0};
// 	float32_t bar_meas[1] = {0.0};
// 	float32_t acc_meas[3] = {0.0, 0.0, 9.82};
// 	float32_t gyr_meas[3] = {0.1, 0.2, 0.1};
// 	// float32_t mag_meas[3] = {0.1, 0.1, 10.0};
// 	float32_t mag_meas[3] = {0.0, 0.0, 0.0};
	
// 	int use_stable_nominal_kinematics = 0;

// 	estimator_init(
// 		&est, 	
// 		GNSS_noise,
// 		accelerometer_noise,
// 		gyroscope_noise,
// 		magnetometer_noise,
// 		barometer_noise,
// 		use_stable_nominal_kinematics
// 	);

// 	// using the estimator (and the internal Kalman filter)
// 	int i = 0;
// 	while (i<1000000) {		
// 		// printf("pos: %f, %f, %f\n", est.eskf.nominal_state.pData[0], est.eskf.nominal_state.pData[1], est.eskf.nominal_state.pData[2]);
// 		// printf("vel: %f, %f, %f\n", est.eskf.nominal_state.pData[3], est.eskf.nominal_state.pData[4], est.eskf.nominal_state.pData[5]);
// 		// printf("att: %f, %f, %f, %f\n", est.eskf.nominal_state.pData[6], est.eskf.nominal_state.pData[7], est.eskf.nominal_state.pData[8], est.eskf.nominal_state.pData[9]);
// 		estimate(
// 			&est,
// 			gns_meas,
// 			bar_meas,
// 			acc_meas, 
// 			gyr_meas,
// 			mag_meas, 
// 			0.1, 0.2,    	// delta t 
// 			1, 1, 1, 1, 1	// new measurement mask
// 		); 

	
// 		i++;
// 	}

// 	// printf("Nominal state\n");
// 	// printmatrix(&est.eskf.nominal_state);
// 	// printf("Covariance\n");
// 	// printmatrix(&est.eskf.covariance);

// }




