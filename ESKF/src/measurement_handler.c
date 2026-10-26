

#include <stdint.h>
#include "arm_math.h" // Required for float32_t and arm_cos_f32
#include "linalg.h"
// #include "stdio.h"

/*Google Gemini helped make this.*/

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


#define R_EARTH_WGS84 6378137.0f // WGS-84 equatorial radius in meters

#define EQUATORIAL_RADIUS 6378137.0f // m
#define POLAR_RADIUS 6356752.0f // m 

#define a2 (EQUATORIAL_RADIUS * EQUATORIAL_RADIUS)
#define b2 (POLAR_RADIUS * POLAR_RADIUS)
#define e2 (1.0f - b2/a2)


float32_t prime_vertical_radius_of_curvature(float32_t latitude_geodetic_rad) {
    /*
    used for GNSS -> local position

    GNSS angle measurements typically come as geodetics
    angles must be converted to radians beforehand 
    */ 
    return a2 / sqrtf(a2*cosf(latitude_geodetic_rad)*cosf(latitude_geodetic_rad) + b2 * sinf(latitude_geodetic_rad) * sinf(latitude_geodetic_rad));
}


int latlonalt_to_xyz_m(const float32_t *pGNSS_latlonalt, 
                       float32_t lat_ref,
                       float32_t lon_ref,
                       float32_t alt_ref,
                       float32_t *pResult) {
    /*
    Convert a GNSS position (latitude, longitude, altitude) to a position in a 
    flat coordinate system defined by some reference.

    Following formulas from 
    https://en.wikipedia.org/wiki/Geographic_coordinate_conversion

    notation:
    _LP : launchpad
    _r : rocket

    on wiki-page _r is for radar (the reference), which is opposite for our case (rocket)

    all inputs angles are in degrees
    */

    // Unpack rocket position
    float32_t lat = pGNSS_latlonalt[0];
    float32_t lon = pGNSS_latlonalt[1];
    float32_t alt = pGNSS_latlonalt[2];

    // convert to radians
    float32_t phi_LP = lat_ref * PI/180.0f;
    float32_t lambda_LP = lon_ref * PI/180.0f;
    float32_t h_LP = alt_ref;
    
    float32_t phi_R = lat*PI/180.0f;
    float32_t lambda_R = lon * PI/180.0f;
    float32_t h_R = alt;

    // ECEF position of launchpad
    // prime vertical radius of curvature for launchpad
    float32_t N_LP = prime_vertical_radius_of_curvature(phi_LP);
    float32_t X_LP_ECEF = (N_LP + h_LP) * cosf(phi_LP) * cosf(lambda_LP);
    float32_t Y_LP_ECEF = (N_LP + h_LP) * cosf(phi_LP) * sinf(lambda_LP);
    float32_t Z_LP_ECEF = (b2/a2 * N_LP + h_LP) * sinf(phi_LP);

    // ECEF position of rocket
    // prime vertical radius of curvature for rocket
    float32_t N_R = prime_vertical_radius_of_curvature(phi_R);
    float32_t X_R_ECEF = (N_R + h_R) * cosf(phi_R) * cosf(lambda_R);
    float32_t Y_R_ECEF = (N_R + h_R) * cosf(phi_R) * sinf(lambda_R);
    float32_t Z_R_ECEF = (b2/a2 * N_R + h_R) * sinf(phi_R);

    // having found the ECEF position of the laundpad and rocket we can find the ENU coordinates 
    arm_matrix_instance_f32 diff;
    arm_matrix_instance_f32 ENU;
    arm_matrix_instance_f32 transformation_matrix;
    float32_t diff_buffer[3];
    float32_t ENU_buffer[3];
    float32_t transformation_matrix_buffer[9];
    init_mat_zeros(&diff, 3, 1, diff_buffer);
    init_mat_zeros(&ENU, 3, 1, ENU_buffer);
    init_mat_zeros(&transformation_matrix, 3, 3, transformation_matrix_buffer);

    diff.pData[0] = X_R_ECEF - X_LP_ECEF;
    diff.pData[1] = Y_R_ECEF - Y_LP_ECEF;
    diff.pData[2] = Z_R_ECEF - Z_LP_ECEF;

    
    transformation_matrix.pData[0] = -sinf(lambda_LP);
    transformation_matrix.pData[1] =  cosf(lambda_LP);
    transformation_matrix.pData[2] =  0.0f;
    transformation_matrix.pData[3] = -sinf(phi_LP) * cosf(lambda_LP);
    transformation_matrix.pData[4] = -sinf(phi_LP) * sinf(lambda_LP);
    transformation_matrix.pData[5] =  cosf(phi_LP);
    transformation_matrix.pData[6] =  cosf(phi_LP) * cosf(lambda_LP);
    transformation_matrix.pData[7] =  cosf(phi_LP) * sinf(lambda_LP);
    transformation_matrix.pData[8] =  sinf(phi_LP);

    matmul(&transformation_matrix, &diff, &ENU);

    // transfer results to result array
    pResult[0] = ENU.pData[0];
    pResult[1] = ENU.pData[1];
    pResult[2] = ENU.pData[2];
     
    // LEGACY approach. less precise. 
    // // Unpack input
    // float32_t lat = pGNSS_latlonalt[0];
    // float32_t lon = pGNSS_latlonalt[1];
    // float32_t alt = pGNSS_latlonalt[2];

    // // Convert degrees to radians
    // float32_t lat_rad     = lat * (M_PI / 180.0f);
    // float32_t lon_rad     = lon * (M_PI / 180.0f);
    // float32_t lat_ref_rad = lat_ref * (M_PI / 180.0f);
    // float32_t lon_ref_rad = lon_ref * (M_PI / 180.0f);
    
    // // Compute difference from reference
    // float32_t d_lat    = lat_rad - lat_ref_rad;
    // float32_t d_lon    = lon_rad - lon_ref_rad;
    // float32_t d_height = alt - alt_ref;
    
    // // Convert to meters using flat-earth approximation
    // float32_t north = d_lat * R_EARTH_WGS84;
    // float32_t east  = d_lon * R_EARTH_WGS84 * cos(lat_ref_rad);
    
    // // Assign to output array in ENU (East, North, Up) frame
    // pResult[0] = east;
    // pResult[1] = north;
    // pResult[2] = d_height;

    return 0;
}


float pressure_to_alt(float32_t pressure, float32_t pressure_ref, float32_t temp_ref) {
    // Constants for the barometric formula
    const float32_t R = 287.05f; // Specific gas constant for dry air in J/(kg·K)
    const float32_t g = 9.80665f; // Standard gravity in m/s²
    const float32_t L = 0.0065f; // Temperature lapse rate in K/m

    // Calculate altitude using the barometric formula
    float32_t alt = (temp_ref / L) * (1 - powf(pressure / pressure_ref, R * L / g));

    return alt;
}
