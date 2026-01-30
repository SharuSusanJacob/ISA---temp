/******************************************************************************
* ISA Flight Software
* @file navigation.c
* @brief ISA Navigation core logic implementation
* @details Implements state estimation and coordinate transformations for guidance
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 1.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#include <math.h>
#include <float.h>
#include <stdlib.h>
#include "navigation.h"

/* Constants for navigation algorithms */
#define NAV_PI                  (3.14159265358979323846f)
#define NAV_TWO_PI              (2.0f * NAV_PI)
#define NAV_HALF_PI             (0.5f * NAV_PI)
#define NAV_DEG_TO_RAD          (NAV_PI / 180.0f)
#define NAV_RAD_TO_DEG          (180.0f / NAV_PI)
// #define NAV_EARTH_ROTATION_RATE (7.2921151467e-5f)    /* Earth rotation rate (rad/s) */
// #define NAV_GRAVITY_NOMINAL     (9.80665f)            /* Nominal gravity (m/s^2) */
// #define NAV_DEFAULT_QUALITY     (50U)                 /* Default solution quality (%) */
// #define NAV_MIN_UPDATE_RATE     (0.001f)              /* Minimum update rate (s) */
// #define NAV_MAX_UPDATE_RATE     (1.0f)                /* Maximum update rate (s) */

/* Limits for quaternion normalization */
// #define NAV_QUAT_NORMALIZATION_THRESHOLD (0.9995f)  /* Threshold for quaternion normalization */
// #define NAV_EPSILON                      (1.0e-6f)  /* Small value to prevent division by zero */

// /* Create identity quaternion constant */
// static const NavQuaternion_t NAV_IDENTITY_QUATERNION = { 1.0f, 0.0f, 0.0f, 0.0f };

// /**
//  * @brief Normalize a quaternion to unit length
//  * 
//  * @param q Pointer to quaternion to normalize (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// static NavigationError_t normalize_quaternion(NavQuaternion_t* q) {
//     float magnitude;
    
//     /* Check for NULL pointer */
//     if (q == NULL) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Calculate magnitude */
//     magnitude = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    
//     /* Check for near-zero quaternion */
//     if (magnitude < NAV_EPSILON) {
//         /* Return identity quaternion for safety */
//         q->q0 = 1.0f;
//         q->q1 = 0.0f;
//         q->q2 = 0.0f;
//         q->q3 = 0.0f;
//         return NAV_ERROR_MATH_ERROR;
//     }
    
//     /* Normalize the quaternion */
//     q->q0 /= magnitude;
//     q->q1 /= magnitude;
//     q->q2 /= magnitude;
//     q->q3 /= magnitude;
    
//     return NAV_SUCCESS;
// }

/**
//  * @brief Update geodetic position from ECI position
//  * 
//  * @param state Pointer to navigation state (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// static NavigationError_t update_geodetic_from_eci(NavigationState_t* state) {
//     /* Check for NULL pointer */
//     if (state == NULL) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Convert ECI position to geodetic coordinates */
//     return (NavigationError_t)eci_to_geodetic(&(state->geodetic), &(state->position_eci));
// }

/* Implementation of public functions */

// NavigationError_t navigation_init(NavigationState_t* state) {
//     /* Check for NULL pointer */
//     if (state == NULL) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Initialize state with zeros */
//     (void)memset(state, 0, sizeof(NavigationState_t));
    
//     /* Set default values */
//     state->attitude_q.q0 = 1.0f;            /* Identity quaternion */
//     state->attitude_q.q1 = 0.0f;
//     state->attitude_q.q2 = 0.0f;
//     state->attitude_q.q3 = 0.0f;
    
//     state->attitude_e.roll_rad = 0.0f;      /* Zero Euler angles */
//     state->attitude_e.pitch_rad = 0.0f;
//     state->attitude_e.yaw_rad = 0.0f;
    
//     state->mode = NAV_MODE_INIT;            /* Initial mode */
//     state->solution_quality = NAV_DEFAULT_QUALITY;
    
//     /* Set default geodetic position (coordinates in middle of ocean for safety) */
//     state->geodetic.lat_deg = 0.0f;         /* Equator */
//     state->geodetic.lon_deg = 0.0f;         /* Prime meridian */
//     state->geodetic.alt_m = 0.0f;           /* Sea level */
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_process_imu(NavigationState_t* state, const NavImuData_t* imu_data, float dt_s) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (imu_data == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Check time step validity */
//     if ((dt_s < NAV_MIN_UPDATE_RATE) || (dt_s > NAV_MAX_UPDATE_RATE)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Store IMU data */
//     state->imu_data = *imu_data;
    
//     /* Copy angular rates to state */
//     state->angular_rate = imu_data->gyro;
    
//     /* For minimal implementation, just store acceleration in body frame */
//     /* In a full implementation, we would transform from body to ECI frame */
//     /* For now, we'll simulate this by just copying the data */
//     state->accel_eci.x = imu_data->accel.x;
//     state->accel_eci.y = imu_data->accel.y;
//     state->accel_eci.z = imu_data->accel.z;
    
//     /* Update attitude by integrating gyro data (simple quaternion integration) */
//     if (imu_data->gyro_valid) {
//         /* Get angular rates */
//         float p = imu_data->gyro.x;  /* Roll rate (rad/s) */
//         float q = imu_data->gyro.y;  /* Pitch rate (rad/s) */
//         float r = imu_data->gyro.z;  /* Yaw rate (rad/s) */
        
//         /* Create rotation quaternion for this time step */
//         float half_dt = 0.5f * dt_s;
//         float half_angle_x = p * half_dt;
//         float half_angle_y = q * half_dt;
//         float half_angle_z = r * half_dt;
        
//         /* Small angle approximation quaternion */
//         NavQuaternion_t delta_q;
//         delta_q.q0 = 1.0f;
//         delta_q.q1 = half_angle_x;
//         delta_q.q2 = half_angle_y;
//         delta_q.q3 = half_angle_z;
        
//         /* Normalize delta quaternion */
//         (void)normalize_quaternion(&delta_q);
        
//         /* Apply rotation to current attitude quaternion */
//         NavigationError_t error = navigation_quaternion_multiply(
//             &(state->attitude_q), 
//             &(state->attitude_q), 
//             &delta_q);
            
//         if (error != NAV_SUCCESS) {
//             return error;
//         }
        
//         /* Normalize result */
//         error = normalize_quaternion(&(state->attitude_q));
//         if (error != NAV_SUCCESS) {
//             return error;
//         }
        
//         /* Update Euler angles from quaternion */
//         error = navigation_quaternion_to_euler(&(state->attitude_e), &(state->attitude_q));
//         if (error != NAV_SUCCESS) {
//             return error;
//         }
//     }
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_process_gps(NavigationState_t* state, const NavGpsData_t* gps_data) {
//     Vector3_t eci_position;
//     GeodeticPos_t geodetic;
    
//     /* Check for NULL pointers */
//     if ((state == NULL) || (gps_data == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Store GPS data */
//     state->gps_data = *gps_data;
    
//     /* If GPS data is valid, update navigation state */
//     if (gps_data->valid && (gps_data->fix_type > 0U)) {
//         /* Create geodetic position */
//         geodetic.lat_deg = gps_data->lat_deg;
//         geodetic.lon_deg = gps_data->lon_deg;
//         geodetic.alt_m = gps_data->alt_m;
        
//         /* Convert GPS geodetic position to ECI */
//         if (geodetic_to_eci(&eci_position, &geodetic) != COORD_TRANSFORM_SUCCESS) {
//             return NAV_ERROR_MATH_ERROR;
//         }
        
//         /* Update state with GPS position */
//         state->position_eci = eci_position;
//         state->geodetic = geodetic;
        
//         /* Convert NED velocity to ECI (simple approximation for minimal implementation) */
//         /* In a full implementation, we would apply proper frame transformations */
//         state->velocity_eci.x = gps_data->vel_e_mps;  /* East -> X (simplified) */
//         state->velocity_eci.y = gps_data->vel_n_mps;  /* North -> Y (simplified) */
//         state->velocity_eci.z = -gps_data->vel_d_mps; /* Down -> -Z (simplified) */
        
//         /* Set solution validity based on GPS quality */
//         if (gps_data->quality <= NAV_SENSOR_QUALITY_FAIR) {
//             state->solution_valid = true;
            
//             /* Update solution quality based on GPS data */
//             state->solution_quality = (uint8_t)(100U - ((uint8_t)gps_data->quality * 20U));
//         }
        
//         /* Update mode */
//         if (state->mode == NAV_MODE_INIT) {
//             state->mode = NAV_MODE_STANDBY;
//         }
//     }
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_update(NavigationState_t* state, float dt_s) {
//     /* Check for NULL pointer */
//     if (state == NULL) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Check time step validity */
//     if ((dt_s < NAV_MIN_UPDATE_RATE) || (dt_s > NAV_MAX_UPDATE_RATE)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Store time step */
//     state->dt_s = dt_s;
    
//     /* Update system time */
//     state->time_ms += (uint32_t)(dt_s * 1000.0f);
    
//     /* In flight mode, propagate position based on velocity */
//     if (state->mode == NAV_MODE_FLIGHT || state->mode == NAV_MODE_TERMINAL) {
//         /* Propagate position: p = p + v*dt + 0.5*a*dt^2 */
//         float dt_squared_half = 0.5f * dt_s * dt_s;
        
//         state->position_eci.x += state->velocity_eci.x * dt_s + state->accel_eci.x * dt_squared_half;
//         state->position_eci.y += state->velocity_eci.y * dt_s + state->accel_eci.y * dt_squared_half;
//         state->position_eci.z += state->velocity_eci.z * dt_s + state->accel_eci.z * dt_squared_half;
        
//         /* Propagate velocity: v = v + a*dt */
//         state->velocity_eci.x += state->accel_eci.x * dt_s;
//         state->velocity_eci.y += state->accel_eci.y * dt_s;
//         state->velocity_eci.z += state->accel_eci.z * dt_s;
        
//         /* Update geodetic position from ECI */
//         (void)update_geodetic_from_eci(state);
//     }
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_get_position_eci(const NavigationState_t* state, Vector3_t* position) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (position == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Copy position */
//     *position = state->position_eci;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_get_velocity_eci(const NavigationState_t* state, Vector3_t* velocity) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (velocity == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Copy velocity */
//     *velocity = state->velocity_eci;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_get_accel_eci(const NavigationState_t* state, Vector3_t* accel) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (accel == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Copy acceleration */
//     *accel = state->accel_eci;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_get_geodetic(const NavigationState_t* state, GeodeticPos_t* geodetic) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (geodetic == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Copy geodetic position */
//     *geodetic = state->geodetic;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_get_attitude_quaternion(const NavigationState_t* state, NavQuaternion_t* q) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (q == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Copy attitude quaternion */
//     *q = state->attitude_q;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_get_attitude_euler(const NavigationState_t* state, NavEulerAngles_t* euler) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (euler == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Copy Euler angles */
//     *euler = state->attitude_e;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_get_angular_rates(const NavigationState_t* state, Vector3_t* rates) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (rates == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Copy angular rates */
//     *rates = state->angular_rate;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_set_position_eci(NavigationState_t* state, const Vector3_t* position) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (position == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Set position */
//     state->position_eci = *position;
    
//     /* Update geodetic position from ECI */
//     (void)update_geodetic_from_eci(state);
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_set_velocity_eci(NavigationState_t* state, const Vector3_t* velocity) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (velocity == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Set velocity */
//     state->velocity_eci = *velocity;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_set_attitude(NavigationState_t* state, const NavQuaternion_t* q) {
//     /* Check for NULL pointers */
//     if ((state == NULL) || (q == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Set quaternion */
//     state->attitude_q = *q;
    
//     /* Normalize quaternion */
//     NavigationError_t error = normalize_quaternion(&(state->attitude_q));
//     if (error != NAV_SUCCESS) {
//         return error;
//     }
    
//     /* Update Euler angles */
//     error = navigation_quaternion_to_euler(&(state->attitude_e), &(state->attitude_q));
    
//     /* Return result */
//     return error;
// }

// NavigationError_t navigation_quaternion_multiply(
//     NavQuaternion_t* result, 
//     const NavQuaternion_t* q1, 
//     const NavQuaternion_t* q2) {
    
//     /* Check for NULL pointers */
//     if ((result == NULL) || (q1 == NULL) || (q2 == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Quaternion multiplication */
//     NavQuaternion_t temp;
//     temp.q0 = q1->q0 * q2->q0 - q1->q1 * q2->q1 - q1->q2 * q2->q2 - q1->q3 * q2->q3;
//     temp.q1 = q1->q0 * q2->q1 + q1->q1 * q2->q0 + q1->q2 * q2->q3 - q1->q3 * q2->q2;
//     temp.q2 = q1->q0 * q2->q2 - q1->q1 * q2->q3 + q1->q2 * q2->q0 + q1->q3 * q2->q1;
//     temp.q3 = q1->q0 * q2->q3 + q1->q1 * q2->q2 - q1->q2 * q2->q1 + q1->q3 * q2->q0;
    
//     /* Copy result */
//     *result = temp;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_quaternion_to_euler(
//     NavEulerAngles_t* euler, 
//     const NavQuaternion_t* q) {
    
//     /* Check for NULL pointers */
//     if ((euler == NULL) || (q == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Calculate Euler angles from quaternion */
//     /* Using aerospace sequence (3-2-1 rotation, Yaw-Pitch-Roll) */
//     float q0_squared = q->q0 * q->q0;
//     float q1_squared = q->q1 * q->q1;
//     float q2_squared = q->q2 * q->q2;
//     float q3_squared = q->q3 * q->q3;
    
//     /* Roll (rotation about X) */
//     euler->roll_rad = atan2f(2.0f * (q->q2 * q->q3 + q->q0 * q->q1),
//                             q0_squared - q1_squared - q2_squared + q3_squared);
    
//     /* Pitch (rotation about Y) */
//     float sin_pitch = -2.0f * (q->q1 * q->q3 - q->q0 * q->q2);
    
//     /* Limit sin_pitch to valid range */
//     if (sin_pitch > 1.0f) {
//         sin_pitch = 1.0f;
//     } else if (sin_pitch < -1.0f) {
//         sin_pitch = -1.0f;
//     }
    
//     euler->pitch_rad = asinf(sin_pitch);
    
//     /* Yaw (rotation about Z) */
//     euler->yaw_rad = atan2f(2.0f * (q->q1 * q->q2 + q->q0 * q->q3),
//                            q0_squared + q1_squared - q2_squared - q3_squared);
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_euler_to_quaternion(
//     NavQuaternion_t* q, 
//     const NavEulerAngles_t* euler) {
    
//     /* Check for NULL pointers */
//     if ((q == NULL) || (euler == NULL)) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Calculate half angles */
//     float half_roll = euler->roll_rad * 0.5f;
//     float half_pitch = euler->pitch_rad * 0.5f;
//     float half_yaw = euler->yaw_rad * 0.5f;
    
//     /* Compute sin and cos values */
//     float sin_roll = sinf(half_roll);
//     float cos_roll = cosf(half_roll);
//     float sin_pitch = sinf(half_pitch);
//     float cos_pitch = cosf(half_pitch);
//     float sin_yaw = sinf(half_yaw);
//     float cos_yaw = cosf(half_yaw);
    
//     /* Compute quaternion components using aerospace sequence (3-2-1, Yaw-Pitch-Roll) */
//     q->q0 = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
//     q->q1 = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
//     q->q2 = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
//     q->q3 = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
    
//     /* Normalize quaternion */
//     (void)normalize_quaternion(q);
    
//     /* Return success */
//     return NAV_SUCCESS;
// }

// NavigationError_t navigation_reset(NavigationState_t* state) {
//     /* Check for NULL pointer */
//     if (state == NULL) {
//         return NAV_ERROR_INVALID_PARAM;
//     }
    
//     /* Store system time temporarily */
//     uint32_t time_ms = state->time_ms;
    
//     /* Reset state with zeros */
//     (void)memset(state, 0, sizeof(NavigationState_t));
    
//     /* Restore system time */
//     state->time_ms = time_ms;
    
//     /* Set default values */
//     state->attitude_q.q0 = 1.0f;           /* Identity quaternion */
//     state->mode = NAV_MODE_INIT;           /* Initial mode */
//     state->solution_quality = NAV_DEFAULT_QUALITY;
    
//     /* Return success */
//     return NAV_SUCCESS;
// }