/******************************************************************************
* ISA Flight Software
* @file navigation.h
* @brief Navigation System Core Module
* @details Implements state estimation and coordinate transformations for guidance
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 1.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "vector3.h"
// #include "coordinate_transforms.h"

/**
 * Error codes for navigation module
 * Following MISRA-C enumeration pattern
 */
typedef enum {
    NAV_SUCCESS = 0U,
    NAV_ERROR_INVALID_PARAM = 1U,
    NAV_ERROR_INVALID_STATE = 2U,
    NAV_ERROR_MATH_ERROR = 3U,
    NAV_ERROR_SENSOR_FAILURE = 4U
} NavigationError_t;

/**
 * Sensor data quality flags
 */
typedef enum {
    NAV_SENSOR_QUALITY_EXCELLENT = 0U,
    NAV_SENSOR_QUALITY_GOOD = 1U,
    NAV_SENSOR_QUALITY_FAIR = 2U,
    NAV_SENSOR_QUALITY_POOR = 3U,
    NAV_SENSOR_QUALITY_INVALID = 4U
} NavigationSensorQuality_t;

/**
 * Navigation filter modes
 */
typedef enum {
    NAV_MODE_INIT = 0U,
    NAV_MODE_STANDBY = 1U,
    NAV_MODE_PREFLIGHT = 2U,
    NAV_MODE_FLIGHT = 3U,
    NAV_MODE_TERMINAL = 4U
} NavigationMode_t;

/**
 * Attitude representation quaternion
 */
typedef struct {
    float q0;    /* Scalar component */
    float q1;    /* Vector component x */
    float q2;    /* Vector component y */
    float q3;    /* Vector component z */
} NavQuaternion_t;

/**
 * Euler angle representation (roll, pitch, yaw)
 */
typedef struct {
    float roll_rad;     /* Roll angle in radians */
    float pitch_rad;    /* Pitch angle in radians */
    float yaw_rad;      /* Yaw angle in radians */
} NavEulerAngles_t;

/**
 * GPS sensor data
 */
// typedef struct {
//     float lat_deg;      /* Latitude in degrees */
//     float lon_deg;      /* Longitude in degrees */
//     float alt_m;        /* Altitude in meters */
//     float vel_n_mps;    /* Velocity north in m/s */
//     float vel_e_mps;    /* Velocity east in m/s */
//     float vel_d_mps;    /* Velocity down in m/s */
//     float gps_week;     /* GPS week number */
//     float tow_sec;      /* Time of week in seconds */
//     float hdop;         /* Horizontal dilution of precision */
//     float vdop;         /* Vertical dilution of precision */
//     uint8_t fix_type;   /* GPS fix type: 0=No fix, 1=2D, 2=3D */
//     uint8_t num_sats;   /* Number of satellites used */
//     NavigationSensorQuality_t quality;  /* GPS data quality */
//     bool valid;         /* True if data is valid */
// } NavGpsData_t;

/**
 * IMU sensor data
 */
typedef struct {
    Vector3_t accel;          /* Accelerometer readings in m/s^2, body frame */
    Vector3_t gyro;           /* Gyroscope readings in rad/s, body frame */
    Vector3_t mag;            /* Magnetometer readings in nT, body frame */
    // float temp_c;             /* IMU temperature in Celsius */
    // NavigationSensorQuality_t accel_quality;  /* Accelerometer data quality */
    // NavigationSensorQuality_t gyro_quality;   /* Gyroscope data quality */
    // NavigationSensorQuality_t mag_quality;    /* Magnetometer data quality */
    // bool accel_valid;         /* True if accelerometer data is valid */
    // bool gyro_valid;          /* True if gyroscope data is valid */
    // bool mag_valid;           /* True if magnetometer data is valid */
} NavImuData_t;

/**
 * Navigation system state
 */
typedef struct {
    /* Core state vectors */
    // Vector3_t position_eci;    /* Current position in ECI frame (m) */
    // Vector3_t velocity_eci;    /* Current velocity in ECI frame (m/s) */
    // Vector3_t accel_eci;       /* Current acceleration in ECI frame (m/s^2) */
    
    /* Attitude representations */
    // NavQuaternion_t attitude_q;  /* Attitude quaternion (body to ECI) */
    NavEulerAngles_t attitude_e; /* Euler angles (roll, pitch, yaw) */
    
    /* Angular rates and accelerations */
    Vector3_t angular_rate;     /* Angular rate in body frame (rad/s) */
    // Vector3_t angular_accel;    /* Angular acceleration in body frame (rad/s^2) */
    
    /* Raw sensor data */
    // NavGpsData_t gps_data;     /* Latest GPS data */
    // NavImuData_t imu_data;     /* Latest IMU data */
    
    /* Navigation solution status */
    // NavigationMode_t mode;     /* Current navigation mode */
    // GeodeticPos_t geodetic;    /* Current geodetic position (lat, lon, alt) */
    // uint32_t time_ms;          /* Navigation system time in milliseconds */
    // float dt_s;                /* Time delta since last update in seconds */
    
    /* Filter status and health */
    // bool solution_valid;       /* True if navigation solution is valid */
    // uint8_t solution_quality;  /* Solution quality 0-100% */
    
} NavigationState_t;

// /**
//  * @brief Initialize navigation module
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// NavigationError_t navigation_init(NavigationState_t* state);

// /**
//  * @brief Process new IMU data
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param imu_data Pointer to IMU data structure (must not be NULL)
//  * @param dt_s Time step in seconds since last IMU update
//  * @return NavigationError_t Success or error code
//  */
// NavigationError_t navigation_process_imu(NavigationState_t* state, const NavImuData_t* imu_data, float dt_s);

// /**
//  * @brief Process new GPS data
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param gps_data Pointer to GPS data structure (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// NavigationError_t navigation_process_gps(NavigationState_t* state, const NavGpsData_t* gps_data);

// /**
//  * @brief Main navigation update function - run at regular interval
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param dt_s Time step in seconds
//  * @return NavigationError_t Success or error code
//  * 
//  * Main state propagation and estimation algorithm. Should be called at fixed rate.
//  */
NavigationError_t navigation_update(NavigationState_t* state, float dt_s);

/**
 * @brief Get current position in ECI frame
 * @param state Pointer to navigation state structure (must not be NULL)
 * @param position Pointer to output position vector (must not be NULL)
 * @return NavigationError_t Success or error code
 */
// NavigationError_t navigation_get_position_eci(const NavigationState_t* state, Vector3_t* position);

// /**
//  * @brief Get current velocity in ECI frame
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param velocity Pointer to output velocity vector (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// NavigationError_t navigation_get_velocity_eci(const NavigationState_t* state, Vector3_t* velocity);

// /**
//  * @brief Get current acceleration in ECI frame
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param accel Pointer to output acceleration vector (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// NavigationError_t navigation_get_accel_eci(const NavigationState_t* state, Vector3_t* accel);

// /**
//  * @brief Get current geodetic position (lat, lon, alt)
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param geodetic Pointer to output geodetic position (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// NavigationError_t navigation_get_geodetic(const NavigationState_t* state, GeodeticPos_t* geodetic);

// /**
//  * @brief Get attitude quaternion (body to ECI frame)
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param q Pointer to output quaternion (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// NavigationError_t navigation_get_attitude_quaternion(const NavigationState_t* state, NavQuaternion_t* q);

// /**
//  * @brief Get attitude as Euler angles (roll, pitch, yaw)
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param euler Pointer to output Euler angles (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
NavigationError_t navigation_get_attitude_euler(const NavigationState_t* state, NavEulerAngles_t* euler);

/**
 * @brief Get angular rates in body frame
 * @param state Pointer to navigation state structure (must not be NULL)
 * @param rates Pointer to output angular rates (must not be NULL)
 * @return NavigationError_t Success or error code
 */
NavigationError_t navigation_get_angular_rates(const NavigationState_t* state, Vector3_t* rates);

/**
 * @brief Set simulated position (for testing without hardware)
 * @param state Pointer to navigation state structure (must not be NULL)
 * @param position Pointer to position vector in ECI frame (must not be NULL)
 * @return NavigationError_t Success or error code
 */
// NavigationError_t navigation_set_position_eci(NavigationState_t* state, const Vector3_t* position);

// /**
//  * @brief Set simulated velocity (for testing without hardware)
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param velocity Pointer to velocity vector in ECI frame (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// NavigationError_t navigation_set_velocity_eci(NavigationState_t* state, const Vector3_t* velocity);

// /**
//  * @brief Set simulated attitude (for testing without hardware)
//  * @param state Pointer to navigation state structure (must not be NULL)
//  * @param q Pointer to attitude quaternion (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
// NavigationError_t navigation_set_attitude(NavigationState_t* state, const NavQuaternion_t* q);

// /**
//  * @brief Quaternion multiplication: result = q1 * q2
//  * @param result Output quaternion (must not be NULL)
//  * @param q1 First quaternion (must not be NULL)
//  * @param q2 Second quaternion (must not be NULL)
//  * @return NavigationError_t Success or error code
//  */
NavigationError_t navigation_quaternion_multiply(
    NavQuaternion_t* result, 
    const NavQuaternion_t* q1, 
    const NavQuaternion_t* q2);

/**
 * @brief Convert quaternion to Euler angles
 * @param euler Output Euler angles (must not be NULL)
 * @param q Input quaternion (must not be NULL)
 * @return NavigationError_t Success or error code
 */
NavigationError_t navigation_quaternion_to_euler(
    NavEulerAngles_t* euler, 
    const NavQuaternion_t* q);

/**
 * @brief Convert Euler angles to quaternion
 * @param q Output quaternion (must not be NULL)
 * @param euler Input Euler angles (must not be NULL)
 * @return NavigationError_t Success or error code
 */
NavigationError_t navigation_euler_to_quaternion(
    NavQuaternion_t* q, 
    const NavEulerAngles_t* euler);

/**
 * @brief Reset navigation state to initial values
 * @param state Pointer to navigation state structure (must not be NULL)
 * @return NavigationError_t Success or error code
 */
NavigationError_t navigation_reset(NavigationState_t* state);

#endif /* NAVIGATION_H */