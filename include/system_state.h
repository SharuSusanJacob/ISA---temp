/******************************************************************************
* ISA Flight Software
* @file system_state.h
* @brief System State for Integrated Flight Software
* @details Shared state structure for module integration
* @author Ananthu Dev - Project Engineer/ Integration, Spacelabs
* @date 2025
* @version 1.3
*****************************************************************************/

#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "math_utils.h"
#include "major.h" /* Include for GuidanceState_t definition */
#include "minor.h" /* Include for NavigationState_t, SequencerState_t, DapParams_t definitions */

// IMU data structure - stores raw LSB values from hardware
typedef struct
{
    int16_t accel_x_raw; /* Raw accelerometer X LSB (signed 16-bit) */
    int16_t accel_y_raw; /* Raw accelerometer Y LSB (signed 16-bit) */
    int16_t accel_z_raw; /* Raw accelerometer Z LSB (signed 16-bit) */
    int16_t gyro_x_raw;  /* Raw gyroscope X LSB (signed 16-bit) */
    int16_t gyro_y_raw;  /* Raw gyroscope Y LSB (signed 16-bit) */
    int16_t gyro_z_raw;  /* Raw gyroscope Z LSB (signed 16-bit) */
    int16_t mag_x_raw;   /* Raw magnetometer X LSB (signed 16-bit) */
    int16_t mag_y_raw;   /* Raw magnetometer Y LSB (signed 16-bit) */
    int16_t mag_z_raw;   /* Raw magnetometer Z LSB (signed 16-bit) */
    uint16_t status;

} ImuData_t;

// Accelerometer data processing structure
typedef struct
{
    Vector3_t accel_current;  /* Current accelerometer data (m/s²) */
    Vector3_t accel_previous; /* Previous accelerometer data (m/s²) */
    Vector3_t accel_obc;      /* Converted accelerometer data from LSB (ft/s²) */
    bool accel_x_health_ok;   /* X-axis health flag for telemetry */
    bool accel_y_health_ok;   /* Y-axis health flag for telemetry */
    bool accel_z_health_ok;   /* Z-axis health flag for telemetry */
} AccelerometerData_t;

// Gyroscope data processing structure
typedef struct
{
    Vector3_t gyro_current;  /* Current gyroscope data (rad/s) */
    Vector3_t gyro_previous; /* Previous gyroscope data (rad/s) */
    Vector3_t gyro_obc;      /* Converted gyroscope data from LSB (rad/s) */
    bool gyro_x_health_ok;   /* X-axis health flag for telemetry */
    bool gyro_y_health_ok;   /* Y-axis health flag for telemetry */
    bool gyro_z_health_ok;   /* Z-axis health flag for telemetry */
} GyroscopeData_t;

// Magnetometer data processing structure
typedef struct
{
    Vector3_t mag_current;          /* Current magnetometer data (mG) */
    Vector3_t mag_previous;         /* Previous magnetometer data (mG) */
    Vector3_t mag_obc;              /* Converted magnetometer data from LSB (mG) */
    Vector3_t mag_calibrated;       /* Soft iron offset corrected magnetometer data */
    Vector3_t mag_offset_corrected; /* Hard iron offset corrected magnetometer data */
    Vector3_t mag_ref_transformed;  // NED to NVE converted data
    bool mag_health_ok;             /* Overall magnetometer health flag for telemetry */
} MagnetometerData_t;

// Incremental velocity data processing structure
typedef struct
{
    int32_t delta_v_x_raw;       /* Raw incremental velocity X LSB (signed 32-bit) */
    int32_t delta_v_y_raw;       /* Raw incremental velocity Y LSB (signed 32-bit) */
    int32_t delta_v_z_raw;       /* Raw incremental velocity Z LSB (signed 32-bit) */
    Vector3_t delta_v_obc;       /* Converted incremental velocity data from LSB (ft/s) */
    Vector3_t delta_v_converted; /* Converted incremental velocity data (m/s) */
    // Vector3_t acceleration;       /* Calculated acceleration data (m/s²) */
} IncrementalVelocityData_t;

// Incremental angle data processing structure
typedef struct
{
    int32_t delta_theta_x_raw; /* Raw incremental angle X LSB (signed 32-bit) */
    int32_t delta_theta_y_raw; /* Raw incremental angle Y LSB (signed 32-bit) */
    int32_t delta_theta_z_raw; /* Raw incremental angle Z LSB (signed 32-bit) */
    Vector3_t delta_theta_obc; /* Converted incremental angle data from LSB (rad) */
} IncrementalAngleData_t;

// Attitude data for telemetry structure
typedef struct
{
    Vector3_t attitude_rad; /* Attitude angles (roll, pitch, yaw) in radians */
} AttitudeTelemetryData_t;

// Global system state structure
typedef struct
{
    // Module states
    GuidanceState_t guidanceState;
    NavigationState_t navigationState;
    SequencerState_t sequencerState;
    SequencerParams_t sequencerParams;
    DapParams_t dapParams;
    // Accelerometer data processing
    AccelerometerData_t accelerometerData;
    GyroscopeData_t gyroscopeData;
    MagnetometerData_t magnetometerData;
    IncrementalVelocityData_t incrementalVelocityData;
    IncrementalAngleData_t incrementalAngleData;
    // Attitude telemetry data
    AttitudeTelemetryData_t attitudeTelemetryData;
    // Shared flags
    struct
    {
        bool isT0Set;
        bool isT1Set;
        bool isT2Set;
        bool isT3Set;
        bool fsaActivateFlag;
        bool canardDeployFlag;
        bool canardControlFlag;
        bool guidStartFlag;
        bool proximitySensorFlag;
    } flags;

    // ECEF frames from GNSS
    Vector3_t position_ecef;
    Vector3_t velocity_ecef;
    Vector3_t positionLocal;
    Vector3_t velocityLocal;

    // Sensor inputs
    double rollRateFp;
    Vector3_t angularRates;
    ImuData_t imuData;
    double rollIntegrator;    /* DAP roll integrator value */
    double previousPitchRate; /* DAP previous pitch rate for derivative calculation */
    double previousYawRate;   /* DAP previous yaw rate for derivative calculation */

    // Actuator outputs
    ActuatorCommands_t actuatorCommands;

    // Timing information
    uint32_t minorCycleCount;
    uint32_t majorCycleCount;
    uint32_t timeFromObcReset;

    // Test mode flag
    bool testMode; /* True if running in test harness (bypasses hardware processing) */
} SystemState_t;

// Global system state declaration
extern SystemState_t systemState;

#endif /* SYSTEM_STATE_H */