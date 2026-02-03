/******************************************************************************
* ISA Flight Software
* @file minor.h
* @brief Minor Cycle (100Hz) Interface
* @details Defines the interface for the 10ms cycle operations
* @author Integration Team, Spacelabs
* @date 2025
* @version 1.0
*****************************************************************************/

#ifndef MINOR_H
#define MINOR_H

#include <stdint.h>
#include <stdbool.h>
#include "math_utils.h"
#include "pefcs_init.h"

// Navigation Accelerometer data processing constants
#define FT_TO_M_S2_CONVERSION_FACTOR 0.3048 /* ft/s² to m/s² */

// Navigation roll rate threshold for magnetometer-based roll estimation
#define NAV_ROLL_RATE_THRESHOLD_RPS 5.0 /* Roll rate threshold: 5.0 rps (rad/s / (2*PI)) */

// Sequencer Parameters structure
typedef struct
{
    uint32_t confirmationCycles;
    double rollRateT1Threshold;
    double rollRateT2Threshold;
    uint32_t t1WindowIn;
    uint32_t t1WindowOut;
    uint32_t t2WindowIn;
    uint32_t t2WindowOut;
    uint32_t t3WindowIn;
    uint32_t t3WindowOut;
    double tProximity;
    uint32_t canardDeployDelay;
    uint32_t canardControlDelay;
    uint32_t fsaDelay;
    uint32_t guidStartDelay;
} SequencerParams_t;

// Sensor Health Status type (must be defined before NavigationState_t uses it)
typedef uint16_t SensorHealthStatus_t;

// Bit positions for sensor health flags
#define SENSOR_ACCEL_X_IS_VALID (1U << 6)
#define SENSOR_ACCEL_Y_IS_VALID (1U << 7)
#define SENSOR_ACCEL_Z_IS_VALID (1U << 8)
#define SENSOR_GYRO_X_IS_VALID (1U << 9)
#define SENSOR_GYRO_Y_IS_VALID (1U << 10)
#define SENSOR_GYRO_Z_IS_VALID (1U << 11)
#define SENSOR_MAG_IS_VALID (1U << 12)

// Navigation Sensor health check
typedef struct
{
    bool accel_healthy; /* True if accelerometer is healthy */
    bool gyro_healthy;  /* True if gyroscope is healthy */
    bool mag_healthy;   /* True if magnetometer is healthy */
} NavigationStatus_t;

// Navigation State structure
typedef struct
{
    EulerAngles_t attitude_e; /* Euler angles (roll, pitch, yaw) */
    Vector3_t angular_rate;
    SensorHealthStatus_t sensorHealth; /* Sensor health status */
    Vector3_t gyro_offset;
    Vector3_t accel_offset;       /* Accelerometer offset values (m/s²) */
    bool mag_3_cycles_confirmed;  /* Magnetometer 3-cycle confirmation flag */
    bool gyro_3_cycles_confirmed; /* Gyroscope 3-cycle confirmation flag */
    double pitch_deg;             /* Pitch angle input from checkout system (degrees) */
    double yaw_deg;               /* Yaw angle input from checkout system (degrees) */
    Vector3_t mag_ref_ned;        /* Magnetic field reference in NED frame (mG) */
    Vector3_t mag_hard_iron;      /* Magnetometer hard iron offsets (mG) */
    double mag_soft_iron[3][3];   /* Magnetometer soft iron calibration matrix */
} NavigationState_t;

// Sequencer State structure
typedef struct
{
    bool isT0Set;
    bool isT1Set;
    bool isT2Set;
    bool isT3Set;
    bool isFsaActivateFlagSent;
    bool isCanardDeployFlagSent;
    bool isCanardControlFlagSent;
    bool isGuidStartFlagSent;
    uint32_t mainClockCycles;           /* Counts minor cycles since OBC reset */
    uint32_t t1SetTime;                 /* When T1 was set (in cycles) */
    uint32_t t2SetTime;                 /* When T2 was set (in cycles) */
    uint32_t t3SetTime;                 /* When T3 was set (in cycles) */
    uint32_t fsaActivateFlagSendTime;   /* When to send the FSA flag (in cycles) */
    uint32_t canardDeployFlagSendTime;  /* When to send canard deploy flag (in cycles) */
    uint32_t canardControlFlagSendTime; /* When to send canard control flag (in cycles) */
    uint32_t guidStartFlagSendTime;     /* When to send guidance start flag (in cycles) */
    uint8_t t1RollRateCount;            /* Roll rate confirmation counter for T1 */
    uint8_t t2RollRateCount;            /* Roll rate confirmation counter for T2 */
    bool isOBCReset;                    /* OBC reset status (activates sequencer) */
} SequencerState_t;

// Sequencer Output structure - flags and phase transitions (publicly accessible)
typedef struct
{
    // Flags to send (publicly accessible via systemState.flags.*)
    bool fsaActivateFlag;       /* FSA activation flag */
    bool canardDeployFlag;      /* Canard deploy flag */
    bool canardControlFlag;     /* Canard control flag */
    bool sendGuidStartFlag;     /* Guidance start flag */
    bool enableProximitySensor; /* Proximity sensor enable flag */
    // Phase transitions (publicly accessible via systemState.flags.*)
    bool setT0; /* Set T0 phase */
    bool setT1; /* Set T1 phase */
    bool setT2; /* Set T2 phase */
    bool setT3; /* Set T3 phase */
} SequencerOutput_t;

// Actuator Commands structure
typedef struct
{
    double ActuatorC3;  /* Canard 3 Actuator Command angle in radians */
    double ActuatorC6;  /* Canard 6 Actuator Command angle in radians */
    double ActuatorC9;  /* Canard 9 Actuator Command angle in radians */
    double ActuatorC12; /* Canard 12 Actuator Command angle in radians */
} ActuatorCommands_t;

// DAP Parameters structure (matches test/dap/types.h DAPParameters_t)
typedef struct
{
    /* Roll controller constants */
    double kp_roll;             /* Proportional gain for roll */
    double ki_roll;             /* Integrator gain for roll */
    double kr_roll;             /* Rate gain for roll */
    double integratorR_min_rad; /* Integrator min limit in radians */
    double integratorR_max_rad; /* Integrator max limit in radians */
    double phi_min_rad;         /* Roll angle min limit in radians */
    double phi_max_rad;         /* Roll angle max limit in radians */
    /* Pitch controller constants */
    double kp_pitch;            /* Proportional gain for pitch */
    double ki_pitch;            /* Integrator gain for pitch */
    double kr_pitch;            /* Rate gain for pitch */
    double K_LPF_Pitch;         /* Low-pass filter gain for pitch */
    double wC_pitch;            /* Cut-off frequency for pitch */
    double pitch_a;             /* Lag filter a constant */
    double pitch_b;             /* Lag filter b constant */
    double theta_min_rad;       /* Pitch angle min limit in radians */
    double theta_max_rad;       /* Pitch angle max limit in radians */
    double integratorP_min_rad; /* Integrator min limit in radians for pitch */
    double integratorP_max_rad; /* Integrator max limit in radians for pitch */
    /* Yaw controller constants */
    double kp_yaw;              /* Proportional gain for yaw */
    double ki_yaw;              /* Integrator gain for yaw */
    double kr_yaw;              /* Rate gain for yaw */
    double K_LPF_Yaw;           /* Low-pass filter gain for yaw */
    double wC_yaw;              /* Cut-off frequency for yaw */
    double yaw_a;               /* Lag filter a constant */
    double yaw_b;               /* Lag filter b constant */
    double psi_min_rad;         /* Yaw angle min limit in radians */
    double psi_max_rad;         /* Yaw angle max limit in radians */
    double integratorY_min_rad; /* Integrator min limit in radians for yaw */
    double integratorY_max_rad; /* Integrator max limit in radians for yaw */

    double actuator_deadband_rad;      /* Actuator deadband in radians */
    double actuator_max_deflection_rad; /* Actuator maximum deflection in radians */

    bool NavigationLastOutFlag;
    double latchedguidanceAccelY;
    double latchedguidanceAccelZ;
} DapParams_t;

// DAP Output structure for canard deflection commands
typedef struct
{
    double delta3_rad;  /* Canard 3 deflection angle in radians */
    double delta6_rad;  /* Canard 6 deflection angle in radians */
    double delta9_rad;  /* Canard 9 deflection angle in radians */
    double delta12_rad; /* Canard 12 deflection angle in radians */
} DAPOutput_t;

// Pitch/Yaw output structure for control commands
typedef struct
{
    double delta1_rad; /* Canard 1 deflection angle in radians */
    double delta2_rad; /* Canard 2 deflection angle in radians */
} PYOutput_t;

// delta t , delta theta - from IMU , had to implement it here

/* Magnetometer calibration constants moved to pefcs_init.h and NavigationState_t */

void minor_cycle(void);

/**
 * @brief Set OBC reset flag
 *
 * This function sets the OBC reset flag for timing calculations.
 *
 * @param isActive true to activate OBC reset, false to deactivate
 * @return Status_t Success or error code
 */

/**
  * @brief Process accelerometer data with health check and flash storage
  *
  * This function processes raw accelerometer data, performs health checks,
  * converts units, and stores data to flash memory.
  * Each axis is processed individually based on its health status.
  *
  * @return void
  */

/**
   * @brief Minor cycle function - Called every 10ms (100Hz)
   *
   * This is the main entry point for the minor cycle processing.
   * It executes navigation, sequencer, and DAP algorithms at 100Hz.
   *
   * @return void
   */

void process_accelerometer_data(void);

/**
  * @brief Placeholder function for flash memory write
  *
  * This function is a placeholder for the hardware team to implement
  * actual flash memory write functionality.
  *
  * @param data Pointer to accelerometer data to store
  * @return void
  */

/**
   * @brief Send accelerometer health data to telemetry
   *
   * This function sends individual accelerometer axis health flags
   * to the telemetry system.
   *
   * @return void
   */
void send_accelerometer_health_to_telemetry(void);

/**
 * @brief Placeholder function for flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality.
 *
 * @param data Pointer to accelerometer data to store
 * @return void
 */
void store_accelerometer_to_flash(const Vector3_t *data);

// Error codes
//raheese debugine vendi chilapol avishyam varum, futurel
typedef enum
{
    SUCCESS = 0,
    ERROR_INVALID_PARAMETER,
    ERROR_NOT_INITIALIZED,
    ERROR_HARDWARE_FAILURE,
    ERROR_TIMEOUT
} Status_t;

Status_t set_obc_reset(bool isActive);

/**
 * @brief Set magnetic field reference in NED frame from checkout system
 *
 * This is a placeholder function for the checkout system to input
 * the magnetic field reference values in NED (North-East-Down) frame.
 *
 * The checkout system should call this function to set the magnetic field
 * reference values. These values will be transformed using the transformation
 * matrix to calculate mv and me for roll estimation.
 *
 * @param mag_ref_ned Pointer to magnetic field reference Vector3 in NED frame
 * @return Status_t Success or error code
 */
Status_t set_mag_ref_ned_from_checkout(const Vector3_t *mag_ref_ned);

/**
 * @brief Load default PEFCS parameters into system state
 *
 * This function initializes all mission-specific parameters from pefcs_init.h
 * Called automatically when OBC reset flag is received
 *
 * @return void
 */
void load_pefcs_defaults(void);

/**
 * @brief Process gyroscope data with health check and flash storage
 *
 * This function processes raw gyroscope data, performs health checks,
 * and stores data to flash memory.
 * Each axis is processed individually based on its health status.
 *
 * @return void
 */
void process_gyroscope_data(void);

/**
 * @brief Placeholder function for gyroscope flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality for gyroscope data.
 *
 * @param data Pointer to gyroscope data to store
 * @return void
 */
void store_gyroscope_to_flash(const Vector3_t *data);

/**
 * @brief Send gyroscope health data to telemetry
 *
 * This function sends individual gyroscope axis health flags
 * to the telemetry system.
 *
 * @return void
 */
void send_gyroscope_health_to_telemetry(void);

/**
 * @brief Process magnetometer data with health check and flash storage
 *
 * This function processes raw magnetometer data, performs health checks,
 * and stores data to flash memory.
 * Uses overall magnetometer health status (not individual axes).
 *
 * @return void
 */
void process_magnetometer_data(void);

/**
 * @brief Placeholder function for magnetometer flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality for magnetometer data.
 *
 * @param data Pointer to magnetometer data to store
 * @return void
 */
void store_magnetometer_to_flash(const Vector3_t *data);

/**
 * @brief Send magnetometer health data to telemetry
 *
 * This function sends magnetometer health flag
 * to the telemetry system.
 *
 * @return void
 */
void send_magnetometer_health_to_telemetry(void);

/**
 * @brief Process incremental velocity data with unit conversion and acceleration calculation
 *
 * This function processes raw incremental velocity data, converts units,
 * calculates acceleration, and stores data to flash memory.
 * No health check is needed - just conversion and calculation.
 *
 * @return void
 */
void process_incremental_velocity_data(void);

/**
 * @brief Placeholder function for incremental velocity flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality for incremental velocity data.
 *
 * @param delta_v_converted Pointer to converted velocity data to store
 * @param acceleration Pointer to calculated acceleration data to store
 * @return void
 */
void store_incremental_velocity_to_flash(const Vector3_t *delta_v_converted, const Vector3_t *acceleration);

/**
 * @brief Process incremental angle data and store to flash memory
 *
 * This function processes raw incremental angle data and stores it to flash memory.
 * No health check or unit conversion is needed - just read and store.
 *
 * @return void
 */
void process_incremental_angle_data(void);

/**
 * @brief Placeholder function for incremental angle flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality for incremental angle data.
 *
 * @param data Pointer to incremental angle data to store
 * @return void
 */
void store_incremental_angle_to_flash(const Vector3_t *data);

/**
 * @brief Send attitude data to telemetry
 *
 * This function sends attitude angles (roll, pitch, yaw) to the telemetry system.
 *
 * @return void
 */
void send_attitude_to_telemetry(void);

/**
 * @brief Send navigation confirmation flags to telemetry
 *
 * This function sends magnetometer and gyroscope 3-cycle confirmation flags
 * to the telemetry system.
 *
 * @return void
 */
void send_navigation_confirmation_flags_to_telemetry(void);

/**
 * @brief Placeholder function for attitude flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality for attitude data.
 *
 * @param data Pointer to attitude data to store
 * @return void
 */
void store_attitude_to_flash(const Vector3_t *data);

#endif /* MINOR_H */