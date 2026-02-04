/******************************************************************************
* ISA Flight Software
* @file minor.c
* @brief Minor Cycle (100Hz) Implementation for Integrated Flight Software
* @details Implements 10ms cycle tasks for navigation, sequencer and DAP
* @author Integration Team, Spacelabs
* @date 2025
* @version 1.0.4
*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "minor.h" /* Include before system_state.h so types are defined */
#include "math_utils.h"
#include "type_convert.h" /* Include for LSB to double conversion */
#include "system_state.h"
#include "major.h"      /* Include for guidance setter functions */
#include "pefcs_init.h" /* Include for PEFCS initialization constants */

// Global system state
extern SystemState_t systemState;

/* ===== ATTITUDE ESTIMATION TRACKING VARIABLES ===== */
/* Global variables for attitude estimation tracking */
/* Magnetometer attitude estimation flags (3 consecutive cycles check) */
static int magcount = 0;
static bool MAG_3_CYCLES_CONFIRMED = false;
static bool magEstOkFlag = false;

/* Gyroscope attitude integration flags (3 consecutive cycles check) */
static int gyrocount = 0;
static bool gyro_rate_condition_met = false;
static bool gyro_3_cycles_confirmed = false;
static bool gyroattitude = false;

/* Roll rate in rps (revolutions per second) */
static double rate = 0.0;

/* ===== SENSOR HEALTH CHECK ===== */
/**
 * @brief Check sensor health in major cycle
 *
 * Performs health check on all IMU sensors
 * 0 = OK, 1 = Failed
 */
static void check_sensor_health(void)
{
    /* Read status word from IMU hardware (replace with actual hardware interface) */
    SensorHealthStatus_t sensor_status = 0U;

    /* Store raw status word for telemetry */
    systemState.navigationState.sensorHealth = sensor_status;

    /* Check individual sensor channels */
    bool accel_x_ok = ((sensor_status & SENSOR_ACCEL_X_IS_VALID) == 0U);
    bool accel_y_ok = ((sensor_status & SENSOR_ACCEL_Y_IS_VALID) == 0U);
    bool accel_z_ok = ((sensor_status & SENSOR_ACCEL_Z_IS_VALID) == 0U);
    bool gyro_x_ok = ((sensor_status & SENSOR_GYRO_X_IS_VALID) == 0U);
    bool gyro_y_ok = ((sensor_status & SENSOR_GYRO_Y_IS_VALID) == 0U);
    bool gyro_z_ok = ((sensor_status & SENSOR_GYRO_Z_IS_VALID) == 0U);
    bool mag_ok = ((sensor_status & SENSOR_MAG_IS_VALID) == 0U);
}

/* ===== ACCELEROMETER DATA PROCESSING ===== */

/**
 * @brief Process accelerometer data with health check and flash storage
 *
 * This function processes raw accelerometer data, performs health checks,
 * converts units, and stores data to flash memory.
 * Each axis is processed individually based on its health status.
 *
 * @return void
 */

void process_accelerometer_data(void)
{
    /* Check individual accelerometer axis health */
    bool accel_x_ok = ((systemState.navigationState.sensorHealth & SENSOR_ACCEL_X_IS_VALID) == 0U);
    bool accel_y_ok = ((systemState.navigationState.sensorHealth & SENSOR_ACCEL_Y_IS_VALID) == 0U);
    bool accel_z_ok = ((systemState.navigationState.sensorHealth & SENSOR_ACCEL_Z_IS_VALID) == 0U);

    /* Process each axis individually based on health status */

    /* X-axis processing */
    if (accel_x_ok)
    {
        /* X-axis is healthy - convert LSB to double (ft/s²) */
        systemState.accelerometerData.accel_obc.x = (double)systemState.imuData.accel_x_raw * CONV_ACCEL_LSB_TO_FT_S2;
        /* Convert ft/s² to m/s² and subtract accelerometer offset */
        systemState.accelerometerData.accel_current.x = (systemState.accelerometerData.accel_obc.x * FT_TO_M_S2_CONVERSION_FACTOR) - systemState.navigationState.accel_offset.x;
        /* Update previous value for future use */
        systemState.accelerometerData.accel_previous.x = systemState.accelerometerData.accel_current.x;
    }
    else
    {
        /* X-axis is unhealthy - use previous data */
        systemState.accelerometerData.accel_current.x = systemState.accelerometerData.accel_previous.x;
    }

    /* Y-axis processing */
    if (accel_y_ok)
    {
        /* Y-axis is healthy - convert LSB to double (ft/s²) */
        systemState.accelerometerData.accel_obc.y = (double)systemState.imuData.accel_y_raw * CONV_ACCEL_LSB_TO_FT_S2;
        /* Convert ft/s² to m/s² and subtract accelerometer offset */
        systemState.accelerometerData.accel_current.y = (systemState.accelerometerData.accel_obc.y * FT_TO_M_S2_CONVERSION_FACTOR) - systemState.navigationState.accel_offset.y;
        /* Update previous value for future use */
        systemState.accelerometerData.accel_previous.y = systemState.accelerometerData.accel_current.y;
    }
    else
    {
        /* Y-axis is unhealthy - use previous converted data */
        systemState.accelerometerData.accel_current.y = systemState.accelerometerData.accel_previous.y;
    }

    /* Z-axis processing */
    if (accel_z_ok)
    {
        /* Z-axis is healthy - convert LSB to double (ft/s²) */
        systemState.accelerometerData.accel_obc.z = (double)systemState.imuData.accel_z_raw * CONV_ACCEL_LSB_TO_FT_S2;
        /* Convert ft/s² to m/s² and subtract accelerometer offset */
        systemState.accelerometerData.accel_current.z = (systemState.accelerometerData.accel_obc.z * FT_TO_M_S2_CONVERSION_FACTOR) - systemState.navigationState.accel_offset.z;
        /* Update previous value for future use */
        systemState.accelerometerData.accel_previous.z = systemState.accelerometerData.accel_current.z;
    }
    else
    {
        /* Z-axis is unhealthy - use previous converted data */
        systemState.accelerometerData.accel_current.z = systemState.accelerometerData.accel_previous.z;
    }

    /* Store processed data to flash memory */
    store_accelerometer_to_flash(&systemState.accelerometerData.accel_current);
    /* Processed accelerometer data is stored in systemState.accelerometerData.accel_current */
    /* DAP will read Ay and Az from this processed data (see execute_dap function) */

    /* Set individual health flags for telemetry */
    systemState.accelerometerData.accel_x_health_ok = accel_x_ok;
    systemState.accelerometerData.accel_y_health_ok = accel_y_ok;
    systemState.accelerometerData.accel_z_health_ok = accel_z_ok;

    /* Send health data to telemetry */
    send_accelerometer_health_to_telemetry();
}

/* Send health data to telemetry */
void send_accelerometer_health_to_telemetry(void)
{
    /* Placeholder implementation for telemetry system */
    /* Hardware team should implement actual telemetry transmission here */

    /* Extract individual health flags */
    // ask ananthu or suhaib
    bool x_axis_healthy = systemState.accelerometerData.accel_x_health_ok;
    bool y_axis_healthy = systemState.accelerometerData.accel_y_health_ok;
    bool z_axis_healthy = systemState.accelerometerData.accel_z_health_ok;

    /* TODO: Hardware team - implement actual telemetry transmission */
    /* Example interface:
     * telemetry_send_accel_health(x_axis_healthy, y_axis_healthy, z_axis_healthy);
     * telemetry_send_accel_data(systemState.accelerometerData.accel_current.x,
     *                          systemState.accelerometerData.accel_current.y,
     *                          systemState.accelerometerData.accel_current.z);
     */

    /* For now, just store in system state for other modules to access */
    /* Telemetry system can read these flags from systemState.accelerometerData */
}

/**
 * @brief Placeholder function for flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality.
 *
 * @param data Pointer to accelerometer data to store
 * @return void
 */
void store_accelerometer_to_flash(const Vector3_t *data)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (data == NULL)
    {
        return;
    }

    /* TODO: Hardware team - implement actual flash memory write */
    /* Example interface:
     * flash_write_accelerometer_data(data->x, data->y, data->z);
     */
}

/* ===== GYROSCOPE DATA PROCESSING ===== */

/**
 * @brief Send gyroscope health data to telemetry
 *
 * This function sends individual gyroscope axis health flags
 * to the telemetry system.
 *
 * @return void
 */
void send_gyroscope_health_to_telemetry(void)
{
    /* Placeholder implementation for telemetry system */
    /* Hardware team should implement actual telemetry transmission here */

    /* Extract individual health flags */
    bool x_axis_healthy = systemState.gyroscopeData.gyro_x_health_ok;
    bool y_axis_healthy = systemState.gyroscopeData.gyro_y_health_ok;
    bool z_axis_healthy = systemState.gyroscopeData.gyro_z_health_ok;

    /* TODO: Hardware team - implement actual telemetry transmission */
    /* Example interface:
     * telemetry_send_gyro_health(x_axis_healthy, y_axis_healthy, z_axis_healthy);
     * telemetry_send_gyro_data(systemState.gyroscopeData.gyro_current.x,
     *                         systemState.gyroscopeData.gyro_current.y,
     *                         systemState.gyroscopeData.gyro_current.z);
     */

    /* For now, just store in system state for other modules to access */
    /* Telemetry system can read these flags from systemState.gyroscopeData */
}

/**
 * @brief Process gyroscope data with health check and flash storage
 *
 * This function processes raw gyroscope data, performs health checks,
 * and stores data to flash memory.
 * Each axis is processed individually based on its health status.
 *
 * @return void
 */
void process_gyroscope_data(void)
{
    /* Check individual gyroscope axis health */
    bool gyro_x_ok = ((systemState.navigationState.sensorHealth & SENSOR_GYRO_X_IS_VALID) == 0U);
    bool gyro_y_ok = ((systemState.navigationState.sensorHealth & SENSOR_GYRO_Y_IS_VALID) == 0U);
    bool gyro_z_ok = ((systemState.navigationState.sensorHealth & SENSOR_GYRO_Z_IS_VALID) == 0U);

    /* Process each axis individually based on health status */

    /* X-axis processing */
    if (gyro_x_ok)
    {
        /* X-axis is healthy - convert LSB to double (rad/s) */
        systemState.gyroscopeData.gyro_obc.x = (double)systemState.imuData.gyro_x_raw * CONV_GYRO_LSB_TO_RAD_S;
        /* Apply gyro offset to converted value */
        systemState.gyroscopeData.gyro_current.x = systemState.gyroscopeData.gyro_obc.x - systemState.navigationState.gyro_offset.x;
        /* Update previous value for future use (store offset-corrected value) */
        systemState.gyroscopeData.gyro_previous.x = systemState.gyroscopeData.gyro_current.x;
    }
    else
    {
        /* X-axis is unhealthy - use previous data (already has offset applied) */
        systemState.gyroscopeData.gyro_current.x = systemState.gyroscopeData.gyro_previous.x;
    }

    /* Y-axis processing */
    if (gyro_y_ok)
    {
        /* Y-axis is healthy - convert LSB to double (rad/s) */
        systemState.gyroscopeData.gyro_obc.y = (double)systemState.imuData.gyro_y_raw * CONV_GYRO_LSB_TO_RAD_S;
        /* Apply gyro offset to converted value */
        systemState.gyroscopeData.gyro_current.y = systemState.gyroscopeData.gyro_obc.y - systemState.navigationState.gyro_offset.y;
        /* Update previous value for future use (store offset-corrected value) */
        systemState.gyroscopeData.gyro_previous.y = systemState.gyroscopeData.gyro_current.y;
    }
    else
    {
        /* Y-axis is unhealthy - use previous converted data (already has offset applied) */
        systemState.gyroscopeData.gyro_current.y = systemState.gyroscopeData.gyro_previous.y;
    }

    /* Z-axis processing */
    if (gyro_z_ok)
    {
        /* Z-axis is healthy - convert LSB to double (rad/s) */
        systemState.gyroscopeData.gyro_obc.z = (double)systemState.imuData.gyro_z_raw * CONV_GYRO_LSB_TO_RAD_S;
        /* Apply gyro offset to converted value */
        systemState.gyroscopeData.gyro_current.z = systemState.gyroscopeData.gyro_obc.z - systemState.navigationState.gyro_offset.z;
        /* Update previous value for future use (store offset-corrected value) */
        systemState.gyroscopeData.gyro_previous.z = systemState.gyroscopeData.gyro_current.z;
    }
    else
    {
        /* Z-axis is unhealthy - use previous converted data (already has offset applied) */
        systemState.gyroscopeData.gyro_current.z = systemState.gyroscopeData.gyro_previous.z;
    }

    /* Calculate angular rates for DAP (rad/s) */
    systemState.angularRates.x = systemState.gyroscopeData.gyro_current.x; /* Roll rate */
    systemState.angularRates.y = systemState.gyroscopeData.gyro_current.y; /* Pitch rate */
    systemState.angularRates.z = systemState.gyroscopeData.gyro_current.z; /* Yaw rate */

    /* Calculate roll rate in rps (revolutions per second) for sequencer */
    /* Convert from rad/s to rps: rps = (rad/s) / (2*PI) */
    double roll_rate_rad_s = systemState.gyroscopeData.gyro_current.x;
    systemState.rollRateFp = fabs(roll_rate_rad_s / (2.0 * MATH_PI));

    /* Store processed data to flash memory */
    store_gyroscope_to_flash(&systemState.gyroscopeData.gyro_current);

    /* Set individual health flags for telemetry */
    systemState.gyroscopeData.gyro_x_health_ok = gyro_x_ok;
    systemState.gyroscopeData.gyro_y_health_ok = gyro_y_ok;
    systemState.gyroscopeData.gyro_z_health_ok = gyro_z_ok;

    /* Send health data to telemetry */
    send_gyroscope_health_to_telemetry();
}

/* ===== MAGNETOMETER DATA PROCESSING ===== */

/**
 * @brief Placeholder function for magnetometer flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality for magnetometer data.
 *
 * @param data Pointer to magnetometer data to store
 * @return void
 */
void store_magnetometer_to_flash(const Vector3_t *data)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (data == NULL)
    {
        return;
    }

    /* TODO: Hardware team - implement actual flash memory write */
    /* Example interface:
     * flash_write_magnetometer_data(data->x, data->y, data->z);
     */
}

/**
 * @brief Send magnetometer health data to telemetry
 *
 * This function sends magnetometer health flag
 * to the telemetry system.
 *
 * @return void
 */
void send_magnetometer_health_to_telemetry(void)
{
    /* Placeholder implementation for telemetry system */
    /* Hardware team should implement actual telemetry transmission here */

    /* Extract health flag */
    bool mag_healthy = systemState.magnetometerData.mag_health_ok;

    /* TODO: Hardware team - implement actual telemetry transmission */
    /* Example interface:
     * telemetry_send_mag_health(mag_healthy);
     * telemetry_send_mag_data(systemState.magnetometerData.mag_current.x,
     *                        systemState.magnetometerData.mag_current.y,
     *                        systemState.magnetometerData.mag_current.z);
     */

    /* For now, just store in system state for other modules to access */
    /* Telemetry system can read this flag from systemState.magnetometerData */
}

/**
 * @brief Process magnetometer data with health check and flash storage
 *
 * This function processes raw magnetometer data, performs health checks,
 * and stores data to flash memory.
 * Uses overall magnetometer health status (not individual axes).
 *
 * @return void
 */
void process_magnetometer_data(void)
{
    /* Check overall magnetometer health (single flag, not individual axes) */
    bool mag_ok = ((systemState.navigationState.sensorHealth & SENSOR_MAG_IS_VALID) == 0U);

    /* Process magnetometer data based on health status */
    // TO DO -
    if (mag_ok)
    {
        /* Magnetometer is healthy - convert LSB to double (mG) */
        systemState.magnetometerData.mag_obc.x = (double)systemState.imuData.mag_x_raw * CONV_MAG_LSB_TO_MG;
        systemState.magnetometerData.mag_obc.y = (double)systemState.imuData.mag_y_raw * CONV_MAG_LSB_TO_MG;
        systemState.magnetometerData.mag_obc.z = (double)systemState.imuData.mag_z_raw * CONV_MAG_LSB_TO_MG;
        systemState.magnetometerData.mag_current = systemState.magnetometerData.mag_obc;
        /* Update previous value for future use */
        systemState.magnetometerData.mag_previous = systemState.magnetometerData.mag_current;
    }
    else
    {
        /* Magnetometer is unhealthy - use previous data */
        systemState.magnetometerData.mag_current = systemState.magnetometerData.mag_previous;
    }

    /* Store processed data to flash memory */
    store_magnetometer_to_flash(&systemState.magnetometerData.mag_current);

    /* Set health flag for telemetry */
    systemState.magnetometerData.mag_health_ok = mag_ok;

    /* Send health data to telemetry */
    send_magnetometer_health_to_telemetry();
}

/* ===== INCREMENTAL VELOCITY DATA PROCESSING ===== */

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
void store_incremental_velocity_to_flash(const Vector3_t *delta_v_converted, const Vector3_t *acceleration)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (delta_v_converted == NULL)
    {
        return;
    }

    /* Acceleration parameter is optional - only process if provided */
    /* TODO: Hardware team - implement actual flash memory write */
    /* Example interface:
     * flash_write_delta_v_converted(delta_v_converted->x, delta_v_converted->y, delta_v_converted->z);
     * if (acceleration != NULL) {
     *     flash_write_acceleration(acceleration->x, acceleration->y, acceleration->z);
     * }
     */
}

/**
 * @brief Process incremental velocity data with unit conversion and acceleration calculation
 *
 * This function processes raw incremental velocity data, converts units,
 * calculates acceleration, and stores data to flash memory.
 * No health check is needed - just conversion and calculation.
 *
 * @return void
 */
void process_incremental_velocity_data(void)
{
    /* Convert LSB to double (ft/s) - hardware team will populate raw LSB values */
    systemState.incrementalVelocityData.delta_v_obc.x = (double)systemState.incrementalVelocityData.delta_v_x_raw * CONV_INC_VEL_LSB_TO_FT_S;
    systemState.incrementalVelocityData.delta_v_obc.y = (double)systemState.incrementalVelocityData.delta_v_y_raw * CONV_INC_VEL_LSB_TO_FT_S;
    systemState.incrementalVelocityData.delta_v_obc.z = (double)systemState.incrementalVelocityData.delta_v_z_raw * CONV_INC_VEL_LSB_TO_FT_S;

    /* Convert units from ft/s to m/s */
    systemState.incrementalVelocityData.delta_v_converted.x =
        systemState.incrementalVelocityData.delta_v_obc.x * FT_TO_M_S2_CONVERSION_FACTOR;
    systemState.incrementalVelocityData.delta_v_converted.y =
        systemState.incrementalVelocityData.delta_v_obc.y * FT_TO_M_S2_CONVERSION_FACTOR;
    systemState.incrementalVelocityData.delta_v_converted.z =
        systemState.incrementalVelocityData.delta_v_obc.z * FT_TO_M_S2_CONVERSION_FACTOR;

    /* Store both converted velocity to flash memory */
    store_incremental_velocity_to_flash(&systemState.incrementalVelocityData.delta_v_converted, NULL);
}

/* ===== INCREMENTAL ANGLE DATA PROCESSING ===== */

/**
 * @brief Placeholder function for incremental angle flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality for incremental angle data.
 *
 * @param data Pointer to incremental angle data to store
 * @return void
 */
void store_incremental_angle_to_flash(const Vector3_t *data)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (data == NULL)
    {
        return;
    }

    /* TODO: Hardware team - implement actual flash memory write */
    /* Example interface:
     * flash_write_incremental_angle(data->x, data->y, data->z);
     */
}

/**
 * @brief Process incremental angle data and store to flash memory
 *
 * This function processes raw incremental angle data and stores it to flash memory.
 * No health check or unit conversion is needed - just read and store.
 *
 * @return void
 */
void process_incremental_angle_data(void)
{
    /* Convert LSB to double (radians) - hardware team will populate raw LSB values */
    systemState.incrementalAngleData.delta_theta_obc.x = (double)systemState.incrementalAngleData.delta_theta_x_raw * CONV_INC_ANGLE_LSB_TO_RAD;
    systemState.incrementalAngleData.delta_theta_obc.y = (double)systemState.incrementalAngleData.delta_theta_y_raw * CONV_INC_ANGLE_LSB_TO_RAD;
    systemState.incrementalAngleData.delta_theta_obc.z = (double)systemState.incrementalAngleData.delta_theta_z_raw * CONV_INC_ANGLE_LSB_TO_RAD;

    /* Store converted data directly to flash memory (no processing needed) */
    store_incremental_angle_to_flash(&systemState.incrementalAngleData.delta_theta_obc);
}

/* ===== ATTITUDE TELEMETRY PROCESSING ===== */

/**
 * @brief Placeholder function for attitude flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality for attitude data.
 *
 * @param data Pointer to attitude data to store
 * @return void
 */
void store_attitude_to_flash(const Vector3_t *data)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (data == NULL)
    {
        return;
    }

    /* TODO: Hardware team - implement actual flash memory write */
    /* Example interface:
     * flash_write_attitude(data->x, data->y, data->z);
     */
}

/**
 * @brief Send attitude data to telemetry
 *
 * This function sends attitude angles (roll, pitch, yaw) to the telemetry system.
 *
 * @return void
 */
void send_attitude_to_telemetry(void)
{
    /* Placeholder implementation for telemetry system */
    /* Hardware team should implement actual telemetry transmission here */

    /* Extract attitude angles */
    double roll_rad = systemState.attitudeTelemetryData.attitude_rad.x;
    double pitch_rad = systemState.attitudeTelemetryData.attitude_rad.y;
    double yaw_rad = systemState.attitudeTelemetryData.attitude_rad.z;

    /* TODO: Hardware team - implement actual telemetry transmission */
    /* Example interface:
     * telemetry_send_attitude(roll_rad, pitch_rad, yaw_rad);
     */

    /* For now, just store in system state for other modules to access */
    /* Telemetry system can read these values from systemState.attitudeTelemetryData */
}

/**
 * @brief Send navigation confirmation flags to telemetry
 *
 * This function sends magnetometer and gyroscope 3-cycle confirmation flags
 * to the telemetry system.
 *
 * @return void
 */
void send_navigation_confirmation_flags_to_telemetry(void)
{
    /* Placeholder implementation for telemetry system */
    /* Hardware team should implement actual telemetry transmission here */

    /* Extract navigation confirmation flags from system state */
    bool mag_3_cycles_confirmed = systemState.navigationState.mag_3_cycles_confirmed;
    bool gyro_3_cycles_confirmed = systemState.navigationState.gyro_3_cycles_confirmed;

    /* TODO: Hardware team - implement actual telemetry transmission */
    /* Example interface:
     * telemetry_send_nav_confirmation_flags(mag_3_cycles_confirmed, gyro_3_cycles_confirmed);
     */

    /* For now, just store in system state for other modules to access */
    /* Telemetry system can read these flags from systemState.navigationState */
}

/* ===== PEFCS INITIALIZATION ===== */

/**
 * @brief Load default PEFCS parameters into system state
 *
 * This function initializes all mission-specific parameters from pefcs_init.h
 * Called automatically when OBC reset flag is received
 *
 * @return void
 */
void load_pefcs_defaults(void)
{
    /* ===== NAVIGATION PARAMETERS ===== */

    /* Initial attitude angles (convert from degrees to radians) */
    double azimuth_rad, elevation_rad;
    deg_to_rad(&azimuth_rad, PEFCS_INITIAL_AZIMUTH_DEG);
    deg_to_rad(&elevation_rad, PEFCS_INITIAL_ELEVATION_DEG);

    systemState.navigationState.pitch_deg = PEFCS_INITIAL_ELEVATION_DEG;
    systemState.navigationState.yaw_deg = PEFCS_INITIAL_AZIMUTH_DEG;

    /* Magnetic field reference in NED frame (milliGauss) */
    systemState.navigationState.mag_ref_ned.x = PEFCS_MAG_REF_NORTH_MG;
    systemState.navigationState.mag_ref_ned.y = PEFCS_MAG_REF_EAST_MG;
    systemState.navigationState.mag_ref_ned.z = PEFCS_MAG_REF_DOWN_MG;

    /* Hard iron calibration offsets */
    systemState.navigationState.mag_hard_iron.x = PEFCS_MAG_HARD_IRON_X;
    systemState.navigationState.mag_hard_iron.y = PEFCS_MAG_HARD_IRON_Y;
    systemState.navigationState.mag_hard_iron.z = PEFCS_MAG_HARD_IRON_Z;

    /* Soft iron calibration matrix */
    systemState.navigationState.mag_soft_iron[0][0] = PEFCS_MAG_SOFT_IRON_XX;
    systemState.navigationState.mag_soft_iron[0][1] = PEFCS_MAG_SOFT_IRON_XY;
    systemState.navigationState.mag_soft_iron[0][2] = PEFCS_MAG_SOFT_IRON_XZ;
    systemState.navigationState.mag_soft_iron[1][0] = PEFCS_MAG_SOFT_IRON_YX;
    systemState.navigationState.mag_soft_iron[1][1] = PEFCS_MAG_SOFT_IRON_YY;
    systemState.navigationState.mag_soft_iron[1][2] = PEFCS_MAG_SOFT_IRON_YZ;
    systemState.navigationState.mag_soft_iron[2][0] = PEFCS_MAG_SOFT_IRON_ZX;
    systemState.navigationState.mag_soft_iron[2][1] = PEFCS_MAG_SOFT_IRON_ZY;
    systemState.navigationState.mag_soft_iron[2][2] = PEFCS_MAG_SOFT_IRON_ZZ;

    /* Gyroscope bias offsets */
    systemState.navigationState.gyro_offset.x = PEFCS_GYRO_OFFSET_X;
    systemState.navigationState.gyro_offset.y = PEFCS_GYRO_OFFSET_Y;
    systemState.navigationState.gyro_offset.z = PEFCS_GYRO_OFFSET_Z;

    /* Accelerometer bias offsets */
    systemState.navigationState.accel_offset.x = PEFCS_ACCEL_OFFSET_X;
    systemState.navigationState.accel_offset.y = PEFCS_ACCEL_OFFSET_Y;
    systemState.navigationState.accel_offset.z = PEFCS_ACCEL_OFFSET_Z;

    /* ===== GUIDANCE PARAMETERS ===== */
    /* Guidance module initializes its own parameters from PEFCS */
    guidance_init();

    /* ===== DAP PARAMETERS ===== */

    /* Roll Controller Gains (from PEFCS) */
    systemState.dapParams.kp_roll = PEFCS_DAP_KP_ROLL;
    systemState.dapParams.ki_roll = PEFCS_DAP_KI_ROLL;
    systemState.dapParams.kr_roll = PEFCS_DAP_KR_ROLL;

    /* Roll Controller Limits (from PEFCS) */
    systemState.dapParams.integratorR_min_rad = PEFCS_DAP_INTEGRATOR_R_MIN;
    systemState.dapParams.integratorR_max_rad = PEFCS_DAP_INTEGRATOR_R_MAX;
    systemState.dapParams.phi_min_rad = PEFCS_DAP_PHI_MIN;
    systemState.dapParams.phi_max_rad = PEFCS_DAP_PHI_MAX;

    /* Pitch Controller Gains & Filter Params (from PEFCS) */
    systemState.dapParams.kp_pitch = PEFCS_DAP_KP_PITCH;
    systemState.dapParams.ki_pitch = PEFCS_DAP_KI_PITCH;
    systemState.dapParams.kr_pitch = PEFCS_DAP_KR_PITCH;
    systemState.dapParams.K_LPF_Pitch = PEFCS_DAP_K_LPF_PITCH;
    systemState.dapParams.wC_pitch = PEFCS_DAP_WC_PITCH;
    systemState.dapParams.pitch_a = PEFCS_DAP_PITCH_A;
    systemState.dapParams.pitch_b = PEFCS_DAP_PITCH_B;

    /* Pitch Controller Limits (from PEFCS) */
    systemState.dapParams.theta_min_rad = PEFCS_DAP_THETA_MIN;
    systemState.dapParams.theta_max_rad = PEFCS_DAP_THETA_MAX;
    systemState.dapParams.integratorP_min_rad = PEFCS_DAP_INTEGRATOR_P_MIN;
    systemState.dapParams.integratorP_max_rad = PEFCS_DAP_INTEGRATOR_P_MAX;

    /* Yaw Controller Gains & Filter Params (from PEFCS) */
    systemState.dapParams.kp_yaw = PEFCS_DAP_KP_YAW;
    systemState.dapParams.ki_yaw = PEFCS_DAP_KI_YAW;
    systemState.dapParams.kr_yaw = PEFCS_DAP_KR_YAW;
    systemState.dapParams.K_LPF_Yaw = PEFCS_DAP_K_LPF_YAW;
    systemState.dapParams.wC_yaw = PEFCS_DAP_WC_YAW;
    systemState.dapParams.yaw_a = PEFCS_DAP_YAW_A;
    systemState.dapParams.yaw_b = PEFCS_DAP_YAW_B;

    /* Yaw Controller Limits (from PEFCS) */
    systemState.dapParams.psi_min_rad = PEFCS_DAP_PSI_MIN;
    systemState.dapParams.psi_max_rad = PEFCS_DAP_PSI_MAX;
    systemState.dapParams.integratorY_min_rad = PEFCS_DAP_INTEGRATOR_Y_MIN;
    systemState.dapParams.integratorY_max_rad = PEFCS_DAP_INTEGRATOR_Y_MAX;

    /* Actuator Parameters */
    systemState.dapParams.actuator_deadband_rad = PEFCS_ACTUATOR_DEADBAND_RAD;
    systemState.dapParams.actuator_max_deflection_rad = PEFCS_ACTUATOR_MAX_DEFLECTION_RAD;

    /* ===== SEQUENCER PARAMETERS ===== */
    systemState.sequencerParams.confirmationCycles = PEFCS_SEQ_CONFIRMATION_CYCLES;
    systemState.sequencerParams.rollRateT1Threshold = PEFCS_SEQ_ROLL_RATE_T1_THRESHOLD;
    systemState.sequencerParams.rollRateT2Threshold = PEFCS_SEQ_ROLL_RATE_T2_THRESHOLD;
    systemState.sequencerParams.t1WindowIn = PEFCS_SEQ_T1_WINDOW_IN_TIME;
    systemState.sequencerParams.t1WindowOut = PEFCS_SEQ_T1_WINDOW_OUT_TIME;
    systemState.sequencerParams.t2WindowIn = PEFCS_SEQ_T2_WINDOW_IN_TIME;
    systemState.sequencerParams.t2WindowOut = PEFCS_SEQ_T2_WINDOW_OUT_TIME;
    systemState.sequencerParams.t3WindowIn = PEFCS_SEQ_T3_WINDOW_IN_TIME;
    systemState.sequencerParams.t3WindowOut = PEFCS_SEQ_T3_WINDOW_OUT_TIME;
    systemState.sequencerParams.tProximity = PEFCS_SEQ_T_PROXIMITY;
    systemState.sequencerParams.canardDeployDelay = PEFCS_SEQ_CANARD_DEPLOY_FLAG_DELAY;
    systemState.sequencerParams.canardControlDelay = PEFCS_SEQ_CANARD_CONTROL_ON_FLAG_DELAY;
    systemState.sequencerParams.fsaDelay = PEFCS_SEQ_FSA_FLAG_DELAY;
    systemState.sequencerParams.guidStartDelay = PEFCS_SEQ_GUID_START_FLAG_DELAY;
}

/* ===== NAVIGATION IMPLEMENTATION ===== */

/**
 * @brief Gyroscope attitude integration using Euler kinematic equations
 *
 * This function integrates body angular rates from gyroscope data
 * using proper Euler kinematic transformations to update attitude angles.
 * Accounts for axis coupling in 3D rotation.
 *
 * @return void
 */
static void gyroscope_attitude_integration(void)
{

    /* Time step (10ms minor cycle) - Using dimensionless step size */
    double dt = INTEGRATION_STEP_SIZE;

    /* Read current attitude (Roll, Yaw, Pitch order from telemetry) */
    double phi = systemState.navigationState.attitude_e.roll_rad;
    double theta = systemState.navigationState.attitude_e.pitch_rad;
    double psi = systemState.navigationState.attitude_e.yaw_rad;

    /* Map body rates from incremental angle data */
    /* p, q, r are the body angular rates (rad/s) */
    double p = systemState.incrementalAngleData.delta_theta_obc.x / dt;
    double q = systemState.incrementalAngleData.delta_theta_obc.y / dt;
    double r = systemState.incrementalAngleData.delta_theta_obc.z / dt;

    /* Calculate Euler angle rates using kinematic equations */
    /* These transform body rates to Euler angle rates */
    double psi_dot = (q * cos(phi) + r * sin(phi)) / cos(theta);
    double theta_dot = r * cos(phi) - q * sin(phi);
    double phi_dot = p + (q * cos(phi) + r * sin(phi)) * tan(theta);

    /* Integrate Euler angle rates to get new attitude */
    phi = phi + phi_dot * dt;
    theta = theta + theta_dot * dt;
    psi = psi + psi_dot * dt;

    /* Wrap using mod_double */
    mod_double(&phi, phi, 2.0 * MATH_PI);
    // mod_double(&theta, theta, 2.0 * MATH_PI);
    // mod_double(&psi, psi, 2.0 * MATH_PI);

    /* Store integrated attitude for DAP/telemetry */
    systemState.attitudeTelemetryData.attitude_rad.x = phi;
    systemState.attitudeTelemetryData.attitude_rad.y = psi;   /* Yaw */
    systemState.attitudeTelemetryData.attitude_rad.z = theta; /* Pitch */
    systemState.navigationState.attitude_e.roll_rad = phi;
    systemState.navigationState.attitude_e.pitch_rad = theta;
    systemState.navigationState.attitude_e.yaw_rad = psi;
}

static void magnetometer_attitude_estimation(void)
{
    /* Attitude angles from checkout system (input in degrees, converted to radians) */
    double theta; /* Pitch angle in radians */
    double psi;   /* Yaw angle in radians */

    /* Convert checkout input angles from degrees to radians */
    deg_to_rad(&theta, systemState.navigationState.pitch_deg); /* Pitch angle */
    deg_to_rad(&psi, systemState.navigationState.yaw_deg);     /* Yaw angle */

    double phi = 0.0; /* Roll angle - will be estimated */

    // to be initialized in init remove from here.

    /* Get current magnetometer data */
    Vector3_t mag_current = systemState.magnetometerData.mag_current;
    Vector3_t mag_offset_corrected;
    Vector3_t mag_calibrated;

    /* Get calibration parameters from navigation state (loaded from PEFCS) */
    Vector3_t hard_iron = systemState.navigationState.mag_hard_iron;
    double(*soft_iron)[3] = systemState.navigationState.mag_soft_iron;

    /* Step 1: Subtract hard iron offset (HI) */
    mag_offset_corrected.x = mag_current.x - hard_iron.x;
    mag_offset_corrected.y = mag_current.y - hard_iron.y;
    mag_offset_corrected.z = mag_current.z - hard_iron.z;

    /* Step 2: Apply soft iron correction matrix (SI) */
    mag_calibrated.x = soft_iron[0][0] * mag_offset_corrected.x + soft_iron[0][1] * mag_offset_corrected.y + soft_iron[0][2] * mag_offset_corrected.z;
    mag_calibrated.y = soft_iron[1][0] * mag_offset_corrected.x + soft_iron[1][1] * mag_offset_corrected.y + soft_iron[1][2] * mag_offset_corrected.z;
    mag_calibrated.z = soft_iron[2][0] * mag_offset_corrected.x + soft_iron[2][1] * mag_offset_corrected.y + soft_iron[2][2] * mag_offset_corrected.z;

    /* Store calibrated data in system state */
    systemState.magnetometerData.mag_offset_corrected = mag_offset_corrected;
    systemState.magnetometerData.mag_calibrated = mag_calibrated;

    /* Magnetic field reference in NED frame (from PEFCS) */
    double m_n = systemState.navigationState.mag_ref_ned.x; /* North component (mG) */
    double m_e = systemState.navigationState.mag_ref_ned.y; /* East component (mG) */
    double m_d = systemState.navigationState.mag_ref_ned.z; /* Down component (mG) */

    /* Transform to navigation frame coordinates */
    double mn = m_n;
    double mv = -m_d; /* Vertical: negative down */
    double me = m_e;

    /* Roll estimation using refined formula from validated test code */
    /* This formula properly accounts for the full magnetic reference vector */
    double phi1 = atan2(
        (sin(theta) * cos(psi) * mn + cos(theta) * mv + sin(theta) * sin(psi) * me) * mag_calibrated.z -
            (-sin(psi) * mn + cos(psi) * me) * mag_calibrated.y,
        (sin(theta) * cos(psi) * mn + cos(theta) * mv + sin(theta) * sin(psi) * me) * mag_calibrated.y +
            (-sin(psi) * mn + cos(psi) * me) * mag_calibrated.z);

    /* Wrap roll angle to 2π */
    mod_double(&phi, phi1, 2.0 * MATH_PI);

    /* Update navigation state with estimated attitude */
    systemState.navigationState.attitude_e.roll_rad = phi;
    systemState.navigationState.attitude_e.pitch_rad = theta;
    systemState.navigationState.attitude_e.yaw_rad = psi;

    /* Save attitude values to telemetry memory */
    systemState.attitudeTelemetryData.attitude_rad.x = phi;   /* Roll */
    systemState.attitudeTelemetryData.attitude_rad.y = psi;   /* Yaw */
    systemState.attitudeTelemetryData.attitude_rad.z = theta; /* Pitch */
}

/**
 * @brief Process navigation data and estimate roll rate
 *
 * @param dt_s Time step in seconds
 * @return void
 */
static void process_navigation(double dt_s)
{
    /* ===== ROLL RATE CALCULATION ===== */
    /* Read roll rate from gyroscope data */
    rate = fabs(systemState.gyroscopeData.gyro_current.x / (2.0 * MATH_PI));

    // add a shared state to save phi, theta , psi TO BE SAVED IN csv

    /* ===== GYROSCOPE ATTITUDE INTEGRATION LOGIC ===== */
    if (gyro_3_cycles_confirmed)
    {
        /* Gyro confirmed - integrate angular rates every cycle */
        gyroscope_attitude_integration();
        gyroattitude = true;
    }
    else if (rate <= 2.0)
    {
        /* Rate condition met for gyroscope - check for 3 consecutive cycles */
        gyro_rate_condition_met = true;
        gyrocount++;
        if (gyrocount >= 3)
        {
            gyro_3_cycles_confirmed = true;
            systemState.navigationState.gyro_3_cycles_confirmed = true;
            gyroscope_attitude_integration();
            gyroattitude = true;
        }
    }
    else
    {
        /* Reset gyro counter if rate condition not met */
        gyrocount = 0;
        gyro_rate_condition_met = false;
    }

    /* ===== MAGNETOMETER ATTITUDE ESTIMATION LOGIC ===== */
    /* Only perform magnetometer estimation if gyroscope integration is not active */
    if (!gyroattitude)
    {
        if (MAG_3_CYCLES_CONFIRMED)
        {
            /* Magnetometer confirmed - estimate attitude every cycle until gyro takes over */
            magnetometer_attitude_estimation();
        }
        else if (rate <= 5.0)
        {
            /* Rate condition met for magnetometer - check for 3 consecutive cycles */
            magEstOkFlag = true;
            magcount++;
            if (magcount >= 3)
            {
                MAG_3_CYCLES_CONFIRMED = true;
                systemState.navigationState.mag_3_cycles_confirmed = true;
                magnetometer_attitude_estimation();
            }
        }
        else
        {
            /* Reset mag counter if rate condition not met */
            magcount = 0;
            magEstOkFlag = false;
        }
    }

    /* If neither magnetometer nor gyroscope attitude estimation is active, */
    /* maintain previous attitude values (already stored in systemState) */

    /* Store attitude data to flash memory */
    store_attitude_to_flash(&systemState.attitudeTelemetryData.attitude_rad);

    /* Send attitude data to telemetry */
    send_attitude_to_telemetry();

    /* Send navigation confirmation flags to telemetry */
    send_navigation_confirmation_flags_to_telemetry();
}

/* ===== SEQUENCER IMPLEMENTATION ===== */

/**
 * @brief Initialize sequencer state
 *
 * This function initializes all sequencer state variables to safe default values.
 *
 * @param state Pointer to sequencer state structure
 * @return void
 */
static void sequencer_init(SequencerState_t *state)
{
    /* Flight software rule: Always validate input parameters first */
    if (state == NULL)
    {
        return;
    }

    /* Initialize all state to zero (safest starting condition) */
    memset(state, 0, sizeof(SequencerState_t));

    /* Explicitly set initial values (makes code self-documenting) */
    state->isT0Set = false;
    state->isT1Set = false;
    state->isT2Set = false;
    state->isT3Set = false;

    /* Initialize all flags to false */
    state->isFsaActivateFlagSent = false;
    state->isCanardDeployFlagSent = false;
    state->isCanardControlFlagSent = false;
    state->isGuidStartFlagSent = false;

    /* Initialize counters and timers to zero */
    state->mainClockCycles = 0U;
    state->fsaActivateFlagSendTime = 0U;
    state->canardDeployFlagSendTime = 0U;
    state->canardControlFlagSendTime = 0U;
    state->guidStartFlagSendTime = 0U;
    state->t1RollRateCount = 0U;
    state->t2RollRateCount = 0U;

    /* OBC reset starts inactive */
    state->isOBCReset = false;
}

/**
 * @brief Set OBC reset status for sequencer
 *
 * This function sets the OBC reset flag and initializes sequencer timing.
 * Called when OBC reset flag becomes true.
 *
 * @param state Pointer to sequencer state structure
 * @param isActive true to activate OBC reset, false to deactivate
 * @return void
 */
static void sequencer_set_obc_reset(SequencerState_t *state, bool isActive)
{
    /* Parameter validation */
    if (state == NULL)
    {
        return;
    }

    /* Set OBC reset state */
    state->isOBCReset = isActive;

    if (isActive)
    {
        /* OBC reset activation triggers the start of timing */
        /* This is T=0 moment - reset the OBC */
        state->mainClockCycles = 0U;

        /* Set T0 phase immediately when OBC reset activates */
        state->isT0Set = true;

        /* Reset all confirmation counters */
        state->t1RollRateCount = 0U;
        state->t2RollRateCount = 0U;
    }
    /* Once launched, it stays launched - no action when OBC reset goes inactive */
}

/**
 * @brief Helper function to check roll rate condition for T1
 *
 * @param rollRateFp Roll rate in rps (revolutions per second)
 * @param params Pointer to sequencer parameters
 * @return true if roll rate is OK for T1, false otherwise
 */
static bool is_roll_rate_ok_for_t1(double rollRateFp, const SequencerParams_t *params)
{
    return (rollRateFp <= params->rollRateT1Threshold);
}

/**
 * @brief Helper function to check roll rate condition for T2
 *
 * @param rollRateFp Roll rate in rps (revolutions per second)
 * @param params Pointer to sequencer parameters
 * @return true if roll rate is OK for T2, false otherwise
 */
static bool is_roll_rate_ok_for_t2(double rollRateFp, const SequencerParams_t *params)
{
    return (rollRateFp <= params->rollRateT2Threshold);
}

/**
 * @brief Process T1 phase logic
 *
 * Handles T1 phase detection based on roll rate and timing windows.
 *
 * @param state Pointer to sequencer state structure
 * @param rollRateFp Roll rate in rps
 * @param params Pointer to sequencer parameters
 * @param output Pointer to sequencer output structure
 * @return void
 */
static void process_t1_logic(SequencerState_t *state, double rollRateFp,
                             const SequencerParams_t *params, SequencerOutput_t *output)
{
    /* First check if T1 window is out (T > T1WindowOut) */
    if (state->mainClockCycles > params->t1WindowOut)
    {
        /* Window out - Set T1 immediately */
        state->isT1Set = true;
        output->setT1 = true;
        state->t1SetTime = state->mainClockCycles; /* Record T1 set time */

        /* Schedule the canard deploy flag if not already sent */
        if (!state->isCanardDeployFlagSent)
        {
            state->canardDeployFlagSendTime = state->mainClockCycles + params->canardDeployDelay;
        }

        return;
    }

    /* Check for T1 window sensing */
    if (state->mainClockCycles > params->t1WindowIn)
    {
        /* Check if the roll rate <= 7rps conditions are met */
        if (is_roll_rate_ok_for_t1(rollRateFp, params))
        {
            /* If the roll rate is good, increment the confirmation counter */
            state->t1RollRateCount++;

            /* Check if the condition has been met for the required number of cycles */
            if (state->t1RollRateCount >= params->confirmationCycles)
            {
                /* Set T1 */
                state->isT1Set = true;
                output->setT1 = true;
                state->t1SetTime = state->mainClockCycles;

                /* Schedule the canard deploy flag if not already sent */
                if (!state->isCanardDeployFlagSent)
                {
                    state->canardDeployFlagSendTime = state->mainClockCycles + params->canardDeployDelay;
                }

                return;
            }
        }
        else
        {
            /* Roll rate is not within the threshold - reset the counter */
            state->t1RollRateCount = 0U;
        }
    }
}

/**
 * @brief Process T2 phase logic
 *
 * Handles T2 phase detection based on roll rate and timing windows.
 *
 * @param state Pointer to sequencer state structure
 * @param rollRateFp Roll rate in rps
 * @param params Pointer to sequencer parameters
 * @param output Pointer to sequencer output structure
 * @return void
 */
static void process_t2_logic(SequencerState_t *state, double rollRateFp,
                             const SequencerParams_t *params, SequencerOutput_t *output)
{
    /* Check if it's time to send the canard deploy flag */
    if (!state->isCanardDeployFlagSent && (state->mainClockCycles > state->canardDeployFlagSendTime))
    {
        output->canardDeployFlag = true;
        state->isCanardDeployFlagSent = true;
    }

    /* Handle the timeout conditions */
    if (state->mainClockCycles > (state->t1SetTime + params->t2WindowOut))
    {
        /* Window expired - set T2 immediately */
        state->isT2Set = true;
        output->setT2 = true;
        state->t2SetTime = state->mainClockCycles; /* Record T2 set time */
        /* Schedule the Canard control flag */
        state->canardControlFlagSendTime = state->mainClockCycles + params->canardControlDelay;
        return;
    }

    /* Check the T2 detection window */
    if (state->mainClockCycles > (state->t1SetTime + params->t2WindowIn))
    {
        /* Check if the roll rate is below the T2 threshold (<= 2 rps) */
        if (is_roll_rate_ok_for_t2(rollRateFp, params))
        {
            state->t2RollRateCount++;
            /* Check if the roll rate persists for 3 consecutive cycles */
            if (state->t2RollRateCount >= params->confirmationCycles)
            {
                /* Set T2 */
                state->isT2Set = true;
                output->setT2 = true;
                state->t2SetTime = state->mainClockCycles; /* Record T2 set time */
                /* Schedule the canard control flag */
                state->canardControlFlagSendTime = state->mainClockCycles + params->canardControlDelay;
                return;
            }
        }
        else
        {
            /* Roll rate is not within the threshold - reset the counter */
            state->t2RollRateCount = 0U;
        }
    }
}

/**
 * @brief Process T3 phase logic
 *
 * Handles T3 phase detection based on time-to-go and timing windows.
 * Note: tGo is in seconds (double), not cycles.
 *
 * @param state Pointer to sequencer state structure
 * @param tGo Time to go from guidance in seconds (double)
 * @param params Pointer to sequencer parameters
 * @param output Pointer to sequencer output structure
 * @return void
 */
static void process_t3_logic(SequencerState_t *state, double tGo,
                             const SequencerParams_t *params, SequencerOutput_t *output)
{
    /* First check if guidance start flag is already sent */
    if (state->isGuidStartFlagSent)
    {
        /* Guidance start flag has already been sent */
        /* Check if FSA flag is already sent */
        if (state->isFsaActivateFlagSent)
        {
            /* Both guidance and FSA flags are sent */
            /* Check if T3 window is out */
            if (state->mainClockCycles > (state->t2SetTime + params->t3WindowOut))
            {
                /* T3 window is out, set T3 if not already set */
                if (!state->isT3Set)
                {
                    state->isT3Set = true;
                    output->setT3 = true;
                    state->t3SetTime = state->mainClockCycles;
                    output->enableProximitySensor = true;
                }
            }
            else
            {
                /* T3 window is not out, check if T3 window in */
                if (state->mainClockCycles > (state->t2SetTime + params->t3WindowIn))
                {
                    /* T3 window is in, check tGo from guidance (in seconds) */
                    if (tGo < params->tProximity)
                    {
                        /* tGo is less than time to enable proximity sensor */
                        state->isT3Set = true;
                        output->setT3 = true;
                        state->t3SetTime = state->mainClockCycles;
                        output->enableProximitySensor = true;
                    }
                    /* If tGo not less than threshold, exit */
                }
                /* If not T3 window in, exit */
            }
        }
        else
        {
            /* FSA flag not sent yet */
            /* Check if it's time to send the FSA flag */
            if (state->mainClockCycles > state->fsaActivateFlagSendTime)
            {
                output->fsaActivateFlag = true;
                state->isFsaActivateFlagSent = true;
            }
        }
    }
    else
    {
        /* Guidance start flag has not been sent */
        /* Check if canard control flag is already sent */
        if (state->isCanardControlFlagSent)
        {
            /* Control flag has been sent */
            /* First check if FSA flag is already sent */
            if (state->isFsaActivateFlagSent)
            {
                /* FSA flag already sent */
                /* Now check if it's time to send the Guidance Start Flag */
                if (state->mainClockCycles > state->guidStartFlagSendTime)
                {
                    output->sendGuidStartFlag = true;
                    state->isGuidStartFlagSent = true;
                    return;
                }
                /* Not time for guidance flag yet, exit and wait for next cycle */
                return;
            }
            else
            {
                /* FSA flag not sent yet */
                /* Check if it's time to send FSA flag */
                if (state->mainClockCycles > state->fsaActivateFlagSendTime)
                {
                    output->fsaActivateFlag = true;
                    state->isFsaActivateFlagSent = true;
                }

                /* Check if it's time to send the Guidance Start Flag regardless of FSA flag */
                if (state->mainClockCycles > state->guidStartFlagSendTime)
                {
                    output->sendGuidStartFlag = true;
                    state->isGuidStartFlagSent = true;
                    return;
                }
                return;
            }
        }
        else
        {
            /* Canard control flag not sent yet */
            /* Check if it's time to send the canard control flag */
            if (state->mainClockCycles > state->canardControlFlagSendTime)
            {
                output->canardControlFlag = true;
                state->isCanardControlFlagSent = true;

                /* Schedule the flags to be sent */
                state->fsaActivateFlagSendTime = state->mainClockCycles + params->fsaDelay;
                state->guidStartFlagSendTime = state->mainClockCycles + params->guidStartDelay;
            }
            /* If not time, we just return and check again next cycle */

            /* Check if T2 window is out */
            if (state->mainClockCycles > (state->t1SetTime + params->t2WindowOut))
            {
                /* T2 window out, set T2 immediately if not already set */
                if (!state->isT2Set)
                {
                    state->isT2Set = true;
                    output->setT2 = true;
                    state->t2SetTime = state->mainClockCycles;
                }
            }
            else
            {
                /* T2 window not out, check if T2 window in */
                if (state->mainClockCycles > (state->t1SetTime + params->t2WindowIn))
                {
                    /* T2 window in logic */
                    /* In processT3Logic we already know T2 conditions were met */
                    /* This is just for completeness */
                }
            }
        }
    }
}

/**
 * @brief Execute sequencer algorithm
 *
 * This function executes the sequencer state machine logic.
 * It reads roll rate from systemState and timeToGo from guidance,
 * processes phase transitions, and updates systemState.flags.* publicly.
 *
 * @param dt_s Time step in seconds (0.01s for 100Hz)
 * @return void
 */
static void execute_sequencer(double dt_s)
{
    /* Read roll rate from system state (in rps) */
    double rollRateFp = systemState.rollRateFp;

    /* Read timeToGo from guidance (in seconds, double) */
    double tGo = systemState.guidanceState.timeToGo;

    /* Get pointer to sequencer state and parameters */
    SequencerState_t *state = &systemState.sequencerState;
    const SequencerParams_t *params = &systemState.sequencerParams;

    /* Initialize sequencer output structure */
    SequencerOutput_t output;
    memset(&output, 0, sizeof(SequencerOutput_t)); /* Safe starting state */

    /* Increment main clock if OBC reset is active */
    if (state->isOBCReset)
    {
        state->mainClockCycles++;
    }

    /* Priority-based logic for ISA */
    /* Check T3 first, then T2, then T1, then T0 */

    if (state->isT3Set)
    {
        /* T3 is set - this is the end, sequencer exits */
        return;
    }

    if (state->isT2Set)
    {
        /* The projectile is in T2 phase - check for T3 conditions */
        process_t3_logic(state, tGo, params, &output);
    }
    else if (state->isT1Set)
    {
        /* The projectile is in T1 phase - check for T2 conditions */
        process_t2_logic(state, rollRateFp, params, &output);
    }
    else if (state->isT0Set)
    {
        /* The projectile is in T0 phase, check for T1 conditions */
        process_t1_logic(state, rollRateFp, params, &output);
    }

    /* Update systemState.flags.* publicly from sequencer output */
    /* All flags and phase transitions are now accessible to other functions */
    /* Use latch semantics - once a flag is set, it remains set permanently */
    if (output.fsaActivateFlag)
        systemState.flags.fsaActivateFlag = true;
    if (output.canardDeployFlag)
        systemState.flags.canardDeployFlag = true;
    if (output.canardControlFlag)
        systemState.flags.canardControlFlag = true;
    if (output.sendGuidStartFlag)
        systemState.flags.guidStartFlag = true;
    if (output.enableProximitySensor)
        systemState.flags.proximitySensorFlag = true;

    /* Update phase flags in systemState.flags.* */
    if (output.setT0)
    {
        systemState.flags.isT0Set = true;
    }
    if (output.setT1)
    {
        systemState.flags.isT1Set = true;
    }
    if (output.setT2)
    {
        systemState.flags.isT2Set = true;
    }
    if (output.setT3)
    {
        systemState.flags.isT3Set = true;
    }
}

/* ===== DAP IMPLEMENTATION ===== */

/* Module-level variables for rate derivative calculations */
/* Module-level variables for rate derivative calculations */
static double previousPitchrate = 0.0;
static double previousYawrate = 0.0;

/* DAP State Variables (Matching test/dap/dap.c) */
static double rollIntegrator = 0.0;
static double LPF_out_Pitch = 0.0;
static double lag_out_pitch = 0.0;
static double theta = 0.0;
static double pitchIntegrator = 0.0;
static double LPF_out_Yaw = 0.0;
static double lag_out_yaw = 0.0;
static double psi = 0.0;
static double yawIntegrator = 0.0;

/**
 * @brief Compute roll control command
 *
 * @param rollAngle_rad Current roll angle in radians
 * @param rollRate_radps Current roll rate in rad/s
 * @param dapParams DAP controller parameters
 * @param dt_s Time step in seconds
 * @return DAPOutput_t Roll control command
 */
static DAPOutput_t compute_delta_roll_command(double rollAngle_rad, double rollRate_radps,
                                              const DapParams_t *dapParams, double dt_s)
{
    double phiCommand = 0.0; /* Roll command is zero (maintain zero roll) */
    double phiErr = phiCommand - rollAngle_rad;
    double phiLimit;
    double out;
    double deltaCommandRoll;
    double integratorValue = rollIntegrator;
    DAPOutput_t result;

    /* Apply roll angle limits */
    if (phiErr < dapParams->phi_min_rad)
    {
        phiLimit = dapParams->phi_min_rad;
    }
    else if (phiErr > dapParams->phi_max_rad)
    {
        phiLimit = dapParams->phi_max_rad;
    }
    else
    {
        phiLimit = phiErr;
    }

    /* Calculate proportional term */
    out = phiLimit - (dapParams->kr_roll * rollRate_radps);

    /* Calculate final control command (Logic matching dap.c: Command first, then Update Integrator) */
    deltaCommandRoll = (dapParams->kp_roll * out) + rollIntegrator;

    /* Update integrator with anti-windup */
    integratorValue = integratorValue + ((out * dapParams->ki_roll) * dt_s);
    if (integratorValue > dapParams->integratorR_max_rad)
    {
        integratorValue = dapParams->integratorR_max_rad;
    }
    else if (integratorValue < dapParams->integratorR_min_rad)
    {
        integratorValue = dapParams->integratorR_min_rad;
    }
    rollIntegrator = integratorValue;

    if(fabs(deltaCommandRoll)< dapParams->actuator_deadband_rad)// actuator deadband
    {
        deltaCommandRoll = 0.0;
    }
    else
    {
        deltaCommandRoll = deltaCommandRoll;
    }

    if (deltaCommandRoll > dapParams->actuator_max_deflection_rad) // Actuator deflection limit +6 degrees
    {
        deltaCommandRoll = dapParams->actuator_max_deflection_rad;
    }
    else if (deltaCommandRoll < -dapParams->actuator_max_deflection_rad)
    {
        deltaCommandRoll = -dapParams->actuator_max_deflection_rad;
    }

    result.delta12_rad = deltaCommandRoll;
    result.delta3_rad = deltaCommandRoll;
    result.delta6_rad = deltaCommandRoll;
    result.delta9_rad = deltaCommandRoll;

    return result;
}

/**
 * @brief Compute pitch control command
 *
 * @param accelerationY_mps2 Current Y-axis acceleration in m/s^2
 * @param pitchRate_radps Current pitch rate in rad/s
 * @param accelerationYCommand_mps2 Commanded Y-axis acceleration in m/s^2
 * @param dapParams DAP controller parameters
 * @param dt_s Time step in seconds
 * @return PYOutput_t Pitch control command
 */
static PYOutput_t compute_delta_pitch_command(double accelerationY_mps2, double pitchRate_radps,
                                              double accelerationYCommand_mps2, const DapParams_t *dapParams, double dt_s)
{
    double accErrorPitch;
    double deltaCommandPitch;
    PYOutput_t result;
    double thetaLimit;

    /* Safety check for time step - prevent division by zero */
    double safe_timeStep = dt_s;
    if (safe_timeStep < MATH_EPSILON)
    {
        safe_timeStep = MATH_EPSILON;
    }

    accErrorPitch = accelerationYCommand_mps2 - accelerationY_mps2;
    double LPF_outdot = (accErrorPitch * dapParams->K_LPF_Pitch - LPF_out_Pitch) * dapParams->wC_pitch;
    double lag_outdot = LPF_outdot + (LPF_out_Pitch * dapParams->pitch_a) - (lag_out_pitch * dapParams->pitch_b);
    double thetaErr = lag_out_pitch - theta;

    if (thetaErr < dapParams->theta_min_rad)
    {
        thetaLimit = dapParams->theta_min_rad;
    }
    else if (thetaErr > dapParams->theta_max_rad)
    {
        thetaLimit = dapParams->theta_max_rad;
    }
    else
    {
        thetaLimit = thetaErr;
    }

    /* Calculate proportional term */
    double out_Pitch = thetaLimit - (dapParams->kr_pitch * pitchRate_radps);

    /* Calculate final control command */
    deltaCommandPitch = (dapParams->kp_pitch * out_Pitch) + pitchIntegrator;

    LPF_out_Pitch += LPF_outdot * safe_timeStep;
    lag_out_pitch += lag_outdot * safe_timeStep;
    theta += pitchRate_radps * safe_timeStep;

    /* Update integrator with anti-windup */
    double integratorValue = pitchIntegrator + ((out_Pitch * dapParams->ki_pitch) * dt_s);
    if (integratorValue > dapParams->integratorP_max_rad)
    {
        integratorValue = dapParams->integratorP_max_rad;
    }
    else if (integratorValue < dapParams->integratorP_min_rad)
    {
        integratorValue = dapParams->integratorP_min_rad;
    }
    pitchIntegrator = integratorValue;

    if(fabs(deltaCommandPitch)< dapParams->actuator_deadband_rad) // actuator deadband
    {
        deltaCommandPitch = 0.0;
    }
    else
    {
        deltaCommandPitch = deltaCommandPitch;
    }

    if (deltaCommandPitch > dapParams->actuator_max_deflection_rad)  // Actuator deflection limit +6 degrees
    {
        deltaCommandPitch = dapParams->actuator_max_deflection_rad;
    }
    else if (deltaCommandPitch < -dapParams->actuator_max_deflection_rad)
    {
        deltaCommandPitch = -dapParams->actuator_max_deflection_rad;
    }    

    result.delta1_rad = deltaCommandPitch;
    result.delta2_rad = deltaCommandPitch;
    return result;
}

/**
 * @brief Compute yaw control command
 *
 * @param accelerationZ_mps2 Current Z-axis acceleration in m/s^2
 * @param yawRate_radps Current yaw rate in rad/s
 * @param accelerationZCommand_mps2 Commanded Z-axis acceleration in m/s^2
 * @param dapParams DAP controller parameters
 * @param dt_s Time step in seconds
 * @return PYOutput_t Yaw control command
 */
static PYOutput_t compute_delta_yaw_command(double accelerationZ_mps2, double yawRate_radps,
                                            double accelerationZCommand_mps2, const DapParams_t *dapParams, double dt_s)
{
    double accErrorYaw;
    double deltaCommandYaw;
    PYOutput_t result;
    double psiLimit;

    /* Safety check for time step - prevent division by zero */
    double safe_timeStep = dt_s;
    if (safe_timeStep < MATH_EPSILON)
    {
        safe_timeStep = MATH_EPSILON;
    }

    accErrorYaw = accelerationZCommand_mps2 - accelerationZ_mps2;
    double LPF_outdot = (accErrorYaw * dapParams->K_LPF_Yaw - LPF_out_Yaw) * dapParams->wC_yaw;
    double lag_outdot = LPF_outdot + (LPF_out_Yaw * dapParams->yaw_a) - (lag_out_yaw * dapParams->yaw_b);
    double psiErr = lag_out_yaw - psi;

    if (psiErr < dapParams->psi_min_rad)
    {
        psiLimit = dapParams->psi_min_rad;
    }
    else if (psiErr > dapParams->psi_max_rad)
    {
        psiLimit = dapParams->psi_max_rad;
    }
    else
    {
        psiLimit = psiErr;
    }

    /* Calculate proportional term */
    double out_yaw = psiLimit - (dapParams->kr_yaw * yawRate_radps);

    /* Calculate final control command */
    deltaCommandYaw = (dapParams->kp_yaw * out_yaw) + yawIntegrator;

    LPF_out_Yaw += LPF_outdot * safe_timeStep;
    lag_out_yaw += lag_outdot * safe_timeStep;
    psi += yawRate_radps * safe_timeStep;

    /* Update integrator with anti-windup */
    double integratorValue = yawIntegrator + ((out_yaw * dapParams->ki_yaw) * dt_s);
    if (integratorValue > dapParams->integratorY_max_rad)
    {
        integratorValue = dapParams->integratorY_max_rad;
    }
    else if (integratorValue < dapParams->integratorY_min_rad)
    {
        integratorValue = dapParams->integratorY_min_rad;
    }
    yawIntegrator = integratorValue;

    if(fabs(deltaCommandYaw)< dapParams->actuator_deadband_rad)// actuator deadband
    {
        deltaCommandYaw = 0.0;
    }
    else
    {
        deltaCommandYaw = deltaCommandYaw;
    }

    if (deltaCommandYaw > dapParams->actuator_max_deflection_rad) // Actuator deflection limit +6 degrees
    {
        deltaCommandYaw = dapParams->actuator_max_deflection_rad;
    }
    else if (deltaCommandYaw < -dapParams->actuator_max_deflection_rad)
    {
        deltaCommandYaw = -dapParams->actuator_max_deflection_rad;
    }   

    result.delta1_rad = deltaCommandYaw;
    result.delta2_rad = deltaCommandYaw;
    return result;
}

/**
 * @brief Execute Digital Autopilot (DAP) algorithm
 *
 * This function executes the DAP control algorithm using:
 * - Guidance acceleration commands from major cycle (body frame)
 * - Current accelerometer measurements (Y and Z axes)
 * - Current angular rates from gyroscope
 * - Current roll angle from navigation
 * - DAP parameters from system state
 *
 * Guidance updates at 10Hz (major cycle), DAP executes at 100Hz (minor cycle).
 * DAP will use the same guidance command for 10 consecutive cycles until guidance updates.
 *
 * @return void
 */
static void execute_dap(double dt_s)
{
    /* Only execute DAP if canard control is enabled */
    if (!systemState.flags.canardControlFlag)
    {
        /* Canard control disabled - set zero actuator commands */
        systemState.actuatorCommands.ActuatorC3 = -0.024432809773124;
        systemState.actuatorCommands.ActuatorC6 = -0.024432809773124;
        systemState.actuatorCommands.ActuatorC9 = -0.024432809773124;
        systemState.actuatorCommands.ActuatorC12 = -0.024432809773124;
        return;
    }

    /* Read DAP parameters from system state */
    DapParams_t *dapParams = &systemState.dapParams;
    /* Handle guidance command latching based on guidStartFlag and canardControlFlag */
    if (systemState.flags.guidStartFlag)
    {
        dapParams->NavigationLastOutFlag = false;
        
    }
    else if ((!systemState.flags.guidStartFlag) && (systemState.flags.canardControlFlag))
    {
        /* Guidance command is zero - set nav last out as guidance command */
        if (!dapParams->NavigationLastOutFlag)
        {
            dapParams->latchedguidanceAccelY = systemState.accelerometerData.accel_current.y;
            dapParams->latchedguidanceAccelZ = systemState.accelerometerData.accel_current.z;
            dapParams->NavigationLastOutFlag = true;
        }
        systemState.guidanceState.accelCmdBody.y = dapParams->latchedguidanceAccelY;
        systemState.guidanceState.accelCmdBody.z = dapParams->latchedguidanceAccelZ;

    }
    /* Read guidance acceleration commands from DAP interface */
    /* Guidance updates these at 10Hz in major cycle */
    /* DAP Interface: Only Y (pitch) and Z (yaw) components needed */
    double accelerationYCommand = systemState.guidanceState.accelCmdBody.y; /* Pitch command */
    double accelerationZCommand = systemState.guidanceState.accelCmdBody.z; /* Yaw command */

    /* Read current accelerometer measurements (already processed and converted) */
    double accelerationY = systemState.accelerometerData.accel_current.y; /* Current pitch acceleration */
    double accelerationZ = systemState.accelerometerData.accel_current.z; /* Current yaw acceleration */

    /* Read current angular rates from gyroscope (already processed) */
    double rollRate = systemState.angularRates.x;  /* Roll rate (rad/s) */
    double pitchRate = systemState.angularRates.y; /* Pitch rate (rad/s) */
    double yawRate = systemState.angularRates.z;   /* Yaw rate (rad/s) */

    /* Read current roll angle from navigation */
    double rollAngle = systemState.navigationState.attitude_e.roll_rad; /* Roll angle (rad) */

    /* Execute DAP algorithm - canard control is enabled if we reach here */
    DAPOutput_t dapOutput;
    DAPOutput_t deltar = {0.0, 0.0, 0.0, 0.0};
    PYOutput_t deltap = {0.0, 0.0};
    PYOutput_t deltay = {0.0, 0.0};

    if (systemState.rollRateFp > 0.01)
    {
        /* Roll rate is non-zero - use roll control */
        deltar = compute_delta_roll_command(rollAngle, rollRate, dapParams, dt_s);
        dapOutput = deltar;
    }
    else
    {
        /* Roll rate is zero - use pitch and yaw control */
        deltap = compute_delta_pitch_command(accelerationY, pitchRate,
                                             accelerationYCommand, dapParams, dt_s);
        deltay = compute_delta_yaw_command(accelerationZ, yawRate,
                                           accelerationZCommand, dapParams, dt_s);

        dapOutput.delta12_rad = deltay.delta2_rad;
        dapOutput.delta3_rad = deltap.delta1_rad;
        dapOutput.delta6_rad = deltay.delta1_rad;
        dapOutput.delta9_rad = deltap.delta2_rad;
    }

    /* Map DAP output to actuator commands */
    /* Note: Actual canard mapping depends on hardware configuration */
    /* For now, storing individual canard deflections in actuatorCommands */
    systemState.actuatorCommands.ActuatorC12 = dapOutput.delta12_rad; /* Canard 12 */
    systemState.actuatorCommands.ActuatorC3 = dapOutput.delta3_rad;   /* Canard 3 */
    systemState.actuatorCommands.ActuatorC6 = dapOutput.delta6_rad;   /* Canard 6 */
    systemState.actuatorCommands.ActuatorC9 = dapOutput.delta9_rad;   /* Canard 9 */
}

/* ===== PUBLIC FUNCTIONS ===== */

/**
 * @brief Set OBC reset status
 *
 * This function sets the OBC reset flag and initializes sequencer state.
 * Called when OBC reset flag becomes true (not G-switch).
 *
 * @param isActive true to activate OBC reset, false to deactivate
 * @return Status_t Success or error code
 */
Status_t set_obc_reset(bool isActive)
{
    /* Initialize sequencer state first */
    sequencer_init(&systemState.sequencerState);

    /* Set OBC reset state for sequencer */
    sequencer_set_obc_reset(&systemState.sequencerState, isActive);

    if (isActive)
    {
        /* Reset the OBC */
        systemState.timeFromObcReset = 0U;

        /* Set T0 phase immediately when OBC reset activates */
        systemState.flags.isT0Set = true;

        /* Load PEFCS default parameters into system state */
        load_pefcs_defaults();

        /* Initialize dynamic sensor data (Zeroing measurements) */
        memset(&systemState.accelerometerData, 0, sizeof(AccelerometerData_t));
        memset(&systemState.gyroscopeData, 0, sizeof(GyroscopeData_t));
        memset(&systemState.magnetometerData, 0, sizeof(MagnetometerData_t));
        memset(&systemState.incrementalVelocityData, 0, sizeof(IncrementalVelocityData_t));
        memset(&systemState.incrementalAngleData, 0, sizeof(IncrementalAngleData_t));

        /* Note: navigationState.accel_offset and mag_ref_ned are preserved 
           because they were JUST loaded by load_pefcs_defaults() above. */

        /* Initialize navigation confirmation flags and counters */
        systemState.navigationState.mag_3_cycles_confirmed = false;
        systemState.navigationState.gyro_3_cycles_confirmed = false;
        magcount = 0;
        MAG_3_CYCLES_CONFIRMED = false;
        gyrocount = 0;
        gyro_3_cycles_confirmed = false;
        gyro_rate_condition_met = false;
        magEstOkFlag = false;
        gyroattitude = false;
        rate = 0.0;

        /* Initialize DAP state variables (Integrators and Filters) */
        rollIntegrator = 0.0;
        pitchIntegrator = 0.0;
        yawIntegrator = 0.0;
        LPF_out_Pitch = 0.0;
        lag_out_pitch = 0.0;
        theta = 0.0;
        LPF_out_Yaw = 0.0;
        lag_out_yaw = 0.0;
        psi = 0.0;

        /* Initialize Actuator Commands to Neutral/Bias */
        systemState.actuatorCommands.ActuatorC3 = -0.024432809773124;
        systemState.actuatorCommands.ActuatorC6 = -0.024432809773124;
        systemState.actuatorCommands.ActuatorC9 = -0.024432809773124;
        systemState.actuatorCommands.ActuatorC12 = -0.024432809773124;

        /* Reset Guidance Persistence State */
        systemState.guidanceState.prevAccel.x = 0.0;
        systemState.guidanceState.prevAccel.y = 0.0;
        systemState.guidanceState.prevAccel.z = 0.0;
    }

    return SUCCESS;
}

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
Status_t set_mag_ref_ned_from_checkout(const Vector3_t *mag_ref_ned) // place holder function aaneea, ningalk istham ullath pole maatiko,
// Checkoutine varunna Vector3 ee varibales pass cheythal mathi.
{
    /* Placeholder implementation for checkout system */
    /* Hardware/checkout team should implement actual interface here */
    /* TODO: Hardware team - implement actual checkout system interface */
    /* Example interface:
     * checkout_set_mag_ref_ned(mag_ref_ned->x,
     *                         mag_ref_ned->y,
     *                         mag_ref_ned->z);
     */

    /* Store NED frame magnetic field reference in system state */
    /* These values will be transformed using the transformation matrix in process_navigation() */
    systemState.navigationState.mag_ref_ned.x = mag_ref_ned->x;
    systemState.navigationState.mag_ref_ned.y = mag_ref_ned->y;
    systemState.navigationState.mag_ref_ned.z = mag_ref_ned->z;

    return SUCCESS;
}

/**
 * @brief Minor cycle function - Called every 10ms (100Hz)
 *
 * This is the main entry point for the minor cycle processing.
 * It executes all 100Hz tasks in the required sequence.
 */
void minor_cycle(void)
{
    double dt_s = INTEGRATION_STEP_SIZE;

    /* Hardware sensor processing - ONLY for real hardware mode */
    if (!systemState.testMode)
    {
        /* 0. Sensor health check */
        check_sensor_health();

        /* 1. Accelerometer data processing */
        process_accelerometer_data();

        /* 2. Gyroscope data processing */
        process_gyroscope_data();

        /* 3. Magnetometer data processing */
        process_magnetometer_data();

        /* 4. Incremental velocity data processing */
        process_incremental_velocity_data();

        /* 5. Incremental angle data processing */
        process_incremental_angle_data();
    }
    else
    {
        /* Test mode: Data is directly injected by test harness */
        /* Compute derived values that are always needed */

        /* Calculate angular rates for DAP (rad/s) */
        systemState.angularRates.x = systemState.gyroscopeData.gyro_current.x; /* Roll rate */
        systemState.angularRates.y = systemState.gyroscopeData.gyro_current.y; /* Pitch rate */
        systemState.angularRates.z = systemState.gyroscopeData.gyro_current.z; /* Yaw rate */

        /* Calculate roll rate in rps (revolutions per second) for sequencer */
        /* Convert from rad/s to rps: rps = (rad/s) / (2*PI) */
        double roll_rate_rad_s = systemState.gyroscopeData.gyro_current.x;
        systemState.rollRateFp = fabs(roll_rate_rad_s / (2.0 * MATH_PI));
    }

    /* 6. Navigation processing (ALWAYS runs) */
    process_navigation(dt_s);

    /* 7. DAP processing - Execute Digital Autopilot (ALWAYS runs) */
    /* DAP reads guidance commands from systemState.guidanceState.accelCmdBody.y and accelCmdBody.z */
    /* Guidance updates at 10Hz (major cycle), DAP executes at 100Hz (minor cycle) */
    /* DAP will use the same guidance command for 10 consecutive cycles until guidance updates */
    execute_dap(dt_s);

    /* 8. Sequencer processing - Execute Sequencer (ALWAYS runs) */
    /* Sequencer reads rollRateFp from systemState.rollRateFp (in rps) */
    /* Sequencer reads timeToGo from systemState.guidanceState.timeToGo (in seconds, double) */
    /* Sequencer uses timeToGo for T3 phase transition logic (t_go < proximity threshold) */
    /* Sequencer outputs are publicly accessible via systemState.flags.* */
    execute_sequencer(dt_s);

    /* 9. Update timing information */
    systemState.minorCycleCount++;
}