/******************************************************************************
* ISA Flight Software
* @file nav_test_standalone.c
* @brief Navigation Software Testing with CSV Input/Output (Standalone)
* @details Standalone test harness for navigation attitude estimation
* @author Integration Team, Spacelabs
* @date 2025
* @version 1.2
*****************************************************************************/
//To compile // cd /home/spacelabs/Devolop/isa-obc-flight-software-embedded-c && gcc nav_test_standalone.c -lm -o nav_test
//To run //  cd /home/spacelabs/Devolop/isa-obc-flight-software-embedded-c && ./nav_test /navout.csv test_output.csv

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Minimal type definitions needed for standalone testing
typedef struct {
    double x;
    double y;
    double z;
} Vector3_t;

typedef struct {
    double roll_rad;
    double pitch_rad;
    double yaw_rad;
} EulerAngles_t;

typedef struct {
    Vector3_t accel;
    Vector3_t gyro;
    Vector3_t mag;
} ImuData_t;

typedef struct {
    Vector3_t gyro_current;
} GyroscopeData_t;

typedef struct {
    Vector3_t delta_theta_raw;
} IncrementalAngleData_t;

typedef struct {
    Vector3_t attitude_rad;
} AttitudeTelemetryData_t;

typedef struct {
    EulerAngles_t attitude_e;
} NavigationState_t;

typedef struct {
    NavigationState_t navigationState;
    GyroscopeData_t gyroscopeData;
    IncrementalAngleData_t incrementalAngleData;
    AttitudeTelemetryData_t attitudeTelemetryData;
    ImuData_t imuData;
    double rollRateFp;
    uint32_t minorCycleCount;
} SystemState_t;

// Constants
#define MATH_PI 3.14159265358979323846
#define DEG2RAD 0.017453292519943295769236907684886
//#define MATH_EPSILON 1.0e-6f

// Global system state
SystemState_t systemState;

// Global variables for attitude estimation tracking
// Magnetometer attitude estimation flags (3 consecutive cycles check)
int magcount = 0;
bool mag_rate_condition_met = false;
bool mag_3_cycles_confirmed = false;

// Gyroscope attitude integration flags (3 consecutive cycles check)
int gyrocount = 0;
bool gyro_rate_condition_met = false;
bool gyro_3_cycles_confirmed = false;
bool gyroattitude = false;
double dt = 0.01;
double rate = 0.0;
double phi =  0.0;
double theta = 0.0; 
double psi = 0.0;
double mn = 0.0;
double mv = 0.0;
double me = 0.0;

// TO BE INITIALIZED FROM PEFCS
double Azimuth = 0.0;  // Yaw angle (should be initialized from PEFCS)
double Elevation = 25.0; // Pitch angle (should be initialized from PEFCS)
double m_n = 407.18455450466178; // Magnetic field reference value X in mG (should be initialized from PEFCS)
double m_e = -12.541435991209887; // Magnetic field reference value Y in mG (should be initialized from PEFCS)
double m_d =  28.342885818553878; // Magnetic field reference value Z in mG (should be initialized from PEFCS)
double MAG_HARD_IRON_X = 0.0;
double MAG_HARD_IRON_Y = 0.0;
double MAG_HARD_IRON_Z = 0.0;
double MAG_SOFT_IRON_XX = 1.0;
double MAG_SOFT_IRON_XY = 0.0;
double MAG_SOFT_IRON_XZ = 0.0;
double MAG_SOFT_IRON_YX = 0.0;
double MAG_SOFT_IRON_YY = 1.0;
double MAG_SOFT_IRON_YZ = 0.0;
double MAG_SOFT_IRON_ZX = 0.0;
double MAG_SOFT_IRON_ZY = 0.0;
double MAG_SOFT_IRON_ZZ = 1.0;
Vector3_t gyro_offset = {0.0, 0.0, 0.0};
Vector3_t accel_offset = {0.0, 0.0, 0.0};


// Math utility function
void mod_double(double* result, double a, double d) {
    double m = fmodf(a, d);
    if ((m < 0.0 && d > 0.0) || (m > 0.0 && d < 0.0))
    {
        m += d;
    }
    *result = m;
}

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
 * @brief Placeholder function for flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality.
 *
 * @param data Pointer to accelerometer data to store
 * @return void
 */
void store_accelerometer_to_flash(const Vector3_t* data)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */
    // ASK ANANTHU

    if (data == NULL) {
        return;
    }

    /* TODO: Hardware team - implement actual flash memory write */
    /* Example interface:
     * flash_write_accelerometer_data(data->x, data->y, data->z);
     */
}

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
    /* Store raw accelerometer data */
    systemState.accelerometerData.accel_raw = systemState.imuData.accel;

    /* Check individual accelerometer axis health */
    bool accel_x_ok = ((systemState.navigationState.sensorHealth & SENSOR_ACCEL_X_IS_VALID) == 0U);
    bool accel_y_ok = ((systemState.navigationState.sensorHealth & SENSOR_ACCEL_Y_IS_VALID) == 0U);
    bool accel_z_ok = ((systemState.navigationState.sensorHealth & SENSOR_ACCEL_Z_IS_VALID) == 0U);

    /* Process each axis individually based on health status */

    /* X-axis processing */
    if (accel_x_ok) {
        /* X-axis is healthy - convert current data */
        systemState.accelerometerData.accel_current.x = (systemState.accelerometerData.accel_raw.x * FT_TO_M_S2_CONVERSION_FACTOR) - systemState.navigationState.accel_offset.x;
        /* Update previous value for future use */
        systemState.accelerometerData.accel_previous.x = systemState.accelerometerData.accel_current.x;
    }
    else {
        /* X-axis is unhealthy - use previous data */
        systemState.accelerometerData.accel_current.x = systemState.accelerometerData.accel_previous.x;
    }

    /* Y-axis processing */
    if (accel_y_ok) {
        /* Y-axis is healthy - convert current data */
        systemState.accelerometerData.accel_current.y = (systemState.accelerometerData.accel_raw.y * FT_TO_M_S2_CONVERSION_FACTOR) - systemState.navigationState.accel_offset.y;
        /* Update previous value for future use */
        systemState.accelerometerData.accel_previous.y = systemState.accelerometerData.accel_current.y;
    }
    else {
        /* Y-axis is unhealthy - use previous converted data */
        systemState.accelerometerData.accel_current.y = systemState.accelerometerData.accel_previous.y;
    }

    /* Z-axis processing */
    if (accel_z_ok) {
        /* Z-axis is healthy - convert current data */
        systemState.accelerometerData.accel_current.z = (systemState.accelerometerData.accel_raw.z * FT_TO_M_S2_CONVERSION_FACTOR) - systemState.navigationState.accel_offset.z ;
        /* Update previous value for future use */
        systemState.accelerometerData.accel_previous.z = systemState.accelerometerData.accel_current.z;
    }
    else {
        /* Z-axis is unhealthy - use previous converted data */
        systemState.accelerometerData.accel_current.z = systemState.accelerometerData.accel_previous.z;
    }


    /* Store processed data to flash memory */
    store_accelerometer_to_flash(&systemState.accelerometerData.accel_current);
    // pass the values to DAP - TO DO
    // pass the values Ay, Az only to dap (Ax not needed)



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

/* ===== GYROSCOPE DATA PROCESSING ===== */

/**
 * @brief Placeholder function for gyroscope flash memory write
 *
 * This function is a placeholder for the hardware team to implement
 * actual flash memory write functionality for gyroscope data.
 *
 * @param data Pointer to gyroscope data to store
 * @return void
 */
void store_gyroscope_to_flash(const Vector3_t* data)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (data == NULL) {
        return;
    }

    /* TODO: Hardware team - implement actual flash memory write */
    /* Example interface:
     * flash_write_gyroscope_data(data->x, data->y, data->z);
     */
}

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
    /* Store raw gyroscope data */
    systemState.gyroscopeData.gyro_raw = systemState.imuData.gyro;

    /* Check individual gyroscope axis health */
    bool gyro_x_ok = ((systemState.navigationState.sensorHealth & SENSOR_GYRO_X_IS_VALID) == 0U);
    bool gyro_y_ok = ((systemState.navigationState.sensorHealth & SENSOR_GYRO_Y_IS_VALID) == 0U);
    bool gyro_z_ok = ((systemState.navigationState.sensorHealth & SENSOR_GYRO_Z_IS_VALID) == 0U);

    /* Process each axis individually based on health status */

    /* X-axis processing */
    if (gyro_x_ok) {
        /* X-axis is healthy - use current data */
        systemState.gyroscopeData.gyro_current.x = systemState.gyroscopeData.gyro_raw.x - systemState.navigationState.gyro_offset.x;
        /* Update previous value for future use */
        systemState.gyroscopeData.gyro_previous.x = systemState.gyroscopeData.gyro_current.x;
    }
    else {
        /* X-axis is unhealthy - use previous data */
        systemState.gyroscopeData.gyro_current.x = systemState.gyroscopeData.gyro_previous.x;
    }

    /* Y-axis processing */
    if (gyro_y_ok) {
        /* Y-axis is healthy - use current data */
        systemState.gyroscopeData.gyro_current.y = systemState.gyroscopeData.gyro_raw.y - systemState.navigationState.gyro_offset.y;
        /* Update previous value for future use */
        systemState.gyroscopeData.gyro_previous.y = systemState.gyroscopeData.gyro_current.y;
    }
    else {
        /* Y-axis is unhealthy - use previous converted data */
        systemState.gyroscopeData.gyro_current.y = systemState.gyroscopeData.gyro_previous.y;
    }

    /* Z-axis processing */
    if (gyro_z_ok) {
        /* Z-axis is healthy - use current data */
        systemState.gyroscopeData.gyro_current.z = systemState.gyroscopeData.gyro_raw.z - systemState.navigationState.gyro_offset.z;
        /* Update previous value for future use */
        systemState.gyroscopeData.gyro_previous.z = systemState.gyroscopeData.gyro_current.z;
    }
    else {
        /* Z-axis is unhealthy - use previous converted data */
        systemState.gyroscopeData.gyro_current.z = systemState.gyroscopeData.gyro_previous.z;
    }

    //systemState.gyroscopeData.gyro_current.x = systemState.gyroscopeData.gyro_current.x - systemState.navigationState.gyro_offset.x;
   // systemState.gyroscopeData.gyro_current.y = systemState.gyroscopeData.gyro_current.y - systemState.navigationState.gyro_offset.y;
   // systemState.gyroscopeData.gyro_current.z = systemState.gyroscopeData.gyro_current.z - systemState.navigationState.gyro_offset.z;

    // TO Do - pass the values Gx, Gy, Gz only to dap.
// pass ther Gx to the sequencer.


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
void store_magnetometer_to_flash(const Vector3_t* data)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (data == NULL) {
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
    /* Store raw magnetometer data */
    systemState.magnetometerData.mag_raw = systemState.imuData.mag;

    /* Check overall magnetometer health (single flag, not individual axes) */
    bool mag_ok = ((systemState.navigationState.sensorHealth & SENSOR_MAG_IS_VALID) == 0U);

    /* Process magnetometer data based on health status */
    if (mag_ok) {
        /* Magnetometer is healthy - use current data */
        systemState.magnetometerData.mag_current = systemState.magnetometerData.mag_raw;
        /* Update previous value for future use */
        systemState.magnetometerData.mag_previous = systemState.magnetometerData.mag_current;
    }
    else {
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
void store_incremental_velocity_to_flash(const Vector3_t* delta_v_converted, const Vector3_t* acceleration)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (delta_v_converted == NULL || acceleration == NULL) {
        return;
    }

    /* TODO: Hardware team - implement actual flash memory write */
    /* Example interface:
     * flash_write_delta_v_converted(delta_v_converted->x, delta_v_converted->y, delta_v_converted->z);
     * flash_write_acceleration(acceleration->x, acceleration->y, acceleration->z);
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
    /* Store raw incremental velocity data (ft/s) */
    /* TODO: Hardware team - implement actual hardware interface to read delta_v from REX */
    /* For now, using placeholder values */
    systemState.incrementalVelocityData.delta_v_raw.x = 0.0;  /* Placeholder - read from REX */
    systemState.incrementalVelocityData.delta_v_raw.y = 0.0;  /* Placeholder - read from REX */
    systemState.incrementalVelocityData.delta_v_raw.z = 0.0;  /* Placeholder - read from REX */

    /* Convert units from ft/s to m/s */
    systemState.incrementalVelocityData.delta_v_converted.x =
        systemState.incrementalVelocityData.delta_v_raw.x * FT_TO_M_S2_CONVERSION_FACTOR;
    systemState.incrementalVelocityData.delta_v_converted.y =
        systemState.incrementalVelocityData.delta_v_raw.y * FT_TO_M_S2_CONVERSION_FACTOR;
    systemState.incrementalVelocityData.delta_v_converted.z =
        systemState.incrementalVelocityData.delta_v_raw.z * FT_TO_M_S2_CONVERSION_FACTOR;

    /* Store converted velocity to flash memory */
    /* Note: acceleration parameter removed as it's not calculated in this function */
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
void store_incremental_angle_to_flash(const Vector3_t* data)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (data == NULL) {
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
    /* Store raw incremental angle data (rad) */
    /* TODO: Hardware team - implement actual hardware interface to read delta_theta from REX */
    /* For now, using placeholder values */
    systemState.incrementalAngleData.delta_theta_raw.x = 0.0;  /* Placeholder - read from REX */
    systemState.incrementalAngleData.delta_theta_raw.y = 0.0;  /* Placeholder - read from REX */
    systemState.incrementalAngleData.delta_theta_raw.z = 0.0;  /* Placeholder - read from REX */

    /* Store raw data directly to flash memory (no processing needed) */
    store_incremental_angle_to_flash(&systemState.incrementalAngleData.delta_theta_raw);
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
void store_attitude_to_flash(const Vector3_t* data)
{
    /* Placeholder implementation for hardware team */
    /* Hardware team should implement actual flash memory write here */

    if (data == NULL) {
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
      mag_3_cycles_confirmed = systemState.navigationState.mag_3_cycles_confirmed;
      gyro_3_cycles_confirmed = systemState.navigationState.gyro_3_cycles_confirmed;
 
     /* TODO: Hardware team - implement actual telemetry transmission */
     /* Example interface:
      * telemetry_send_nav_confirmation_flags(mag_3_cycles_confirmed, gyro_3_cycles_confirmed);
      */
 
      /* For now, just store in system state for other modules to access */
      /* Telemetry system can read these flags from systemState.navigationState */
 }


static void magnetometer_attitude_estimation(void)
{
 

    // Read magnetometer data from system state (no calibration)
    //Vector3_t mag_raw = systemState.imuData.mag;
     // Magnetormeter Calibratiion : ((magx.magy,magz)) - HI) * SI

    Vector3_t mag_obc = systemState.imuData.mag_current;
    Vector3_t mag_calibrated;

    // Step 1 : Subtract hard iron offset (HI)
    Vector3_t mag_offset_corrected;
    mag_offset_corrected.x = mag_obc.x - MAG_HARD_IRON_X;
    mag_offset_corrected.y = mag_obc.y - MAG_HARD_IRON_Y;
    mag_offset_corrected.z = mag_obc.z - MAG_HARD_IRON_Z;

    // Step 2: Apply soft iron correction matrix (SI)
    mag_calibrated.x = MAG_SOFT_IRON_XX * mag_offset_corrected.x + MAG_SOFT_IRON_XY * mag_offset_corrected.y + MAG_SOFT_IRON_XZ * mag_offset_corrected.z;

    mag_calibrated.y = MAG_SOFT_IRON_YX * mag_offset_corrected.x + MAG_SOFT_IRON_YY * mag_offset_corrected.y + MAG_SOFT_IRON_YZ * mag_offset_corrected.z;

    mag_calibrated.z = MAG_SOFT_IRON_ZX * mag_offset_corrected.x + MAG_SOFT_IRON_ZY * mag_offset_corrected.y + MAG_SOFT_IRON_ZZ * mag_offset_corrected.z;




    // Roll estimation using raw magnetic field measurements
    double phi1 = atan2f((sinf(theta) * cosf(psi) * mn + cosf(theta) * mv + sinf(theta) * sinf(psi) * me) * mag_calibrated.z - (-sinf(psi) * mn + cosf(psi) * me) * mag_calibrated.y, (sinf(theta) * cosf(psi) * mn + cosf(theta) * mv + sinf(theta) * sinf(psi) * me) * mag_calibrated.y + (-sinf(psi) * mn + cosf(psi) * me) * mag_calibrated.z);

    // Wrap roll angle to 2π
    //double phi;
    mod_double(&phi, phi1, 2.0 * MATH_PI);

    // Update navigation state with estimated attitude
    systemState.navigationState.attitude_e.roll_rad = phi;
    systemState.navigationState.attitude_e.pitch_rad = theta;
    systemState.navigationState.attitude_e.yaw_rad = psi;

    // Save attitude values to telemetry memory
    systemState.attitudeTelemetryData.attitude_rad.x = phi;   // Roll
    systemState.attitudeTelemetryData.attitude_rad.y = psi; // Yaw
    systemState.attitudeTelemetryData.attitude_rad.z = theta;   // Pitch
    // Send attitude data to telemetry
    send_attitude_to_telemetry();
}

void gyroscope_attitude_integration(void) {
    // Time step (10ms cycle time)

    
    // Read current attitude
    double phi = systemState.navigationState.attitude_e.roll_rad;
    double theta = systemState.navigationState.attitude_e.pitch_rad;
    double psi = systemState.navigationState.attitude_e.yaw_rad;

    // Map body rates from incremental angle data
    // p, q, r are the body angular rates (rad/s)
    double p = systemState.incrementalAngleData.delta_theta_raw.x / dt;
    double q = systemState.incrementalAngleData.delta_theta_raw.y / dt;
    double r = systemState.incrementalAngleData.delta_theta_raw.z / dt;

    // Calculate Euler angle rates using kinematic equations
    // These transform body rates to Euler angle rates
    double psi_dot   = (q * cosf(phi) + r * sinf(phi)) / cosf(theta);
    double theta_dot = r * cosf(phi) - q * sinf(phi);
    double phi_dot   = p + (q * cosf(phi) + r * sinf(phi)) * tanf(theta);

    // Integrate Euler angle rates to get new attitude
    phi   = phi   + phi_dot   * dt;
    theta = theta + theta_dot * dt;
    psi   = psi   + psi_dot   * dt;

    // Wrap using mod_double:
    mod_double(&phi,   phi,   2.0 * MATH_PI);
    mod_double(&theta, theta, 2.0 * MATH_PI);
    mod_double(&psi,   psi,   2.0 * MATH_PI);

    // Store integrated attitude for DAP/telemetry
    systemState.attitudeTelemetryData.attitude_rad.x = phi;
    systemState.attitudeTelemetryData.attitude_rad.y = psi;
    systemState.attitudeTelemetryData.attitude_rad.z = theta;
    systemState.navigationState.attitude_e.roll_rad = phi;
    systemState.navigationState.attitude_e.pitch_rad = theta;
    systemState.navigationState.attitude_e.yaw_rad = psi;
    // Store/telemetry as before
    //store_attitude_to_flash(&systemState.attitudeTelemetryData.attitude_rad);
    send_attitude_to_telemetry();
}

void minor_cycle(void)
{
    

    /* Read roll rate from gyroscope data */
   // roll_rate_x = systemState.gyroscopeData.gyro_current.x;
    rate = fabsf(systemState.gyroscopeData.gyro_current.x /(2.0 * MATH_PI));
    //systemState.rollRateFp = rate;

    /* ===== MAGNETOMETER ATTITUDE ESTIMATION - 3 CONSECUTIVE CYCLES CHECK ===== */
    /* Check 1: Rate condition for magnetometer (rate <= 5 rps and gyro integration not active) */
   
    if (rate <= 5.0 && !mag_3_cycles_confirmed) 
    {
        mag_rate_condition_met = true;
        magcount++;
       // printf("Magnetometer rate condition met \n");
    }


if (mag_rate_condition_met && !gyroattitude && magcount >= 3)
{
    mag_3_cycles_confirmed = true;
   // printf("Magnetometer 3 cycles confirmed \n");
    magnetometer_attitude_estimation();
}

if (rate <= 2.0 && !gyro_3_cycles_confirmed) 
{
    gyro_rate_condition_met = true;
   // printf("Gyroscope rate condition met \n");
    gyrocount++;
}

if (gyro_rate_condition_met && gyrocount >= 3)
{
    gyro_3_cycles_confirmed = true;
   // printf("Gyroscope 3 cycles confirmed \n");
    gyroscope_attitude_integration();
    gyroattitude = true;
}


    /* Update timing information */
    systemState.minorCycleCount++;
}

/**
 * @brief Initialize system state for testing
 */
void init_system_state(void) {
    memset(&systemState, 0, sizeof(SystemState_t));
    mn = m_n;
    mv = -m_d;
    me = m_e;
    theta = DEG2RAD * Elevation; // Pitch angle (should be initialized from PEFCS)
    psi = DEG2RAD * Azimuth;// Yaw angle (should be initialized from PEFCS)
    // Initialize attitude with computed initial values (from global phi, theta, psi)
    // This ensures CSV output doesn't start with zeros before magnetometer estimation
    systemState.navigationState.attitude_e.roll_rad = phi;
    systemState.navigationState.attitude_e.pitch_rad = theta;
    systemState.navigationState.attitude_e.yaw_rad = psi;
    
    // Save attitude values to telemetry memory
    systemState.attitudeTelemetryData.attitude_rad.x = phi;   // Roll
    systemState.attitudeTelemetryData.attitude_rad.y = psi;   // Yaw
    systemState.attitudeTelemetryData.attitude_rad.z = theta; // Pitch
    
    // Reset counters and flags
    magcount = 0;
    mag_rate_condition_met = false;
    mag_3_cycles_confirmed = false;
    
    gyrocount = 0;
    gyro_rate_condition_met = false;
    gyro_3_cycles_confirmed = false;
    gyroattitude = false;
}

// /**
//  * @brief Read CSV file and process each row
//  */
// int process_csv(const char* input_file, const char* output_file) {
//     FILE* fin = fopen(input_file, "r");
//     if (!fin) {
//         printf("Error: Cannot open input file %s\n", input_file);
//         return -1;
//     }
    
//     FILE* fout = fopen(output_file, "w");
//     if (!fout) {
//         printf("Error: Cannot open output file %s\n", output_file);
//         fclose(fin);
//         return -1;
//     }
    
//     char line[1024];
//     int line_num = 0;
    
//     // Read and skip header
//     if (fgets(line, sizeof(line), fin) == NULL) {
//         printf("Error: Empty input file\n");
//         fclose(fin);
//         fclose(fout);
//         return -1;
//     }
    
//     // Write output header
//     fprintf(fout, "missiontime,bmx_input,bmy_input,bmz_input,Gyro_p_input,");
//     fprintf(fout, "roll_rad,yaw_rad,pitch_rad,roll_rate_used,");
//     fprintf(fout, "mag_rate_condition_met,magcount,mag_3_cycles_confirmed,");
//     fprintf(fout, "gyro_rate_condition_met,gyrocount,gyro_3_cycles_confirmed,gyroattitude_active\n");
    
//     // Process each data row
//     while (fgets(line, sizeof(line), fin) != NULL) {
//         line_num++;
        
//         // Parse CSV line
//         double missiontime, Pos_X, Pos_Y, Pos_Z, Vel_X, Vel_Y, Vel_Z;
//         double Acc_X, Acc_Y, Acc_Z, Gyro_p, Gyro_q, Gyro_r;
//         double Roll, Pitch, Yaw,  rollRPS, bmx, bmy, bmz;
//         double deltaVx, deltaVy, deltaVz, deltaThetaX, deltaThetaY, deltaThetaZ;
        
//         int parsed = sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
//             &missiontime, &Pos_X, &Pos_Y, &Pos_Z, &Vel_X, &Vel_Y, &Vel_Z,
//             &Acc_X, &Acc_Y, &Acc_Z, &Gyro_p, &Gyro_q, &Gyro_r,
//             &Roll, &Pitch, &Yaw, &rollRPS, &bmx, &bmy, &bmz, &deltaVx, &deltaVy, &deltaVz,
//             &deltaThetaX, &deltaThetaY, &deltaThetaZ);
        
//         if (parsed != 26) {
//             printf("Warning: Line %d has incorrect format (parsed %d fields)\n", line_num, parsed);
//             continue;
//         }
        
//         // Gyro_p is already in rad/s - use directly
//         systemState.gyroscopeData.gyro_current.x = Gyro_p;
//         systemState.gyroscopeData.gyro_current.y = Gyro_q;
//         systemState.gyroscopeData.gyro_current.z = Gyro_r;
        
//         systemState.imuData.gyro.x = Gyro_p;
//         systemState.imuData.gyro.y = Gyro_q;
//         systemState.imuData.gyro.z = Gyro_r;
        
//         // Magnetometer data (bmx, bmy, bmz are in mG - use directly without calibration)
//         systemState.imuData.mag.x = bmx;
//         systemState.imuData.mag.y = bmy;
//         systemState.imuData.mag.z = bmz;
        
//         // Set incremental angle data (small delta for integration)
//         systemState.incrementalAngleData.delta_theta_raw.x = deltaThetaX ; // dt = 0.01s
//         systemState.incrementalAngleData.delta_theta_raw.y = deltaThetaY ;
//         systemState.incrementalAngleData.delta_theta_raw.z = deltaThetaZ ;
        
//         // Run the minor cycle
//         minor_cycle();
        
//         // Write output to CSV
//         fprintf(fout, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%d\n",
//             missiontime,
//             bmx, bmy, bmz,
//             Gyro_p,
//             systemState.attitudeTelemetryData.attitude_rad.x,
//             systemState.attitudeTelemetryData.attitude_rad.y,
//             systemState.attitudeTelemetryData.attitude_rad.z,
//             systemState.rollRateFp,
//             mag_rate_condition_met ? 1 : 0,
//             magcount,
//             mag_3_cycles_confirmed ? 1 : 0,
//             gyro_rate_condition_met ? 1 : 0,
//             gyrocount,
//             gyro_3_cycles_confirmed ? 1 : 0,
//             gyroattitude ? 1 : 0);
//     }
    
//     fclose(fin);
//     fclose(fout);
    
//    // printf("Processing complete: %d lines processed\n", line_num);
//     return 0;
// }

// /**
//  * @brief Main function for standalone testing
//  */
// int main(int argc, char* argv[]) {
//     const char* input_file = "src/navout.csv";
//     const char* output_file = "test_output.csv";
    
//     // Allow command-line arguments to override file paths
//     if (argc >= 2) {
//         input_file = argv[1];
//     }
//     if (argc >= 3) {
//         output_file = argv[2];
//     }
    
//     //printf("Navigation Software Testing\n");
//    // printf("Input file:  %s\n", input_file);
//    // printf("Output file: %s\n", output_file);
    
//     // Initialize system state
//     init_system_state();
    
//     // Process CSV file
//     int result = process_csv(input_file, output_file);
    
//     // if (result == 0) {
//     //    // printf("Test completed successfully!\n");
//     // } else {
//     //    // printf("Test failed with error code %d\n", result);
//     // }
    
//     return result;
// }

