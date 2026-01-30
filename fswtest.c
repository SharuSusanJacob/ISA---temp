/******************************************************************************
* ISA Flight Software - CSV Test Harness
* @file fswtest.c  
* @brief Open-loop test harness for flight software verification
* @details Injects sensor data from CSV directly into systemState variables,
*          executes FSW logic, and logs comprehensive outputs
* @author Ananthu Dev - Project Engineer, Spacelabs
* @date 2025
* @version 1.0
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "system_state.h"
#include "major.h"
#include "minor.h"
#include "math_utils.h"

// Global system state (extern declared in system_state.h)
extern SystemState_t systemState;

// CSV line buffer
#define LINE_BUFFER_SIZE 1024

/* ===== FLASH STORAGE STUB ===== */
/* store_gyroscope_to_flash is called by FSW but not defined */
void store_gyroscope_to_flash(const Vector3_t *gyro_data)
{
    (void)gyro_data; /* Suppress unused parameter warning */
}

/**
 * @brief Inject sensor data from CSV into systemState
 * 
 * This function bypasses all hardware-specific processing (LSB conversion,
 * health checks) and injects data directly into the algorithm-facing 
 * data structures used by guidance, sequencer, and DAP.
 */
void inject_sensor_data(
    double pos_x, double pos_y, double pos_z,
    double vel_x, double vel_y, double vel_z,
    double acc_x, double acc_y, double acc_z,
    double gyro_p, double gyro_q, double gyro_r,
    double bmx, double bmy, double bmz,
    double deltaVx, double deltaVy, double deltaVz,
    double deltaThetaX, double deltaThetaY, double deltaThetaZ)
{
    /* GNSS Position (ECEF) - meters */
    systemState.position_ecef.x = pos_x;
    systemState.position_ecef.y = pos_y;
    systemState.position_ecef.z = pos_z;

    /* GNSS Velocity (ECEF) - m/s */
    systemState.velocity_ecef.x = vel_x;
    systemState.velocity_ecef.y = vel_y;
    systemState.velocity_ecef.z = vel_z;

    /* Accelerometer - Direct injection into processed data structure (m/s²) */
    systemState.accelerometerData.accel_current.x = acc_x;
    systemState.accelerometerData.accel_current.y = acc_y;
    systemState.accelerometerData.accel_current.z = acc_z;

    /* Gyroscope - Direct injection into processed data structure (rad/s) */
    systemState.gyroscopeData.gyro_current.x = gyro_p;
    systemState.gyroscopeData.gyro_current.y = gyro_q;
    systemState.gyroscopeData.gyro_current.z = gyro_r;

    /* Angular Rates - Used directly by guidance and DAP (rad/s) */
    systemState.angularRates.x = gyro_p;
    systemState.angularRates.y = gyro_q;
    systemState.angularRates.z = gyro_r;

    /* Magnetometer - mG (no conversion needed) */
    systemState.magnetometerData.mag_current.x = bmx;
    systemState.magnetometerData.mag_current.y = bmy;
    systemState.magnetometerData.mag_current.z = bmz;

    /* Incremental Velocity - m/s */
    systemState.incrementalVelocityData.delta_v_converted.x = deltaVx;
    systemState.incrementalVelocityData.delta_v_converted.y = deltaVy;
    systemState.incrementalVelocityData.delta_v_converted.z = deltaVz;

    /* Incremental Angle - rad (no conversion needed) */
    systemState.incrementalAngleData.delta_theta_obc.x = deltaThetaX;
    systemState.incrementalAngleData.delta_theta_obc.y = deltaThetaY;
    systemState.incrementalAngleData.delta_theta_obc.z = deltaThetaZ;
}

/**
 * @brief Log flight software outputs to CSV
 */
void log_outputs(FILE *output_file, double mission_time,
                 double pos_x, double pos_y, double pos_z,
                 double vel_x, double vel_y, double vel_z)
{
    /* Output format: time, GNSS inputs, Guidance outputs, DAP outputs, Nav outputs, Sequencer outputs */
    fprintf(output_file, "%.2f,", mission_time);

    /* Echo GNSS inputs */
    fprintf(output_file, "%.6f,%.6f,%.6f,", pos_x, pos_y, pos_z);
    fprintf(output_file, "%.6f,%.6f,%.6f,", vel_x, vel_y, vel_z);

    /* Guidance outputs - Body frame acceleration commands (m/s²) */
    fprintf(output_file, "%.6f,%.6f,%.6f,",
            systemState.guidanceState.accelCmdBody.x,
            systemState.guidanceState.accelCmdBody.y,
            systemState.guidanceState.accelCmdBody.z);

    /* DAP outputs - Actuator fin angles in clockwise order (radians) */
    fprintf(output_file, "%.6f,%.6f,%.6f,%.6f,",
            systemState.actuatorCommands.ActuatorC12,
            systemState.actuatorCommands.ActuatorC3,
            systemState.actuatorCommands.ActuatorC6,
            systemState.actuatorCommands.ActuatorC9);

    /* Navigation outputs - Attitude (radians) */
    fprintf(output_file, "%.6f,%.6f,%.6f,",
            systemState.navigationState.attitude_e.roll_rad,
            systemState.navigationState.attitude_e.pitch_rad,
            systemState.navigationState.attitude_e.yaw_rad);

    /* Angular rates (rad/s) - store for roll rate calculation */
    double gyro_p_rad_s = systemState.angularRates.x;
    double gyro_q_rad_s = systemState.angularRates.y;
    double gyro_r_rad_s = systemState.angularRates.z;

    fprintf(output_file, "%.6f,%.6f,%.6f,",
            gyro_p_rad_s,
            gyro_q_rad_s,
            gyro_r_rad_s);

    /* Roll rate in RPS (computed by minor_cycle) */
    fprintf(output_file, "%.6f,", systemState.rollRateFp);

    /* Sequencer outputs - Phase flags and timing (cycles) */
    fprintf(output_file, "%d,%d,%d,%d,",
            systemState.sequencerState.isT0Set,
            systemState.sequencerState.isT1Set,
            systemState.sequencerState.isT2Set,
            systemState.sequencerState.isT3Set);
    fprintf(output_file, "%d,%d,%d,%d,",
            systemState.flags.fsaActivateFlag,
            systemState.flags.canardDeployFlag,
            systemState.flags.canardControlFlag,
            systemState.flags.guidStartFlag);

    /* Sequencer timing (cycles) */
    fprintf(output_file, "%u,%u,%u,%u",
            systemState.sequencerState.mainClockCycles,
            systemState.sequencerState.t1SetTime,
            systemState.sequencerState.t2SetTime,
            systemState.sequencerState.t3SetTime);

    fprintf(output_file, "\n");
}

/**
 * @brief Print CSV output header
 */
void print_output_header(FILE *output_file)
{
    fprintf(output_file, "time,");
    fprintf(output_file, "pos_x,pos_y,pos_z,");
    fprintf(output_file, "vel_x,vel_y,vel_z,");
    fprintf(output_file, "guid_ax,guid_ay,guid_az,");
    fprintf(output_file, "dap_c12,dap_c3,dap_c6,dap_c9,");
    fprintf(output_file, "nav_roll,nav_pitch,nav_yaw,");
    fprintf(output_file, "gyro_p,gyro_q,gyro_r,roll_rate_rps,");
    fprintf(output_file, "T0,T1,T2,T3,");
    fprintf(output_file, "fsa_flag,canard_deploy,canard_control,guid_start,");
    fprintf(output_file, "main_cycles,t1_cycles,t2_cycles,t3_cycles");
    fprintf(output_file, "\n");
}

/**
 * @brief Main test harness
 */
int main(int argc, char *argv[])
{
    FILE *input_file;
    FILE *output_file;
    char line_buffer[LINE_BUFFER_SIZE];
    int line_count = 0;

    /* CSV data variables */
    double mission_time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z;
    double acc_x, acc_y, acc_z, gyro_p, gyro_q, gyro_r;
    double bmx, bmy, bmz;
    double deltaVx, deltaVy, deltaVz;
    double deltaThetaX, deltaThetaY, deltaThetaZ;

    printf("=== ISA Flight Software Test Harness ===\n");
    printf("CSV-based open-loop verification\n\n");

    /* Open input CSV */
    input_file = fopen("sensordata.csv", "r");
    if (!input_file)
    {
        printf("ERROR: Could not open sensordata.csv\n");
        return 1;
    }

    /* Open output CSV */
    output_file = fopen("fsw_output.csv", "w");
    if (!output_file)
    {
        printf("ERROR: Could not create fsw_output.csv\n");
        fclose(input_file);
        return 1;
    }

    printf("Input file: sensordata.csv\n");
    printf("Output file: fsw_output.csv\n\n");

    /* Skip header line */
    if (fgets(line_buffer, LINE_BUFFER_SIZE, input_file) == NULL)
    {
        printf("ERROR: Empty input file\n");
        fclose(input_file);
        fclose(output_file);
        return 1;
    }

    /* Write output header */
    print_output_header(output_file);

    /* Initialize flight software when OBC reset is triggered */
    printf("Initializing flight software...\n");
    load_pefcs_defaults(); /* Load navigation/DAP parameters */
    guidance_init();       /* Load guidance parameters (geodetic->ECEF) */

    /* Set OBC reset flag to true to start mission */
    systemState.sequencerState.isOBCReset = true;
    systemState.sequencerState.isT0Set = true; /* CRITICAL: Start in T0 phase */

    printf("OBC reset triggered - Mission starting in T0 phase\n");
    printf("Processing CSV lines...\n\n");

    /* Process each CSV line */
    while (fgets(line_buffer, LINE_BUFFER_SIZE, input_file) != NULL)
    {
        line_count++;

        /* Parse CSV line */
        int fields = sscanf(line_buffer, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                            &mission_time,
                            &pos_x, &pos_y, &pos_z,
                            &vel_x, &vel_y, &vel_z,
                            &acc_x, &acc_y, &acc_z,
                            &gyro_p, &gyro_q, &gyro_r,
                            &bmx, &bmy, &bmz,
                            &deltaVx, &deltaVy, &deltaVz,
                            &deltaThetaX, &deltaThetaY, &deltaThetaZ);

        if (fields != 22)
        {
            printf("Warning: Line %d has %d fields (expected 22), skipping\n", line_count, fields);
            continue;
        }

        /* Inject sensor data into systemState */
        inject_sensor_data(pos_x, pos_y, pos_z,
                           vel_x, vel_y, vel_z,
                           acc_x, acc_y, acc_z,
                           gyro_p, gyro_q, gyro_r,
                           bmx, bmy, bmz,
                           deltaVx, deltaVy, deltaVz,
                           deltaThetaX, deltaThetaY, deltaThetaZ);

        /* Execute flight software cycles */
        minor_cycle(); /* Navigation, Sequencer, DAP (100Hz) */

        /* Major cycle runs at 10Hz - only every 10th minor cycle */
        if (line_count % 10 == 0)
        {
            major_cycle(); /* Guidance (10Hz) */
        }

        /* Log outputs */
        log_outputs(output_file, mission_time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z);

        /* Progress indicator every 100 lines */
        if (line_count % 100 == 0)
        {
            printf("Processed %d lines (t=%.2fs)\n", line_count, mission_time);
        }
    }

    /* Cleanup */
    fclose(input_file);
    fclose(output_file);

    printf("\nTest complete!\n");
    printf("Total lines processed: %d\n", line_count);
    printf("Output written to: fsw_output.csv\n");

    return 0;
}
