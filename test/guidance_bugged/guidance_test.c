/******************************************************************************
* ISA Flight Software
 * @file guidance_test.c
* @brief Test harness for guidance module
 * @details Mirrors MATLAB Test_OBG.m functionality
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
 * @version 1.0
 *
* MISRA C: Compliant Implementation
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "guidance.h"
#include "vector3.h"
#include "quaternion.h"
#include "coordinate_transforms.h"
#include "math_utils.h"

// Test parameters (matching MATLAB Test_OBG.m)
#define TEST_LAUNCH_LAT_DEG     8.529175797045042
#define TEST_LAUNCH_LON_DEG     76.88543289537785
#define TEST_LAUNCH_ALT_M       0.0

#define TEST_TARGET_LAT_DEG     8.50853578598926
#define TEST_TARGET_LON_DEG     76.70458281218454
#define TEST_TARGET_ALT_M       0.0

// Initial position in ECEF frame (m)
#define TEST_X_0_ECEF_M          1440170.839
#define TEST_Y_0_ECEF_M          6144033.51
#define TEST_Z_0_ECEF_M          937590.246010134
// Initial velocity in ECEF frame (m/s)
#define TEST_VX_0_ECEF_M_S       314.1601369
#define TEST_VY_0_ECEF_M_S       -68.20625309
#define TEST_VZ_0_ECEF_M_S       -34.91986689

#define TEST_THETA_F_DEG        -90.0
#define TEST_PSI_F_DEG          0.0


// Simulation parameters (matching MATLAB)
#define SIM_DT_S                0.01  // Time step for simulation (matching MATLAB)
#define SIM_MAX_TIME_S          200.0
#define SIM_MIN_VELOCITY_M_S    1.0

// File output for MATLAB comparison
#define OUTPUT_FILENAME         "C:\\Users\\SPACELABS\\OneDrive\\Desktop\\Projects\\ISA Flight Software\\test\\guidance\\guidance_test_results.csv"

/**
 * @brief Run guidance simulation like MATLAB Test_OBG.m
 * @param launch_lat_deg Launch latitude (degrees)
 * @param launch_lon_deg Launch longitude (degrees)
 * @param launch_alt_m Launch altitude (meters)
 * @param target_lat_deg Target latitude (degrees)
 * @param target_lon_deg Target longitude (degrees)
 * @param target_alt_m Target altitude (meters)
 * @param initial_pos_ecef Initial position in ECEF frame [x,y,z] (meters)
 * @param initial_vel_ecef Initial velocity in ECEF frame [vx,vy,vz] (m/s)
 * @param theta_f_deg Desired impact elevation angle (degrees)
 * @param psi_f_deg Desired impact azimuth angle (degrees)
 * @return int 0 for success, non-zero for failure
 */
int run_guidance_simulation(
    double launch_lat_deg, double launch_lon_deg, double launch_alt_m,
    double target_lat_deg, double target_lon_deg, double target_alt_m,
    const Vector3_t* initial_pos_ecef, const Vector3_t* initial_vel_ecef,
    double theta_f_deg, double psi_f_deg)
{
    // Convert impact angles to radians
    double theta_f_rad, psi_f_rad;
    MathUtilsError_t math_error;

    math_error = deg_to_rad(&theta_f_rad, theta_f_deg);
    if (math_error != MATH_UTILS_SUCCESS) {
        printf("Error converting theta_f to radians\n");
        return 1;
    }

    math_error = deg_to_rad(&psi_f_rad, psi_f_deg);
    if (math_error != MATH_UTILS_SUCCESS) {
        printf("Error converting psi_f to radians\n");
        return 1;
    }

    // Create launch position in geodetic coordinates
    GeodeticPos_t launch;
    CoordTransformError_t coord_error;

    coord_error = geodetic_create(&launch, launch_lat_deg, launch_lon_deg, launch_alt_m);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        printf("Error creating launch geodetic position\n");
        return 1;
    }

    // Create target position in geodetic coordinates
    GeodeticPos_t target;

    coord_error = geodetic_create(&target, target_lat_deg, target_lon_deg, target_alt_m);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        printf("Error creating target geodetic position\n");
        return 1;
    }

    // Initialize guidance state
    GuidanceState_t guidance_state;
    GuidanceError_t guid_error;

    guid_error = guidance_init(&guidance_state);
    if (guid_error != GUID_SUCCESS) {
        printf("Error initializing guidance module\n");
        return 1;
    }

    // Set launch and target positions
    guid_error = guidance_set_launch(&guidance_state, &launch);
    if (guid_error != GUID_SUCCESS) {
        printf("Error setting guidance launch position\n");
        return 1;
    }

    guid_error = guidance_set_target(&guidance_state, &target);
    if (guid_error != GUID_SUCCESS) {
        printf("Error setting guidance target\n");
        return 1;
    }

    guid_error = guidance_set_impact_angles(&guidance_state, theta_f_rad, psi_f_rad);
    if (guid_error != GUID_SUCCESS) {
        printf("Error setting impact angles\n");
        return 1;
    }

    // Set initial state
    guidance_state.position_ecef = *initial_pos_ecef;
    guidance_state.velocity_ecef = *initial_vel_ecef;
    guidance_state.guidance_active = true;

    // Calculate initial distance to target
    // With launch-centered coordinates, convert initial position to local frame
    Vector3_t R_m_local;
    coord_error = ecef_to_local(&R_m_local, &guidance_state.position_ecef, &launch, &target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        printf("Error converting initial position to local frame\n");
        return 1;
    }

    // Calculate exact target position in local frame
    Vector3_t target_ecef;
    coord_error = geodetic_to_ecef(&target_ecef, &target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        printf("Error converting target to ECEF\n");
        return 1;
    }

    Vector3_t target_local;
    coord_error = ecef_to_local(&target_local, &target_ecef, &launch, &target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        printf("Error converting target to local frame\n");
        return 1;
    }

    printf("Target position in local frame: (%.2lf, %.2lf, %.2lf) m\n",
        target_local.x, target_local.y, target_local.z);

    // Calculate R vector from projectile to target
    Vector3_t R;
    R.x = target_local.x - R_m_local.x;
    R.y = target_local.y - R_m_local.y;
    R.z = target_local.z - R_m_local.z;

    /* MATLAB uses distance with a +100-m offset on Z (GUID_TERMINAL_DISTANCE_M) */
    Vector3_t R_offset = R;
    R_offset.z += GUID_TERMINAL_DISTANCE_M;

    double initial_distance;
    Vector3Error_t vec_error = vector3_magnitude(&initial_distance, &R_offset);
    if (vec_error != VECTOR3_SUCCESS) {
        printf("Error calculating initial distance\n");
        return 1;
    }

    // Print initial conditions
    printf("GUIDANCE PHASE STARTED (APOGEE)\n");
    printf("Initial projectile ECEF position: (%.2lf, %.2lf, %.2lf) m\n",
        initial_pos_ecef->x, initial_pos_ecef->y, initial_pos_ecef->z);
    printf("Initial projectile ECEF velocity: (%.2lf, %.2lf, %.2lf) m/s\n",
        initial_vel_ecef->x, initial_vel_ecef->y, initial_vel_ecef->z);
    printf("Target Distance: %.2lf m\n\n", initial_distance);

    // Open output file for MATLAB comparison
    FILE* output_file = NULL;

    // Try to open the file
    output_file = fopen(OUTPUT_FILENAME, "w");
    if (output_file == NULL) {
        printf("Warning: Could not open output file %s. Continuing without file output.\n", OUTPUT_FILENAME);
        // Continue without file output - don't return error
    }
    else {
        printf("Writing results to %s\n", OUTPUT_FILENAME);
    }

    // Write CSV header
    if (output_file != NULL) {
        fprintf(output_file, "Time,X_ECEF,Y_ECEF,Z_ECEF,VX_ECEF,VY_ECEF,VZ_ECEF,AX_ECEF,AY_ECEF,AZ_ECEF,Distance,TimeToGo,Theta,Psi\n");
    }

    // target_ecef and target_local are already calculated above

    // Simulation variables
    double t = 0.0;
    double prev_distance = initial_distance;
    double velocity_mag;
    double current_distance = initial_distance;

    // Calculate initial velocity magnitude
    vec_error = vector3_magnitude(&velocity_mag, initial_vel_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        printf("Error calculating initial velocity magnitude\n");
        fclose(output_file);
        return 1;
    }

    // Timing variables for guidance cycle measurement (multiple iterations for precision)
    clock_t start_time, end_time;
    double guidance_execution_time;
    bool timing_measured = false;
    const int TIMING_ITERATIONS = 1000;  // Run 1000 iterations for better timing precision

    // Simulation loop
    // Simulation condition: while t < max_simulation_time && r <= r_prev && norm(V_m_ECEF) > min_missile_velocity
    while (t < SIM_MAX_TIME_S && current_distance <= prev_distance && velocity_mag > SIM_MIN_VELOCITY_M_S) {
        // Measure execution time for guidance cycle (only once, using multiple iterations)
        if (!timing_measured) {
            printf("Measuring guidance execution time with %d iterations...\n", TIMING_ITERATIONS);
            start_time = clock();

            // Run multiple iterations to get measurable time
            for (int i = 0; i < TIMING_ITERATIONS; i++) {
                guid_error = guidance_execute(&guidance_state, SIM_DT_S);
                if (guid_error != GUID_SUCCESS) {
                    printf("Error executing guidance algorithm during timing test\n");
                    if (output_file != NULL) {
                        fclose(output_file);
                    }
                    return 1;
                }
            }

            end_time = clock();
            guidance_execution_time = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
            double single_cycle_time = guidance_execution_time / TIMING_ITERATIONS;

            printf("Total time for %d iterations: %.6f seconds\n", TIMING_ITERATIONS, guidance_execution_time);
            printf("Single guidance cycle execution time: %.9f seconds (%.6f ms) (%.3f microseconds)\n",
                single_cycle_time, single_cycle_time * 1000.0, single_cycle_time * 1000000.0);
            timing_measured = true;
        }
        else {
            // Normal execution for the rest of the simulation
            guid_error = guidance_execute(&guidance_state, SIM_DT_S);
            if (guid_error != GUID_SUCCESS) {
                printf("Error executing guidance algorithm at t=%.2lf s\n", t);
                if (output_file != NULL) {
                    fclose(output_file);
                }
                return 1;
            }
        }

        // Calculate current velocity magnitude
        vec_error = vector3_magnitude(&velocity_mag, &guidance_state.velocity_ecef);
        if (vec_error != VECTOR3_SUCCESS) {
            printf("Error calculating velocity magnitude at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        // Velocity check is now part of the loop condition

        // Convert position to local frame for distance calculation
        Vector3_t position_local;
        coord_error = ecef_to_local(&position_local, &guidance_state.position_ecef, &launch, &target);
        if (coord_error != COORD_TRANSFORM_SUCCESS) {
            printf("Error converting position to local frame at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        /* Calculate current distance to target using offset on Z (matches MATLAB) */
        Vector3_t R;
        R.x = target_local.x - position_local.x;
        R.y = target_local.y - position_local.y;
        R.z = target_local.z - position_local.z;

        Vector3_t R_offset_iter = R;
        R_offset_iter.z += GUID_TERMINAL_DISTANCE_M;

        prev_distance = current_distance;
        vec_error = vector3_magnitude(&current_distance, &R_offset_iter);
        if (vec_error != VECTOR3_SUCCESS) {
            printf("Error calculating current distance at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        // Distance and velocity checks are now part of the loop condition

        // Calculate current elevation and azimuth angles
        Vector3_t velocity_local;
        coord_error = ecef_to_local_velocity(&velocity_local, &guidance_state.velocity_ecef, &launch, &target);
        if (coord_error != COORD_TRANSFORM_SUCCESS) {
            printf("Error converting velocity to local frame at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        double velocity_local_mag;
        vec_error = vector3_magnitude(&velocity_local_mag, &velocity_local);
        if (vec_error != VECTOR3_SUCCESS) {
            printf("Error calculating local velocity magnitude at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        double theta_rad = asin(velocity_local.z / velocity_local_mag);
        double psi_rad = atan2(velocity_local.y, velocity_local.x);

        double theta_deg, psi_deg;
        math_error = rad_to_deg(&theta_deg, theta_rad);
        if (math_error != MATH_UTILS_SUCCESS) {
            printf("Error converting theta to degrees at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        math_error = rad_to_deg(&psi_deg, psi_rad);
        if (math_error != MATH_UTILS_SUCCESS) {
            printf("Error converting psi to degrees at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        // Write state to output file (for MATLAB comparison)
        if (output_file != NULL) {
            fprintf(output_file, "%.4lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf\n",
                t,
                guidance_state.position_ecef.x, guidance_state.position_ecef.y, guidance_state.position_ecef.z,
                guidance_state.velocity_ecef.x, guidance_state.velocity_ecef.y, guidance_state.velocity_ecef.z,
                guidance_state.acceleration_cmd_ecef.x, guidance_state.acceleration_cmd_ecef.y, guidance_state.acceleration_cmd_ecef.z,
                current_distance, guidance_state.time_to_go_s, theta_deg, psi_deg);
        }

        // Print current state every 0.1 seconds while using 0.01s time step
        if ((int)(t * 10.0) % 10 == 0) { // Print every 0.1 seconds for console output
            printf("Time: %.2lf s\n", t);
            // Display actual local guidance command from guidance state
            printf("GUIDANCE COMMAND LOCAL: (%.2lf, %.2lf, %.2lf) m/s^2\n",
                guidance_state.acceleration_cmd_local.x,
                guidance_state.acceleration_cmd_local.y,
                guidance_state.acceleration_cmd_local.z);
            printf("GUIDANCE COMMAND BODY: (%.2lf, %.2lf, %.2lf) m/s^2\n",
                guidance_state.acceleration_cmd_body.x,
                guidance_state.acceleration_cmd_body.y,
                guidance_state.acceleration_cmd_body.z);
            printf("PURE GUIDANCE ECEF: (%.2lf, %.2lf, %.2lf) m/s^2\n",
                guidance_state.pure_guidance_ecef.x,
                guidance_state.pure_guidance_ecef.y,
                guidance_state.pure_guidance_ecef.z);
            // Display total acceleration in ECEF frame (A_total_ECEF = A_M_ECEF - A_D_ECEF + grav_ECEF)
            // Note: In our implementation, drag is stored with positive sign (same direction as velocity),
            // so we actually compute A_total_ECEF = A_M_ECEF + (-A_D_ECEF) + grav_ECEF
            // This is the final acceleration command that includes guidance, drag, and gravity effects
            printf("TOTAL ACCELERATION IN ECEF: (%.2lf, %.2lf, %.2lf) m/s^2\n",
                guidance_state.acceleration_cmd_ecef.x,
                guidance_state.acceleration_cmd_ecef.y,
                guidance_state.acceleration_cmd_ecef.z);
            // Display actual drag and gravity values from guidance module
            printf("DRAG IN ECEF: (%.2lf, %.2lf, %.2lf) m/s^2\n",
                guidance_state.drag_acceleration_ecef.x,
                guidance_state.drag_acceleration_ecef.y,
                guidance_state.drag_acceleration_ecef.z);
            printf("GRAVITY IN ECEF: (%.2lf, %.2lf, %.2lf) m/s^2\n",
                guidance_state.gravity_acceleration_ecef.x,
                guidance_state.gravity_acceleration_ecef.y,
                guidance_state.gravity_acceleration_ecef.z);
            printf("Projectile ECEF position: (%.2lf, %.2lf, %.2lf) m\n",
                guidance_state.position_ecef.x,
                guidance_state.position_ecef.y,
                guidance_state.position_ecef.z);
            printf("Projectile ECEF velocity: (%.2lf, %.2lf, %.2lf) m/s\n",
                guidance_state.velocity_ecef.x,
                guidance_state.velocity_ecef.y,
                guidance_state.velocity_ecef.z);
            printf("Projectile LOCAL position: (%.2lf, %.2lf, %.2lf) m\n",
                guidance_state.position_local.x,
                guidance_state.position_local.y,
                guidance_state.position_local.z);
            printf("Projectile LOCAL velocity: (%.2lf, %.2lf, %.2lf) m/s\n",
                velocity_local.x,
                velocity_local.y,
                velocity_local.z);
            printf("Theta: %.2lf deg, Psi: %.2lf deg\n", theta_deg, psi_deg);
            printf("Time to go: %.2lf s\n", guidance_state.time_to_go_s);
            printf("Distance to target: %.2lf m\n\n", current_distance);
        }

        // Integrate acceleration to get velocity in ECEF
        Vector3_t velocity_increment;
        vec_error = vector3_scale(&velocity_increment, &guidance_state.acceleration_cmd_ecef, SIM_DT_S);
        if (vec_error != VECTOR3_SUCCESS) {
            printf("Error calculating velocity increment at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        vec_error = vector3_add(&guidance_state.velocity_ecef, &guidance_state.velocity_ecef, &velocity_increment);
        if (vec_error != VECTOR3_SUCCESS) {
            printf("Error updating velocity at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        // Integrate velocity to get position in ECEF
        Vector3_t position_increment;
        vec_error = vector3_scale(&position_increment, &guidance_state.velocity_ecef, SIM_DT_S);
        if (vec_error != VECTOR3_SUCCESS) {
            printf("Error calculating position increment at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        vec_error = vector3_add(&guidance_state.position_ecef, &guidance_state.position_ecef, &position_increment);
        if (vec_error != VECTOR3_SUCCESS) {
            printf("Error updating position at t=%.2lf s\n", t);
            if (output_file != NULL) {
                fclose(output_file);
            }
            return 1;
        }

        // Update for next iteration
        t += SIM_DT_S;
    }

    // Close output file
    fclose(output_file);

    // Check why we exited the loop
    if (t >= SIM_MAX_TIME_S) {
        printf("Simulation time exceeded.\n");
    }
    else if (velocity_mag <= SIM_MIN_VELOCITY_M_S) {
        printf("Projectile velocity too low (%.2lf m/s) at t=%.2lf s\n", velocity_mag, t);
    }
    else if (current_distance > prev_distance) {
        printf("Target impact at t=%.2lf s\n", t);
        printf("Final ECEF position: (%.2lf, %.2lf, %.2lf) m\n",
            guidance_state.position_ecef.x, guidance_state.position_ecef.y, guidance_state.position_ecef.z);
        printf("Final distance to target: %.2lf m\n", prev_distance);
    }

    printf("Results saved to %s for MATLAB comparison\n", OUTPUT_FILENAME);

    return 0;
}

/**
 * @brief Main function that matches MATLAB Test_OBG.m
 * @return int 0 for success, non-zero for failure
 */
int main(void) {
    printf("ISA Flight Software - Guidance Test\n");
    printf("===================================\n\n");

    // Create initial position and velocity vectors (like MATLAB)
    Vector3_t initial_pos_ecef, initial_vel_ecef;
    Vector3Error_t vec_error;

    vec_error = vector3_create(&initial_pos_ecef,
        TEST_X_0_ECEF_M,
        TEST_Y_0_ECEF_M,
        TEST_Z_0_ECEF_M);
    if (vec_error != VECTOR3_SUCCESS) {
        printf("Error creating initial position vector\n");
        return 1;
    }

    vec_error = vector3_create(&initial_vel_ecef,
        TEST_VX_0_ECEF_M_S,
        TEST_VY_0_ECEF_M_S,
        TEST_VZ_0_ECEF_M_S);
    if (vec_error != VECTOR3_SUCCESS) {
        printf("Error creating initial velocity vector\n");
        return 1;
    }

    // Run the guidance simulation with MATLAB parameters
    int result = run_guidance_simulation(
        TEST_LAUNCH_LAT_DEG, TEST_LAUNCH_LON_DEG, TEST_LAUNCH_ALT_M,
        TEST_TARGET_LAT_DEG, TEST_TARGET_LON_DEG, TEST_TARGET_ALT_M,
        &initial_pos_ecef, &initial_vel_ecef,
        TEST_THETA_F_DEG, TEST_PSI_F_DEG);

    if (result == 0) {
        printf("Guidance test completed successfully.\n");
    }
    else {
        printf("Guidance test failed with error code %d.\n", result);
    }

    return result;
}