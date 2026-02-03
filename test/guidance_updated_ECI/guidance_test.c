/******************************************************************************
 * ISA Flight Software
 * @file guidance_test.c
 * @brief Guidance Module Test Harness - CSV Profile Input
 * @details Reads trajectory profile from CSV and generates guidance commands.
 *          Updated for ECEF->ECI flow with GPS time inputs.
 * @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2026
 * @version 6.0
 * 
 * Matches MATLAB: Test_OBG_OL_CSV_FSW.m
 *****************************************************************************/

#include "guidance.h"
#include "math_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE_LENGTH 1024

int main(void)
{
    /* ========== Test states 20 km - MODIFIED FOR CSV PROFILE INPUT ========== */

    /* coordinates of launch point */
    double lat_O = 8.529175797045042;
    double lon_O = 76.88543289537785;
    double alt_O = 0.0;

    /* coordinates target for 20 km range (Sea) */
    double lat_T = 8.50853578598926;
    double lon_T = 76.70458281218454;
    double alt_T = 0.0;

    /* Desired impact angles (radians) */
    double theta_f = DEG2RAD(-73.0); /* desired elevation at impact */
    double psi_f = DEG2RAD(0.0);     /* desired azimuth at impact */

    /* Path to profile CSV file */
    const char *profile_filename = "Profile.csv";

    /* Open input file */
    FILE *profile_file = fopen(profile_filename, "r");
    if (profile_file == NULL)
    {
        fprintf(stderr, "Error: Could not open %s\n", profile_filename);
        return 1;
    }

    /* Open output file */
    const char *results_filename = "Results.csv";
    FILE *results_file = fopen(results_filename, "w");
    if (results_file == NULL)
    {
        fprintf(stderr, "Error: Could not create %s\n", results_filename);
        fclose(profile_file);
        return 1;
    }

    /* Write header to results file */
    /* Matches MATLAB: all_headers = {'t_in', 'x_0_ECEF', 'y_0_ECEF', 'z_0_ECEF', 
       'vx_0_ECEF', 'vy_0_ECEF', 'vz_0_ECEF', 't_go', 'r', 'theta_d', 'psi_d', 
       'A_M_BODYx', 'A_M_BODYy', 'A_M_BODYz'} */
    fprintf(results_file, "t_in,x_0_ECEF,y_0_ECEF,z_0_ECEF,vx_0_ECEF,vy_0_ECEF,vz_0_ECEF,");
    fprintf(results_file, "t_go,r,theta_d,psi_d,A_M_BODYx,A_M_BODYy,A_M_BODYz\n");

    /* Skip header line in Profile CSV (if present) */
    char line[MAX_LINE_LENGTH];
    /* Check if first line is a header by attempting to parse */
    if (fgets(line, sizeof(line), profile_file) == NULL)
    {
        fprintf(stderr, "Error: Empty profile file\n");
        fclose(profile_file);
        fclose(results_file);
        return 1;
    }

    /* Check if first character is a digit - if so, it's data, not header */
    /* If it's a header, we skip it; if it's data, we rewind */
    double test_val;
    if (sscanf(line, "%lf", &test_val) != 1)
    {
        /* First line is header, already skipped */
    }
    else
    {
        /* First line is data, rewind to beginning */
        rewind(profile_file);
    }

    /* Initialize previous command values for first cycle */
    /* Matches MATLAB: A_M_BODY_prevx = 0.0; A_M_BODY_prevy = 0.0; A_M_BODY_prevz = 0.0; */
    Vector3 prev_body_accel = {0.0, 0.0, 0.0};

    int cycle = 0;

    /* Process each cycle */
    while (fgets(line, sizeof(line), profile_file) != NULL)
    {
        /* Extract inputs from current row */
        /* MATLAB format: t_in, x_0_ECEF, y_0_ECEF, z_0_ECEF, vx_0_ECEF, vy_0_ECEF, vz_0_ECEF,
                          GNSSWeek, TOW, leap_seconds */
        double t_in, x_0_ecef, y_0_ecef, z_0_ecef;
        double vx_0_ecef, vy_0_ecef, vz_0_ecef;
        int gnss_week, leap_seconds;
        double tow;

        /* Parse CSV line (10 columns) */
        int items = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%lf,%d",
                           &t_in, &x_0_ecef, &y_0_ecef, &z_0_ecef,
                           &vx_0_ecef, &vy_0_ecef, &vz_0_ecef,
                           &gnss_week, &tow, &leap_seconds);

        if (items != 10)
        {
            fprintf(stderr, "Warning: Could not parse line %d (got %d items), skipping\n", cycle + 1, items);
            continue;
        }

        /* Run guidance algorithm */
        /* Matches MATLAB: onboard_guidance_algorithm_OL_FSW(lat_O, lon_O, alt_O, lat_T, lon_T, alt_T,
           t_in, x_0_ECEF, y_0_ECEF, z_0_ECEF, vx_0_ECEF, vy_0_ECEF, vz_0_ECEF, GNSSWeek, TOW, leap_seconds,
           A_M_BODY_prevx, A_M_BODY_prevy, A_M_BODY_prevz, theta_f, psi_f) */
        GuidanceOutput output;
        onboard_guidance_algorithm(
            lat_O, lon_O, alt_O,
            lat_T, lon_T, alt_T,
            gnss_week, tow, leap_seconds,
            x_0_ecef, y_0_ecef, z_0_ecef,
            vx_0_ecef, vy_0_ecef, vz_0_ecef,
            &prev_body_accel,
            theta_f, psi_f,
            &output);

        /* Store inputs and outputs */
        /* Write results (matches MATLAB output format) */
        fprintf(results_file, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,",
                t_in, x_0_ecef, y_0_ecef, z_0_ecef,
                vx_0_ecef, vy_0_ecef, vz_0_ecef);
        fprintf(results_file, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                output.t_go, output.r, output.theta_deg, output.psi_deg,
                output.A_M_body.x, output.A_M_body.y, output.A_M_body.z);

        /* Update previous command values for next cycle */
        prev_body_accel = output.A_M_body;

        /* Display progress */
        if (cycle % 100 == 0)
        {
            printf("Cycle %d completed. t_go = %.2f, r = %.2f\n",
                   cycle, output.t_go, output.r);
        }

        cycle++;
    }

    /* Display summary */
    printf("\nProcessing complete!\n");
    printf("Input file: %s (%d cycles)\n", profile_filename, cycle);
    printf("Output file: %s\n", results_filename);
    printf("Results contain: Inputs (7 columns) + Outputs (7 columns)\n");

    fclose(profile_file);
    fclose(results_file);

    return 0;
}
