/******************************************************************************
 * ISA Flight Software
 * @file guidance_test.c
 * @brief Guidance Module Test Harness - CSV Profile Input
 * @details Reads trajectory profile from CSV and generates guidance commands
 * @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
 * @version 5.0
 *****************************************************************************/

#include "guidance.h"
#include "math_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE_LENGTH 1024

int main(void)
{
    /* Coordinates of launch point */
    double lat_O = 8.529175797045042;
    double lon_O = 76.88543289537785;
    double alt_O = 0.0;

    /* Coordinates target for 20 km range (Sea) */
    double lat_T = 8.50853578598926;
    double lon_T = 76.70458281218454;
    double alt_T = 0.0;

    /* Desired impact angles (radians) */
    double theta_f = DEG2RAD(-73.0); /* desired elevation at impact */
    double psi_f = DEG2RAD(0.0);     /* desired azimuth at impact */

    /* Open input file */
    FILE *profile_file = fopen("Profile.csv", "r");
    if (profile_file == NULL)
    {
        fprintf(stderr, "Error: Could not open Profile.csv\n");
        return 1;
    }

    /* Open output file */
    FILE *results_file = fopen("Results.csv", "w");
    if (results_file == NULL)
    {
        fprintf(stderr, "Error: Could not create Results.csv\n");
        fclose(profile_file);
        return 1;
    }

    /* Write header to results file */
    fprintf(results_file, "t_in,x_0_ECEF,y_0_ECEF,z_0_ECEF,vx_0_ECEF,vy_0_ECEF,vz_0_ECEF,");
    fprintf(results_file, "t_go,r,theta_d,psi_d,A_M_BODYx,A_M_BODYy,A_M_BODYz\n");

    /* Skip header line in Profile.csv */
    char line[MAX_LINE_LENGTH];
    fgets(line, sizeof(line), profile_file);

    /* Initialize previous body acceleration to zero */
    Vector3 prev_body_accel = {0.0, 0.0, 0.0};

    int cycle = 0;

    /* Process each line of the profile */
    while (fgets(line, sizeof(line), profile_file) != NULL)
    {
        double t_in, x_0_ecef, y_0_ecef, z_0_ecef;
        double vx_0_ecef, vy_0_ecef, vz_0_ecef;

        /* Parse CSV line */
        int items = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                           &t_in, &x_0_ecef, &y_0_ecef, &z_0_ecef,
                           &vx_0_ecef, &vy_0_ecef, &vz_0_ecef);

        if (items != 7)
        {
            fprintf(stderr, "Warning: Could not parse line %d, skipping\n", cycle + 1);
            continue;
        }

        /* Run guidance algorithm */
        GuidanceOutput output;
        onboard_guidance_algorithm(
            lat_O, lon_O, alt_O,
            lat_T, lon_T, alt_T,
            t_in,
            x_0_ecef, y_0_ecef, z_0_ecef,
            vx_0_ecef, vy_0_ecef, vz_0_ecef,
            &prev_body_accel,
            theta_f, psi_f,
            &output);

        /* Write results */
        fprintf(results_file, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,",
                t_in, x_0_ecef, y_0_ecef, z_0_ecef,
                vx_0_ecef, vy_0_ecef, vz_0_ecef);
        fprintf(results_file, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                output.t_go, output.r, output.theta_deg, output.psi_deg,
                output.A_M_body.x, output.A_M_body.y, output.A_M_body.z);

        /* Update previous body acceleration for next cycle */
        prev_body_accel = output.A_M_body;

        /* Display progress every 100 cycles */
        if (cycle % 100 == 0)
        {
            printf("Cycle %d completed. t_go = %.2f, r = %.2f\n",
                   cycle, output.t_go, output.r);
        }

        cycle++;
    }

    printf("\nProcessing complete!\n");
    printf("Input file: Profile.csv (%d cycles)\n", cycle);
    printf("Output file: Results.csv\n");

    fclose(profile_file);
    fclose(results_file);

    return 0;
}
