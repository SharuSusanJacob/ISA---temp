/******************************************************************************
* ISA Flight Software
 * @file guidance_test.c
 * @brief Guidance Module Test Harness
* @details Test harness for validating guidance algorithm implementation
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 4.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#include "guidance.h"
#include "math_utils.h"
#include <stdio.h>

int main(void)
{
    /* Open CSV file for data logging */
    FILE *csv_file = fopen("guidance_test_results.csv", "w");
    if (csv_file == NULL)
    {
        printf("Warning: Could not open CSV file for writing. Continuing without CSV output.\n");
    }
    else
    {
        printf("CSV file opened: guidance_test_results.csv\n");
    }

    double lat_o = 8.529175797045042;
    double lon_o = 76.88543289537785;
    double alt_o = 0.0;

    double lat_t = 8.50853578598926;
    double lon_t = 76.70458281218454;
    double alt_t = 0.0;

    /* Projectile states at apogee (guidance phase start) in ECEF frame */
    double x_0_ecef = 1440170.839;
    double y_0_ecef = 6144033.51;
    double z_0_ecef = 937590.246010134;
    double vx_0_ecef = 314.1601369;
    double vy_0_ecef = -68.20625309;
    double vz_0_ecef = -34.91986689;

    /* Desired impact angles (radians) */
    double theta_f = deg2rad(-90.0);
    double psi_f = deg2rad(0.0);

    /* Run the onboard guidance algorithm */
    onboard_guidance_algorithm(lat_o, lon_o, alt_o, lat_t, lon_t, alt_t,
                               x_0_ecef, y_0_ecef, z_0_ecef,
                               vx_0_ecef, vy_0_ecef, vz_0_ecef,
                               theta_f, psi_f,
                               csv_file);

    /* Close CSV file */
    if (csv_file != NULL)
    {
        fclose(csv_file);
        printf("\nCSV file saved: guidance_test_results.csv\n");
    }

    return 0;
}
