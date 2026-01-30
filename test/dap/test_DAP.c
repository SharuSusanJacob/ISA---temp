/******************************************************************************
 * ISA Digital Autopilot (DAP) Module
 *
 * File: main_DAP.c
 * Description: Main for DAP system test
 *****************************************************************************/
#define _POSIX_C_SOURCE 199309L

#include <stdio.h>
#include <time.h>
#include "types.h"
#include "config.h"
#include "dap.h"
#include "vector3.h"

int main(void)
{
    /* Test input values */
    const double phi = 1.0;                   /* Roll angle in radians */
    // const double accelerationY = 0.1;         /* Y-axis (pitch) acceleration in m/s^2 */
    // const double accelerationZ = 0.1;         /* Z-axis (yaw) acceleration in m/s^2 */
    // const double rollRate = 0.0 * DEG_TO_RAD; /* Roll rate in rad/s */
    // const double pitchRate = 0.2;            /* Pitch rate in rad/s */
    // const double yawRate = 1.0;              /* Yaw rate in rad/s */
    // const double accelerationYCommand = 0.4;  /* Commanded Y-axis acceleration */
    // const double accelerationZCommand = 4.0;  /* Commanded Z-axis acceleration */
    const double timeStep = 0.1;              /* Time step in seconds */
    const bool canardControlFlag = true;     /* Flag to enable canard control */

    /* Use default DAP parameters */
    const DAPParameters_t dapParams = DAP_DEFAULT;
    struct timespec start = { 0, 0 };
    struct timespec end = { 0, 0 };
    double elapsed = 0.0f;
    DAPOutput_t dapOutput = { 0.0, 0.0, 0.0, 0.0 };
    const double one_billion = 1000000000.0;

    /* File pointers */
    FILE *inputFile = fopen("DAP_input_profile_case25666.csv", "r");
    FILE *outputFile = fopen("output.csv", "w");
    if (inputFile == NULL || outputFile == NULL) {
        printf("Error opening files.\n");
        return 1;
    }

    /* Write output CSV header */
    fprintf(outputFile, "time,delta3_rad,delta6_rad,delta9_rad,delta12_rad\n");

    /* Skip input CSV header if present (assuming first line is header) */
    char line[1024];
    if (fgets(line, sizeof(line), inputFile) == NULL) {
        printf("Error reading input file.\n");
        fclose(inputFile);
        fclose(outputFile);
        return 1;
    }

    /* Read and process each line */
    while (fgets(line, sizeof(line), inputFile)) {
        double time, accelerationYCommand, accelerationZCommand, accelerationY, accelerationZ, rollRate, pitchRate, yawRate;
        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &time, &accelerationYCommand, &accelerationZCommand, &accelerationY, &accelerationZ, &rollRate, &pitchRate, &yawRate) != 8) {
            printf("Error parsing line: %s\n", line);
            continue;
        }

    (void)clock_gettime(CLOCK_MONOTONIC, &start);


    /* Execute DAP algorithm */
    dapOutput = DAP_Execute(
        phi,
        accelerationY,
        accelerationZ,
        rollRate,
        pitchRate,
        yawRate,
        accelerationYCommand,
        accelerationZCommand,
        dapParams,
        timeStep,
        canardControlFlag);

    (void)clock_gettime(CLOCK_MONOTONIC, &end);
    elapsed = (double)(end.tv_sec - start.tv_sec);
    elapsed = elapsed + ((double)(end.tv_nsec - start.tv_nsec) / (double)one_billion);

    (void)printf("Execution time: %.9f seconds\n", (double)elapsed);

    /* Print results */
    (void)printf("  DAP Output: (%.6f, %.6f, %.6f, %.6f, %.6f)\n",
        time, dapOutput.delta3_rad, dapOutput.delta6_rad, dapOutput.delta9_rad, dapOutput.delta12_rad);

    /* Write results to output CSV */
    fprintf(outputFile, "%.6f,%.6f,%.6f,%.6f,%.6f\n",
                time,dapOutput.delta3_rad, dapOutput.delta6_rad, dapOutput.delta9_rad, dapOutput.delta12_rad);
    }

    /* Close files */
    fclose(inputFile);
    fclose(outputFile);

    return 0;
}
