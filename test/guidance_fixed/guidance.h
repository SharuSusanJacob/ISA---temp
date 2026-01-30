/******************************************************************************
* ISA Flight Software
 * @file guidance.h
 * @brief Projectile Flight Computer Guidance Module Header
* @details Defines constants and function prototypes for guidance algorithm
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 4.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "vector3.h"
#include <stdio.h>

/* Physical constants (matching MATLAB exactly) */
#define G_CONST 9.8    // m/s2
#define N_GAIN 3.0      // guidance gain
#define M_C 47.0        // mass of projectile
#define C_D 0.2808      // drag coefficient
#define RHO 1.225       // air density
#define DIA 0.155       // diameter of projectile
#define S_REF (3.14159265358979323846 * DIA * DIA / 4.0)

/* Guidance parameters (matching MATLAB exactly) */
#define PARAM_A 0.100001
#define PARAM_B 0.100001
#define PARAM_M 0.90001
#define PARAM_N 1.30001
#define PARAM_D 7.5
#define PARAM_K 21.00000001
#define SIGMA_MAX_RAD (73.0 * 3.14159265358979323846 / 180.0)

/* Simulation parameters */
#define DT 0.01
#define MAX_SIM_TIME 200.0
#define MIN_VEL 1.0
#define R_LOOP3_START 100.0

/* GT3 flag parameters */
#define GT3_THRESHOLD 3.5
#define GT3_CONSECUTIVE 3

/* Epsilon for division protection */
#define EPS 0.0001

/* Function prototype */
void onboard_guidance_algorithm(double lat_o, double lon_o, double alt_o,
                                double lat_t, double lon_t, double alt_t,
                                double x_0_ecef, double y_0_ecef, double z_0_ecef,
                                double vx_0_ecef, double vy_0_ecef, double vz_0_ecef,
                                double theta_f, double psi_f,
                                FILE* csv_file);

#endif

