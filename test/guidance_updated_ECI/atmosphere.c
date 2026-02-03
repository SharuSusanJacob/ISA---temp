/******************************************************************************
 * ISA Flight Software
 * @file atmosphere.c
 * @brief Atmospheric Model Implementation
 * @details Implements ISA standard atmosphere model for calculating speed of sound
 * @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation
 *****************************************************************************/

#include "atmosphere.h"
#include <math.h>

/* ISA Standard Atmosphere Constants */
#define T0 288.15   /* Sea-level temperature (K) */
#define P0 101325.0 /* Sea-level pressure (Pa) */
#define RHO0 1.225  /* Sea-level density (kg/m^3) */
#define L 0.0065    /* Temperature lapse rate (K/m) */
#define R 287.05    /* Specific gas constant for air (J/kg-K) */
#define GAMMA 1.4   /* Ratio of specific heats */
#define G 9.80665   /* Gravity (m/s^2) */

double atmosphere(double z)
{
    double T;   /* Temperature at altitude */
    double a_s; /* Speed of sound */

    /* Temperature at altitude (linear decrease in troposphere) */
    T = T0 - L * z;

    /* Speed of sound calculation: a = sqrt(gamma * R * T) */
    a_s = sqrt(GAMMA * R * T);

    return a_s;
}
