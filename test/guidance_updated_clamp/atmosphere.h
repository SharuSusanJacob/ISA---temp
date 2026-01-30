/******************************************************************************
 * ISA Flight Software
 * @file atmosphere.h
 * @brief Atmospheric Model Header
 * @details Defines atmosphere model function for speed of sound calculation
 * @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation
 *****************************************************************************/

#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

/**
 * @brief Calculate speed of sound at given altitude
 * @param z Altitude in meters (valid range: 0 to 11000 m)
 * @return Speed of sound in m/s
 */
double atmosphere(double z);

#endif /* ATMOSPHERE_H */
