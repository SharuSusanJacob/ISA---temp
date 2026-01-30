/******************************************************************************
 * ISA Flight Software
 * @file pitch_limit.h
 * @brief Pitch Acceleration Limiter Header
 * @details Defines pitch acceleration limiting function based on Mach number
 * @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation
 *****************************************************************************/

#ifndef PITCH_LIMIT_H
#define PITCH_LIMIT_H

/**
 * @brief Limit pitch acceleration based on Mach number
 * @param mach Current Mach number
 * @param input_acc Input pitch acceleration (m/s^2)
 * @return Limited pitch acceleration (m/s^2)
 */
double pitch_limit(double mach, double input_acc);

#endif /* PITCH_LIMIT_H */
