/******************************************************************************
* ISA Flight Software
* @file math_utils.h
* @brief Safe Mathematical Utilities for Guidance System
* @details Protected math functions for aerospace calculations
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 1.0
* 
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>
#include <stdbool.h>

// Error codes
typedef enum {
    MATH_UTILS_SUCCESS = 0U,
    MATH_UTILS_ERROR_INVALID_PARAM = 1U,
    MATH_UTILS_ERROR_DOMAIN_ERROR = 2U,
    MATH_UTILS_ERROR_MATH_ERROR = 3U
} MathUtilsError_t;

// Mathematical constants
#define MATH_PI                 3.14159265358979323846
#define MATH_DEG_TO_RAD         (MATH_PI / 180.0)
#define MATH_RAD_TO_DEG         (180.0 / MATH_PI)
#define MATH_EPSILON            1e-12

//=============================================================================
// ANGLE CONVERSION FUNCTIONS
//=============================================================================
/**
* @brief Convert degrees to radians
* @param result Output in radians (must not be NULL)
* @param degrees Input angle in degrees
* @return MathUtilsError_t Success or error code
*
* Aerospace Usage: Convert navigation angles (degrees) to guidance (radians)
*/
MathUtilsError_t deg_to_rad(double* result, double degrees);


/**
* @brief Convert radians to degrees
* @param result Output in degrees (must not be NULL)
* @param radians Input angle in radians
* @return MathUtilsError_t Success or error code
*
* Aerospace usage: Convert guidance angles to (radians) to display (degrees)
*/
MathUtilsError_t rad_to_deg(double* result, double radians);


//=============================================================================
// PROTECTED TRIGONOMETRIC FUNCTIONS
//=============================================================================

/**
* @brief safe arc cosine function (protected domain)
* @param result Output angle in radians (must not be NULL)
* @param x input value (will be clamped to [-1, 1])
* @return MathUtilsError_t Success or error code
*
* CRITICAL FOR  GUIDANCE: Used to calculate look angle σ
* Formula: σ = acos(dot(velocity_unit, line_of_sight_unit))
* Standard acos() crashes if |x| > 1 , this function clamps safely
*/
MathUtilsError_t safe_acos(double* result, double x);

/**
* @brief Safe arc tangent function (protected for zero inputs)
* @param result Output angale in radians [-π, π] (must not be NULL)
* @param y Y component
* @param x X component
* @return MathUtilsError_t Success or error code
*
* AEROSPACE USAGE: Calculta azimuth/elevation angles
* Hanales (0,0) case gracefully (return 0)
 */
MathUtilsError_t safe_atan2(double* result, double y, double x);

/**
* @brief Safe sine function
* @param result Output cosine value (must not be NULL)
* @param x Input angle in radians
* @return MathUtilsError_t Sucess or error code
*/
MathUtilsError_t safe_sin(double* result, double x);

/**
* @brief Safe Cosine function
* @param result Output sine value (must not be NULL)
* @param x input angle in radians
* @return MathUtilsError_t Success or error code
*/
MathUtilsError_t safe_cos(double* result, double x);
/**
* @brief Safe square root function
* @param result Output square root (must not be NULL)
* @param x input value (must be >= 0)
* @return MathUtilsError_t Success or error code
*/
MathUtilsError_t safe_sqrt(double* result, double x);

//=============================================================================
//COMPATIBILITY FUNCTIONS
//=============================================================================

/**
* @brief Signed power function
* @param result Output (sign(x) * |x|^m) (must not be NULL)
* @param m Power exponent
* @param x Input value
* @return MathUtilsError_t Success or error code
* y = sign(x) * abs(x)^m
* CRITICAL FOR IMPACT ANGLE CONTROL:
* Used in: δ̇ = -K * f(σ) * [(a * sig(m,δ)) + (b * sig(n,δ))]
*
* Where :
* - δ = impact angle error
* - m, n = tuning parameters (0 < m < 1, n > 1) 
* - a, b = weighting parameters
*/
MathUtilsError_t sig_function(double* result, double m, double x);

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

/**
* @brief Clamp value to range [min, max]
* @param result Output clamped values (must not be NULL)
* @param value Input value
* @param min_val Minimum alowed value
* @param max_val Maximum allowd value
* @return MathUtilsError_t Success or error code
*/
MathUtilsError_t clamp_double(double* result, double value, double min_val, double max_val);

/**
* @brief Check if two doubles are approximately equal
* @param result Output true if equal within epsilon (must not be NULL)
* @param a First value
* @param b Second value
* @param epsilon Tolerance (use MATH_EPSILON for default)
* @return MathUtilsError_t Success or error code
*/
MathUtilsError_t double_equals(bool* result, double a, double b, double epsilon);

#endif /*MATH_UTILS_H*/

