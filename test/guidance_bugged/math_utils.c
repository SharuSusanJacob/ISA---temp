/******************************************************************************
* ISA Flight Software
* @file math_utils.c
* @brief Safe Mathematical Utilities Implementation
* @details Implementation of protected math functions for aerospace guidance
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 3.0
* 
* MISRA C: Compliant Implementation
*****************************************************************************/
#include "math_utils.h"
#include <math.h>
#include <string.h>

//=============================================================================
// ANGLE CONVERSION FUNCTIONS
//=============================================================================

MathUtilsError_t deg_to_rad(double* result, double degrees)
{
    //Parameter Validation (MISRA C pattern)
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }
    // Conversion : Degrees to Radians = degress * (π/180)
    // COnvert navigation angles (degress) to guidance calculatios (radians)
    *result = degrees * MATH_DEG_TO_RAD;

    return MATH_UTILS_SUCCESS;
}

MathUtilsError_t rad_to_deg(double* result, double radians)
{
    //Parameter Validation
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }
    // Conversion : Radians to Degrees = radians * (180/π)
    // COnvert guidance resutls (radians) to display (degrees)
    *result = radians * MATH_RAD_TO_DEG;

    return MATH_UTILS_SUCCESS;
}

//=============================================================================
// TRIGONOMETRIC FUNCTIONS
//=============================================================================

MathUtilsError_t safe_acos(double* result, double x)
{
    // Parameter Validation
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }

    // Critical code safety : clamp input to valid domain [-1, 1]
    // Standard acos() crashes if |x| > 1, this prevents that
    //
    // Used to calculate look angle σ
    // Formula: σ = acos(dot(velocity_unit, line_of_sight_unit))
    // Due to doubleing point errors, dot product might be 1.000000
    // Which would crash standard acos()

    double clamped_x = x;
    if (x > 1.0) {
        clamped_x = 1.0;
    } else if (x < -1.0) {
        clamped_x = -1.0;
    }

    *result = acos(clamped_x);
    return MATH_UTILS_SUCCESS;
}
MathUtilsError_t safe_atan2(double* result, double y, double x)
{
    // Parameter validation
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }

    // Handle the special case (0,0) gracefullly
    // Standard atan2(0,0) behaviour is undefined in some implementations
    if ((fabs(x) < MATH_EPSILON) && (fabs(y) < MATH_EPSILON)) {
        *result = 0.0; // Return 0 for (0,0) case
        return MATH_UTILS_SUCCESS;
    }
    // Safe to call standard atan2
    // Calculate azimuth angles, bearing to target
    *result = atan2(y,x);

    return MATH_UTILS_SUCCESS;
}

MathUtilsError_t safe_sin(double* result, double x)
{
    // Parameter validation
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }
    // sin is mathematically safe for all inputs
    *result = sin(x);

    return MATH_UTILS_SUCCESS;
}

MathUtilsError_t safe_cos(double* result, double x)
{
    // Parameter validation
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }
    // Cos is mathematically safe for all inputs
    *result = cos(x);

    return MATH_UTILS_SUCCESS;
}

MathUtilsError_t safe_sqrt(double* result, double x)
{
    // Parameter validation
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }

    // Safety check : sqrt of negative number is undefined
    if (x < 0.0) {
        return MATH_UTILS_ERROR_DOMAIN_ERROR;
    }
    *result = sqrt(x);
    return MATH_UTILS_SUCCESS;
}

//=============================================================================
// COMPATIBILITY FUNCTIONS
//=============================================================================

MathUtilsError_t sig_function(double* result, double m, double x)
{
    // Parameter validation
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }
        // MATLAB EQUIVALENT: y = sign(x) * abs(x)^m
    // 
    // *** THIS IS CRITICAL FOR IMPACT ANGLE CONTROL ***
    // 
    // Used in the guidance law:
    // δ̇ = -K * f(σ) * [(a * sig(m,δ)) + (b * sig(n,δ))]
    // 
    // Where:
    // - δ = impact angle error (difference between desired and current impact angle)
    // - m, n = tuning parameters (typically m=0.9, n=1.3)
    // - a, b = weighting parameters (typically 0.1)
    // - K = gain parameter (typically 21)
    // 
    // The sig function creates a "shaped" response:
    // - For small errors (m < 1): gentler response
    // - For large errors (n > 1): stronger response
    // This gives smooth control near the target impact angle
    //

    // Handle zero case
    if (fabs(x) < MATH_EPSILON) {
        *result = 0.0;
        return MATH_UTILS_SUCCESS;
    }

    // Safety check for power operation
    if (m < 0.0) {
        return MATH_UTILS_ERROR_DOMAIN_ERROR;
    }

    // calculate: sign(x) * |x|^m
    double sign = (x >= 0.0) ? 1.0 : -1.0;
    double abs_x = fabs(x);

    //Use pow for the power operation
    *result = sign * pow(abs_x, m);

    return MATH_UTILS_SUCCESS;
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

MathUtilsError_t clamp_double(double* result, double value, double min_val, double max_val)
{
    // Parameter validation
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }

    // Validate min <= max
    if (min_val > max_val) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }

    // Clamp the value to the specific range
    // Used to limit actuator commands, angle ranges etc
    if (value < min_val) {
        *result = min_val;
    } else if (value > max_val) {
        *result = max_val;
    } else {
        *result = value;
    }
    return MATH_UTILS_SUCCESS;
}

MathUtilsError_t double_equals(bool* result, double a, double b, double epsilon)

{
    // Parameter validation
    if (result == NULL) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }

    // Safety check for epsilon
    if (epsilon < 0.0) {
        return MATH_UTILS_ERROR_INVALID_PARAM;
    }

    // compare doubles with tolerance
    // Essential for comparing calcualted vs expected values
    double diff = fabs(a-b);
    *result = (diff <= epsilon);

    return MATH_UTILS_SUCCESS;
}