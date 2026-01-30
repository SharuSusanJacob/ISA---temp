/******************************************************************************
* ISA Flight Software
* @file vector3.c
* @brief 3D Vector Operations Implementation
* @details Implementation of vector mathematics for aerospace guidance
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 3.0
* 
* MISRA C: Compliant Implementation
*****************************************************************************/
#include "vector3.h"
#include <math.h>
#include <string.h>

// Safety epsilon for doubleing poiint comparisons
#define VECTOR3_EPSILON 1e-12

//=============================================================================
// BASIC VECTOR OPERATIONS
//=============================================================================

Vector3Error_t vector3_create(Vector3_t* result, double x, double y, double z)
{
    // Misra C rule: always validate parameters first
    if (result == NULL) {
        return VECTOR3_ERROR_INVALID_PARAM;
    }

    // Initialize the vector components
    result->x = x;
    result->y = y;
    result->z = z;

    return VECTOR3_SUCCESS;
}
Vector3Error_t vector3_add(Vector3_t* result, const Vector3_t* a, const Vector3_t* b)
{
    // Parameter valiation - critical for flight software
    if ((result == NULL) || (a == NULL) || (b == NULL)) {
        return VECTOR3_ERROR_INVALID_PARAM;
    }
    
    // Vectotr addition: result = a + b
    // Combine velocities, accelerations , forces
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;

    return VECTOR3_SUCCESS;
}

Vector3Error_t vector3_subtract(Vector3_t* result, const Vector3_t* a, const Vector3_t* b)
{
    // Parameter valiation
    if ((result == NULL) || (a == NULL) || (b == NULL)) {
        return VECTOR3_ERROR_INVALID_PARAM;
    }

    // Vector subtraction: result = a - b
    // Find relative position (target_pos - projectile_pos)
    // from projectile to target
    result->x = a->x - b->x;
    result->y = a->y - b->y;
    result->z = a->z - b->z;

    return VECTOR3_SUCCESS;
}

Vector3Error_t vector3_scale(Vector3_t* result, const Vector3_t* v, double scalar)
{
    // parameter validation
    if ((result == NULL) || (v == NULL)) {
        return VECTOR3_ERROR_INVALID_PARAM;
    }

    // Scalar multiplication: result = v * scalar
    // Scale velocity by time, scale unit vector by magnitude
    result->x = v->x * scalar;
    result->y = v->y * scalar;
    result->z = v->z * scalar;

    return VECTOR3_SUCCESS;
}

//=============================================================================
// Guidance CRITICAL OPERATIONS
//=============================================================================

Vector3Error_t vector3_dot(double* result, const Vector3_t* a, const Vector3_t* b)
{
    // Parameter validation
    if ((result == NULL) || (a == NULL) || (b == NULL)) {
        return VECTOR3_ERROR_INVALID_PARAM;
    }

    // Dot product: a.b = ax*bx + ax*by + az*bz
    // - Gices projectio  of one vector onto another
    // - used to calculate angles : cos(theta) = a.b / (|a| * |b|)
    *result = (a->x * b->x) + (a->y * b->y) + (a->z * b->z);

    return VECTOR3_SUCCESS;
}

Vector3Error_t vector3_cross(Vector3_t* result, const Vector3_t* a, const Vector3_t* b)
{
    // Parameter validation
    if ((result == NULL) || a == NULL || (b == NULL)) {
        return VECTOR3_ERROR_INVALID_PARAM;
    }
    // Cross product: a × b = (ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx)
    // 
    // *** THIS IS THE HEART OF PROPORTIONAL NAVIGATION ***
    // 
    // In guidance, we compute: ω = (R × V) / |R|²
    // Where:
    // - R = position vector from projectile to target
    // - V = projectile velocity vector
    // - ω = line-of-sight angular rate vector
    // 
    // The line-of-sight angular rate tells us how fast the target direction
    // is changing. If it's zero, we're on a collision course!
    // If it's non-zero, we need to apply proportional navigation.
    result->x = (a->y * b->z) - (a->z * b->y);
    result->y = (a->z * b->x) - (a->x * b->z);
    result->z = (a->x * b->y) - (a->y * b->x);

    return VECTOR3_SUCCESS;
}

Vector3Error_t vector3_magnitude(double* result, const Vector3_t* v)
{
    // Parameter valiation
    if ((result == NULL) || (v == NULL)) {
        return VECTOR3_ERROR_INVALID_PARAM;
    }
    // Magnitude: |v| = √(vx² + vy² + vz²)
    // Speed (velocity magnitude), distance (position magnitude)
    double mag_squared = (v->x * v->x) + (v->y * v->y) + (v->z * v->z);

    // Safety check for negative values (shouln't happen, but flight software safety)
    if (mag_squared < 0.0) {
        return VECTOR3_ERROR_MATH_ERROR;
    }

    *result = sqrt(mag_squared);

    return VECTOR3_SUCCESS;
}

Vector3Error_t vector3_normalize(Vector3_t* result, const Vector3_t* v)
{
    // Parameter validation
    if ((result == NULL) || (v == NULL)) {
        return VECTOR3_ERROR_INVALID_PARAM;
    }

    // First, get the magnitude
    double magnitude;
    Vector3Error_t error = vector3_magnitude(&magnitude, v);
    if (error != VECTOR3_SUCCESS) {
        return error;
    }


// Safety check : Don't divide by zero (critical for flight sofware)
if (magnitude < VECTOR3_EPSILON) {
    // Zero vector cannot be normalized
    return VECTOR3_ERROR_MATH_ERROR;
}
    // Normalize: result = v / |v|
    // Creates unit vector (magnitude = 1) pointing in same direction
    // 
    // AEROSPACE SIGNIFICANCE:
    // - Line-of-sight unit vector: direction from projectile to target
    // - Velocity unit vector: direction of motion
    // - Used in guidance law calculations
    result->x = v->x / magnitude;
    result->y = v->y / magnitude;
    result->z = v->z / magnitude;

    return VECTOR3_SUCCESS;
}