/******************************************************************************
* ISA Flight Software
* @file vector3.h
* @brief 3D Vector Operations for Guidance System
* @details Provides 3D vector mathematics for aerospace calculations
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 3.0
* 
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef VECTOR3_H
#define VECTOR3_H

#include <stdint.h>
#include <stdbool.h>

// Error codes
typedef enum {
    VECTOR3_SUCCESS = 0U,
    VECTOR3_ERROR_INVALID_PARAM = 1U,
    VECTOR3_ERROR_MATH_ERROR = 2U
} Vector3Error_t;

// 3D Vector Strcuture with double precision
typedef struct {
    double x;   // X component (meters for position, m/s for velocity, m/s^2 for acceleration)
    double y;   // Y component
    double z;   // Z component
} Vector3_t;

//=============================================================================
// VECTOR CREATION AND BASIC OPERATIONS
//=============================================================================

/**
* @brief Create a 3D Vector with given components
* @param result Output Vector (must not be NULL)
* @param x X Component
* @param y Y Component
* @param z Z  Component
* @return Vector3Error_t Success or error code
*/
Vector3Error_t vector3_create(Vector3_t* result, double x, double y, double z);

/**
* @brief Add two vectors (combine velocties/accelerations)
* @param result Output Vector a+b (must not be NULL)
* @param a First Vector (must not be NULL)
* @param b Second Vector (must not be NULL)
* @return Vector3Error_t Success or Error code
*/
Vector3Error_t vector3_add(Vector3_t* result, const Vector3_t* a, const Vector3_t* b);

/**
* @brief Subtract two vectors (relative position/velocity)
* @param result output vector a - b (must not be NULL)
* @param a First Vector (must not be NULL)
* @param b Second vector (must not be NULL)
* @return Vector3Error_t Sucess or Error Code
*/
Vector3Error_t vector3_subtract(Vector3_t* result, const Vector3_t* a, const Vector3_t* b);
/**
* @brief Scale vector by scalar (multiply velocity by time)
* @param result Output Vector v * scalar (Must not be NULL)
* @param v Input vector must (must not be NULL)
* @param scalar Scaling Factor
* @return Vector3Error_t Success or error code
*/
Vector3Error_t vector3_scale(Vector3_t* result, const Vector3_t* v, double scalar);
//=============================================================================
// Guidannce CRITICAL OPERATIONS
//=============================================================================

/**
* @brief Dot Product 
* @param result Output scalar a . b (must not be NULL)
* @param a First Vector (must not be NULL)
* @param b Second Vector (must not be NULL)
* @return Vector3Error_t Success or error code
 */
Vector3Error_t vector3_dot(double* result, const Vector3_t* a, const Vector3_t* b);

/**
* @brief Cross product (anugular momentum, line of sight rate)
* Critical for guidance ; w = r x v / |r|^2 (line of sight angular rate)
* @param result Output vector a x b (must not be NULL)
* @param a First vector (must not be NULL)
* @param b Second vector (must not be NULL)
* @return Vector3Error_t Sucess or Error Code
*/
Vector3Error_t vector3_cross(Vector3_t* result, const Vector3_t* a, const Vector3_t* b);
/** 
* @brief Vector magnitude (speed, distance , acceleration magnitude)
* @param result Output magnitude  |v| (must not be NULL)
* @param v input vector (must not be NULL)
* @return Vector3Error_t Success or error code
*/
Vector3Error_t vector3_magnitude(double* result, const Vector3_t* v);
/**
* @brief Normalize vector (unit direction vectors)
* Creates unit vector: used for line-of-sight direction, velocity direction
* @param result Ouput unit vector v/|v| (must not be NULL)
* @param v Input vector (must not be NULL)
* @return Vector3Error_t Success or error code
*/
Vector3Error_t vector3_normalize(Vector3_t* result, const Vector3_t* v);

#endif /* VECTOR3_H */
