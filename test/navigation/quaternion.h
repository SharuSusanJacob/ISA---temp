/******************************************************************************
* ISA Flight Software
* @file quaternion.h
* @brief Quaternion operations for attitude representation
* @details Implements quaternion mathematics for rotations
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 1.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef QUATERNION_H
#define QUATERNION_H

#include "vector3.h"

// Error codes
typedef enum {
    QUATERNION_SUCCESS = 0U,
    QUATERNION_ERROR_INVALID_PARAM = 1U,
    QUATERNION_ERROR_MATH_ERROR = 2U
} QuaternionError_t;

// Quaternion structure (w + xi + yj + zk)
typedef struct {
    float w;  // Scalar part
    float x;  // Vector part (i)
    float y;  // Vector part (j)
    float z;  // Vector part (k)
} Quaternion_t;

// Create quaternion from scalar and vector parts
QuaternionError_t quaternion_create(Quaternion_t* result, float w, float x, float y, float z);

// Create quaternion from axis-angle representation
QuaternionError_t quaternion_from_axis_angle(Quaternion_t* result, const Vector3_t* axis, float angle_rad);

// Get rotation matrix from quaternion (3x3 matrix as array[9])
QuaternionError_t quaternion_to_rotation_matrix(float* matrix, const Quaternion_t* q);

// Rotate vector by quaternion (v' = q * v * q^(-1))
QuaternionError_t quaternion_rotate_vector(Vector3_t* result, const Quaternion_t* q, const Vector3_t* v);

// Quaternion multiplication (q1 * q2)
QuaternionError_t quaternion_multiply(Quaternion_t* result, const Quaternion_t* q1, const Quaternion_t* q2);

// Quaternion conjugate (q* = [w, -x, -y, -z])
QuaternionError_t quaternion_conjugate(Quaternion_t* result, const Quaternion_t* q);

// Quaternion normalization
QuaternionError_t quaternion_normalize(Quaternion_t* result, const Quaternion_t* q);

#endif /* QUATERNION_H */