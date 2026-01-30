/******************************************************************************
* ISA Flight Software
* @file quaternion.c
* @brief Quaternion operations implementation
* @details Implements quaternion mathematics for rotations
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 1.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#include "quaternion.h"
#include "math_utils.h"
#include <math.h>

QuaternionError_t quaternion_create(Quaternion_t* result, float w, float x, float y, float z)
{
    // Parameter validation
    if (result == NULL) {
        return QUATERNION_ERROR_INVALID_PARAM;
    }
    
    // Initialize quaternion components
    result->w = w;
    result->x = x;
    result->y = y;
    result->z = z;
    
    return QUATERNION_SUCCESS;
}

QuaternionError_t quaternion_from_axis_angle(Quaternion_t* result, const Vector3_t* axis, float angle_rad)
{
    // Parameter validation
    if ((result == NULL) || (axis == NULL)) {
        return QUATERNION_ERROR_INVALID_PARAM;
    }
    
    // Calculate half angle
    float half_angle = angle_rad * 0.5f;
    float sin_half_angle, cos_half_angle;
    
    // Calculate sine and cosine of half angle
    MathUtilsError_t math_error = safe_sin(&sin_half_angle, half_angle);
    if (math_error != MATH_UTILS_SUCCESS) {
        return QUATERNION_ERROR_MATH_ERROR;
    }
    
    math_error = safe_cos(&cos_half_angle, half_angle);
    if (math_error != MATH_UTILS_SUCCESS) {
        return QUATERNION_ERROR_MATH_ERROR;
    }
    
    // Normalize axis
    Vector3_t unit_axis;
    Vector3Error_t vec_error = vector3_normalize(&unit_axis, axis);
    if (vec_error != VECTOR3_SUCCESS) {
        // If normalization fails, return identity quaternion
        result->w = 1.0f;
        result->x = 0.0f;
        result->y = 0.0f;
        result->z = 0.0f;
        return QUATERNION_SUCCESS;
    }
    
    // Create quaternion from axis-angle representation
    result->w = cos_half_angle;
    result->x = sin_half_angle * unit_axis.x;
    result->y = sin_half_angle * unit_axis.y;
    result->z = sin_half_angle * unit_axis.z;
    
    return QUATERNION_SUCCESS;
}

QuaternionError_t quaternion_to_rotation_matrix(float* matrix, const Quaternion_t* q)
{
    // Parameter validation
    if ((matrix == NULL) || (q == NULL)) {
        return QUATERNION_ERROR_INVALID_PARAM;
    }
    
    // Extract quaternion components
    float w = q->w;
    float x = q->x;
    float y = q->y;
    float z = q->z;
    
    // Calculate matrix elements exactly as MATLAB does
    // Row 1
    matrix[0] = w*w + x*x - y*y - z*z;  // (1,1)
    matrix[1] = 2.0f*(x*y - w*z);       // (1,2)
    matrix[2] = 2.0f*(x*z + w*y);       // (1,3)
    
    // Row 2
    matrix[3] = 2.0f*(x*y + w*z);       // (2,1)
    matrix[4] = w*w - x*x + y*y - z*z;  // (2,2)
    matrix[5] = 2.0f*(y*z - w*x);       // (2,3)
    
    // Row 3
    matrix[6] = 2.0f*(x*z - w*y);       // (3,1)
    matrix[7] = 2.0f*(y*z + w*x);       // (3,2)
    matrix[8] = w*w - x*x - y*y + z*z;  // (3,3)
    
    return QUATERNION_SUCCESS;
}

QuaternionError_t quaternion_rotate_vector(Vector3_t* result, const Quaternion_t* q, const Vector3_t* v)
{
    // Parameter validation
    if ((result == NULL) || (q == NULL) || (v == NULL)) {
        return QUATERNION_ERROR_INVALID_PARAM;
    }
    
    // Get rotation matrix from quaternion
    float matrix[9];
    QuaternionError_t quat_error = quaternion_to_rotation_matrix(matrix, q);
    if (quat_error != QUATERNION_SUCCESS) {
        return quat_error;
    }
    
    // Apply rotation: result = matrix * v
    result->x = matrix[0]*v->x + matrix[1]*v->y + matrix[2]*v->z;
    result->y = matrix[3]*v->x + matrix[4]*v->y + matrix[5]*v->z;
    result->z = matrix[6]*v->x + matrix[7]*v->y + matrix[8]*v->z;
    
    return QUATERNION_SUCCESS;
}

QuaternionError_t quaternion_multiply(Quaternion_t* result, const Quaternion_t* q1, const Quaternion_t* q2)
{
    // Parameter validation
    if ((result == NULL) || (q1 == NULL) || (q2 == NULL)) {
        return QUATERNION_ERROR_INVALID_PARAM;
    }
    
    // Hamilton product exactly as in MATLAB
    float w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    float x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
    float y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w + q1->z*q2->x;
    float z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;
    
    // Store result
    result->w = w;
    result->x = x;
    result->y = y;
    result->z = z;
    
    return QUATERNION_SUCCESS;
}

QuaternionError_t quaternion_conjugate(Quaternion_t* result, const Quaternion_t* q)
{
    // Parameter validation
    if ((result == NULL) || (q == NULL)) {
        return QUATERNION_ERROR_INVALID_PARAM;
    }
    
    // Conjugate: q* = [w, -x, -y, -z]
    result->w = q->w;
    result->x = -q->x;
    result->y = -q->y;
    result->z = -q->z;
    
    return QUATERNION_SUCCESS;
}

QuaternionError_t quaternion_normalize(Quaternion_t* result, const Quaternion_t* q)
{
    // Parameter validation
    if ((result == NULL) || (q == NULL)) {
        return QUATERNION_ERROR_INVALID_PARAM;
    }
    
    // Calculate norm
    float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    
    // Check for division by zero
    if (norm < MATH_EPSILON) {
        // Return identity quaternion if normalization fails
        result->w = 1.0f;
        result->x = 0.0f;
        result->y = 0.0f;
        result->z = 0.0f;
        return QUATERNION_SUCCESS;
    }
    
    // Normalize quaternion
    float inv_norm = 1.0f / norm;
    result->w = q->w * inv_norm;
    result->x = q->x * inv_norm;
    result->y = q->y * inv_norm;
    result->z = q->z * inv_norm;
    
    return QUATERNION_SUCCESS;
}