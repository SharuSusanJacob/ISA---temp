/******************************************************************************
 * ISA Flight Software
 *
 * File: vector3.h
 * Description: 3D vector operations for the guidance system
 *****************************************************************************/

#ifndef VECTOR3_H
#define VECTOR3_H

#include "types.h"

 /**
  * @brief 3D vector structure
  *
  * In the MATLAB code, vectors are represented as [x; y; z]
  * We'll use a, b, c components to match the reference C++ code
  */
typedef struct {
    double a;  /* x component */
    double b;  /* y component */
    double c;  /* z component */
} Vector3;

/**
 * @brief Initialize a Vector3 with given components
 *
 * @param a The x component
 * @param b The y component
 * @param c The z component
 * @return Vector3 The initialized vector
 */
Vector3 Vector3_Create(double a, double b, double c);

/**
 * @brief Add two vectors
 *
 * @param v1 First vector
 * @param v2 Second vector
 * @return Vector3 Result of v1 + v2
 */
Vector3 Vector3_Add(Vector3 v1, Vector3 v2);

/**
 * @brief Subtract two vectors
 *
 * @param v1 First vector
 * @param v2 Second vector
 * @return Vector3 Result of v1 - v2
 */
Vector3 Vector3_Subtract(Vector3 v1, Vector3 v2);

/**
 * @brief Multiply a vector by a scalar
 *
 * @param v The vector
 * @param scalar The scalar value
 * @return Vector3 Result of scalar * v
 */
Vector3 Vector3_ScalarMultiply(Vector3 v, double scalar);

/**
 * @brief Calculate the dot product of two vectors
 *
 * @param v1 First vector
 * @param v2 Second vector
 * @return double The dot product v1 · v2
 */
double Vector3_DotProduct(Vector3 v1, Vector3 v2);

/**
 * @brief Calculate the cross product of two vectors
 *
 * @param v1 First vector
 * @param v2 Second vector
 * @return Vector3 The cross product v1 × v2
 */
Vector3 Vector3_CrossProduct(Vector3 v1, Vector3 v2);

/**
 * @brief Calculate the magnitude (norm) of a vector
 *
 * @param v The vector
 * @return double The magnitude ||v||
 */
double Vector3_Norm(Vector3 v);

/**
 * @brief Normalize a vector (make it unit length)
 *
 * @param v The vector to normalize
 * @return Vector3 The normalized vector v/||v||
 */
Vector3 Vector3_Normalize(Vector3 v);

/**
 * @brief Divide a vector by a scalar
 *
 * @param v The vector
 * @param scalar The scalar value (must not be zero)
 * @return Vector3 Result of v / scalar
 */
Vector3 Vector3_ScalarDivide(Vector3 v, double scalar);

/**
 * @brief Print a vector to stdout (for debugging)
 *
 * @param v The vector to print
 * @param name Optional name to display
 */
void Vector3_Print(Vector3 v, const char* name);

#endif /* VECTOR3_H */
