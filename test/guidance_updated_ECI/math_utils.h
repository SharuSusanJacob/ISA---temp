/******************************************************************************
* ISA Flight Software
 * @file math_utils.h
 * @brief Mathematical Utility Functions Header
* @details Defines mathematical constants and utility function prototypes
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 4.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>
#include "vector3.h"

#define PI 3.14159265358979323846
#define DEG2RAD(d) ((d)*PI / 180.0)
#define RAD2DEG(r) ((r)*180.0 / PI)

/* Signed power used by MATLAB sig() */
static inline double sig(double m, double x) { return (x >= 0 ? 1.0 : -1.0) * pow(fabs(x), m); }

/* Angle conversion functions */
double deg2rad(double d);
double rad2deg(double r);

/* Local to Body frame transformation (3-2-1 Euler) */
void l2b(Vector3 *vec_body, const Vector3 *vec_local, double theta, double psi, double phi);

/* Body to Local frame transformation (inverse of l2b) */
void b2l(Vector3 *vec_local, const Vector3 *vec_body, double theta, double psi, double phi);

#endif
