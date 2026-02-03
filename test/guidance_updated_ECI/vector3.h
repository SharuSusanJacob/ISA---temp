/******************************************************************************
* ISA Flight Software
 * @file vector3.h
 * @brief 3D Vector Operations Header
* @details Defines Vector3 structure and vector operation function prototypes
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 4.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef VECTOR3_H
#define VECTOR3_H

typedef struct { double x, y, z; } Vector3;

void v3_add(Vector3* r, const Vector3* a, const Vector3* b);
void v3_sub(Vector3* r, const Vector3* a, const Vector3* b);
void v3_cross(Vector3* r, const Vector3* a, const Vector3* b);
double v3_dot(const Vector3* a, const Vector3* b);
void v3_scale(Vector3* r, const Vector3* a, double s);
double v3_norm(const Vector3* a);
void v3_normed(Vector3* r, const Vector3* a);

#endif

