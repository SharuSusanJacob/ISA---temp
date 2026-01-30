/******************************************************************************
* ISA Flight Software
 * @file math_utils.c
 * @brief Mathematical Utility Functions Implementation
* @details Implements angle conversions and coordinate transformations
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 4.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#include "math_utils.h"

double deg2rad(double d) { return DEG2RAD(d); }
double rad2deg(double r) { return RAD2DEG(r); }

void l2b(Vector3* vec_body, const Vector3* vec_local, double theta, double psi, double phi)
{
    double cos_psi = cos(psi);
    double sin_psi = sin(psi);
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);
    
    /* Rz = [cos(psi)  sin(psi)  0; -sin(psi)  cos(psi)  0; 0  0  1] */
    /* Ry = [cos(theta)  0  -sin(theta); 0  1  0; sin(theta)  0  cos(theta)] */
    /* Rx = [1  0  0; 0  cos(phi)  sin(phi); 0  -sin(phi)  cos(phi)] */
    /* R_l2b = Rx * Ry * Rz */
    
    double R00 = cos_theta * cos_psi;
    double R01 = cos_theta * sin_psi;
    double R02 = -sin_theta;
    double R10 = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi;
    double R11 = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi;
    double R12 = sin_phi * cos_theta;
    double R20 = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
    double R21 = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
    double R22 = cos_phi * cos_theta;
    
    vec_body->x = R00 * vec_local->x + R01 * vec_local->y + R02 * vec_local->z;
    vec_body->y = R10 * vec_local->x + R11 * vec_local->y + R12 * vec_local->z;
    vec_body->z = R20 * vec_local->x + R21 * vec_local->y + R22 * vec_local->z;
}

