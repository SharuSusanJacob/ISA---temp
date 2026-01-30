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

void l2b(Vector3 *vec_body, const Vector3 *vec_local, double theta, double psi, double phi)
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
    /* RbtoIITMb = [1 0 0; 0 0 1; 0 -1 0] */
    /* R_l2b = RbtoIITMb * (Rx * Ry * Rz) */

    /* First compute Rx * Ry * Rz */
    double R00 = cos_theta * cos_psi;
    double R01 = cos_theta * sin_psi;
    double R02 = -sin_theta;
    double R10 = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi;
    double R11 = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi;
    double R12 = sin_phi * cos_theta;
    double R20 = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
    double R21 = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
    double R22 = cos_phi * cos_theta;

    /* Apply RbtoIITMb transformation: [1 0 0; 0 0 1; 0 -1 0] */
    /* Row 0: stays the same (1*R0 + 0*R1 + 0*R2) */
    /* Row 1: becomes R2 (0*R0 + 0*R1 + 1*R2) */
    /* Row 2: becomes -R1 (0*R0 + (-1)*R1 + 0*R2) */

    vec_body->x = R00 * vec_local->x + R01 * vec_local->y + R02 * vec_local->z;
    vec_body->y = R20 * vec_local->x + R21 * vec_local->y + R22 * vec_local->z;
    vec_body->z = -R10 * vec_local->x - R11 * vec_local->y - R12 * vec_local->z;
}

void b2l(Vector3 *vec_local, const Vector3 *vec_body, double theta, double psi, double phi)
{
    double cos_psi = cos(psi);
    double sin_psi = sin(psi);
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    /* R_b2l = (RbtoIITMb * (Rx * Ry * Rz))^T */
    /* This is the transpose of the l2b rotation matrix */

    /* First compute Rx * Ry * Rz (same as in l2b) */
    double R00 = cos_theta * cos_psi;
    double R01 = cos_theta * sin_psi;
    double R02 = -sin_theta;
    double R10 = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi;
    double R11 = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi;
    double R12 = sin_phi * cos_theta;
    double R20 = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
    double R21 = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
    double R22 = cos_phi * cos_theta;

    /* After RbtoIITMb transformation, the matrix becomes:
     * [R00  R01  R02]
     * [R20  R21  R22]
     * [-R10 -R11 -R12]
     * 
     * Then we transpose to get b2l:
     * [R00  R20  -R10]
     * [R01  R21  -R11]
     * [R02  R22  -R12]
     */

    vec_local->x = R00 * vec_body->x + R20 * vec_body->y - R10 * vec_body->z;
    vec_local->y = R01 * vec_body->x + R21 * vec_body->y - R11 * vec_body->z;
    vec_local->z = R02 * vec_body->x + R22 * vec_body->y - R12 * vec_body->z;
}
