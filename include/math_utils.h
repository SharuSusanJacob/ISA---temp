/******************************************************************************
* ISA Flight Software
* @file math_utils.h
* @brief Consolidated Mathematical Utilities that is common for all the Flight Software submodules
* @details Protected math functions, vector operations, angle conversion, quaternion operations, coordinate transformations
* @author Ananthu Dev, Project Engineer / Integrator, Spacelabs
* @date 2025
* @version 2.0.1
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>
#include <stdbool.h>

// Mathematical constants
#define MATH_PI 3.14159265358979323846
#define MATH_DEG_TO_RAD (MATH_PI / 180.0)
#define MATH_RAD_TO_DEG (180.0 / MATH_PI)

#define MATH_EPSILON 0.000000000001
/* Fixed dimensionless integration step size */
#define INTEGRATION_STEP_SIZE 0.01

// Euler angles structure
typedef struct
{
    double roll_rad;
    double pitch_rad;
    double yaw_rad;
} EulerAngles_t;

// Vector3 structure
typedef struct
{
    double x;
    double y;
    double z;
} Vector3_t;

// Quaternion structure
typedef struct
{
    double w;
    double x;
    double y;
    double z;
} Quaternion_t;

// Geodetic Position structure
typedef struct
{
    double lat_deg; // Latitude in degrees
    double lon_deg; // Longitude in degrees
    double alt_m;   // Altitude above WGS84 ellipsoid in meters
} GeodeticPos_t;

// WGS84 Earth Model constants
#define WGS84_SEMI_MAJOR_AXIS 6378137.0                                                      // Earth's Semi-major axis in meters
#define WGS84_FLATTENING (1.0 / 298.257223563)                                               // Earth's flattening
#define WGS84_ECCENTRICITY_SQ (2.0 * WGS84_FLATTENING - WGS84_FLATTENING * WGS84_FLATTENING) // Square of eccentricity

// ===== Vector3 Operations =====
void vector3_add(Vector3_t *result, const Vector3_t *v1, const Vector3_t *v2);
void vector3_subtract(Vector3_t *result, const Vector3_t *v1, const Vector3_t *v2);
void vector3_cross(Vector3_t *result, const Vector3_t *v1, const Vector3_t *v2);
void vector3_dot(double *result, const Vector3_t *v1, const Vector3_t *v2);
void vector3_scale(Vector3_t *result, const Vector3_t *v, double scale);
void vector3_magnitude(double *result, const Vector3_t *v);
void vector3_normalize(Vector3_t *result, const Vector3_t *v);

// ===== Quaternion Operations =====
void quaternion_create(Quaternion_t *result, double w, double x, double y, double z);
void quaternion_multiply(Quaternion_t *result, const Quaternion_t *q1, const Quaternion_t *q2);
void quaternion_to_rotation_matrix(double *matrix, const Quaternion_t *q);

// ===== Coordinate Transformations =====
// ECEF transformations (for GNSS input)
void geodetic_to_ecef(Vector3_t *result, const GeodeticPos_t *pos);
void eci_to_local(Vector3_t *result, const Vector3_t *r_eci,
                  const GeodeticPos_t *launch, const GeodeticPos_t *target);
void local_to_ecef_vel(Vector3_t *result, const Vector3_t *v_local,
                       const GeodeticPos_t *launch, const GeodeticPos_t *target);
void eci_to_local_vel(Vector3_t *result, const Vector3_t *v_eci,
                      const GeodeticPos_t *launch, const GeodeticPos_t *target);

// Local to Body frame transformation (for guidance to DAP interface)
void local_to_body(Vector3_t *result, const Vector3_t *v_local,
                   double theta, double psi, double phi);

// ===== Math Utilities =====
void deg_to_rad(double *result, double degrees);
void rad_to_deg(double *result, double radians);
void safe_acos(double *result, double x);
void safe_atan2(double *result, double y, double x);
void safe_sin(double *result, double x);
void safe_cos(double *result, double x);
void safe_sqrt(double *result, double x);
void sig_function(double *result, double m, double x);

// ===== Guidance Physics Models =====
/**
 * @brief ISA Standard Atmosphere model - Calculate speed of sound at altitude
 * @param result Output speed of sound (m/s)
 * @param altitude_m Altitude in meters
 */
void math_atmosphere(double *result, double altitude_m);

/**
 * @brief Mach-dependent pitch acceleration limiter
 * @param result Output limited pitch acceleration (m/s²)
 * @param mach Mach number (dimensionless)
 * @param pitch_acceleration Input pitch acceleration (m/s²)
 */
void math_pitch_limit(double *result, double mach, double pitch_acceleration);

void clamp_double(double *result, double value, double min_val, double max_val);
void double_equals(bool *result, double a, double b, double epsilon);

/**
 * @brief Modulo operation that handles negative numbers correctly
 * @param a Input value
 * @param d Divisor
 * @return Correct modulo result for positive and negative values
 */
void mod_double(double *result, double a, double d);

#endif /* MATH_UTILS_H */