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

// GPS Time and ECI Conversion constants
#define OMEGA_EARTH 7.2921150e-5 /* Earth rotation rate [rad/s] */
#define GPS_EPOCH_JD 2444244.5   /* Julian Date of GPS Epoch (1980-01-06 00:00:00 UTC) */

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
void ecef_to_local(Vector3_t *result, const Vector3_t *r_ecef,
                   const GeodeticPos_t *launch, const GeodeticPos_t *target);
void local_to_ecef_vel(Vector3_t *result, const Vector3_t *v_local,
                       const GeodeticPos_t *launch, const GeodeticPos_t *target);
void ecef_to_local_vel(Vector3_t *result, const Vector3_t *v_ecef,
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

// ===== ECEF to ECI Conversion (GPS Time Based) =====
/**
 * @brief Convert GPS week and TOW to Julian Date
 * @param jd Output Julian Date
 * @param gnss_week GPS week number
 * @param tow Time of Week in seconds
 * @param leap_seconds Leap seconds offset (GPS - UTC)
 */
void gps_to_julian_date(double *jd, int gnss_week, double tow, int leap_seconds);

/**
 * @brief Convert Julian Date to GMST (Greenwich Mean Sidereal Time)
 * @param gmst_rad Output GMST in radians
 * @param jd Julian Date
 */
void julian_date_to_gmst_rad(double *gmst_rad, double jd);

/**
 * @brief Convert ECEF position and velocity to ECI frame
 * @param r_eci Output ECI position
 * @param v_eci Output ECI velocity
 * @param r_ecef Input ECEF position
 * @param v_ecef Input ECEF velocity
 * @param gnss_week GPS week number
 * @param tow Time of Week in seconds
 * @param leap_seconds Leap seconds offset
 */
void ecef_to_eci(Vector3_t *r_eci, Vector3_t *v_eci,
                 const Vector3_t *r_ecef, const Vector3_t *v_ecef,
                 int gnss_week, double tow, int leap_seconds);

/**
 * @brief Convert ECI position to Local frame
 * @param r_local Output position in Local frame
 * @param r_eci Input ECI position
 * @param launch Launch point geodetic position
 * @param target Target geodetic position
 */
void eci_to_local_pos(Vector3_t *r_local, const Vector3_t *r_eci,
                      const GeodeticPos_t *launch, const GeodeticPos_t *target);

/**
 * @brief Convert ECI velocity to Local frame (no origin subtraction)
 * @param v_local Output velocity in Local frame
 * @param v_eci Input ECI velocity
 * @param launch Launch point geodetic position
 * @param target Target geodetic position
 */
void eci_to_local_vel(Vector3_t *v_local, const Vector3_t *v_eci,
                      const GeodeticPos_t *launch, const GeodeticPos_t *target);

#endif /* MATH_UTILS_H */