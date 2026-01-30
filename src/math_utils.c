/******************************************************************************
* ISA Flight Software
* @file math_utils.c
* @brief Consolidated Math Utilities Implementation
* @details Combined vector, quaternion, and coordinate transformation functions
* @author Ananthu Dev Integration / Project Engineer, Spacelabs
* @date 2025
* @version 2.0.1
*****************************************************************************/

#include "math_utils.h"
#include <math.h>
#include <string.h>

// ===== Vector3 Operations =====

void vector3_add(Vector3_t *result, const Vector3_t *v1, const Vector3_t *v2)
{
    result->x = v1->x + v2->x;
    result->y = v1->y + v2->y;
    result->z = v1->z + v2->z;
}

void vector3_subtract(Vector3_t *result, const Vector3_t *v1, const Vector3_t *v2)
{
    result->x = v1->x - v2->x;
    result->y = v1->y - v2->y;
    result->z = v1->z - v2->z;
}

void vector3_cross(Vector3_t *result, const Vector3_t *v1, const Vector3_t *v2)
{
    result->x = v1->y * v2->z - v1->z * v2->y;
    result->y = v1->z * v2->x - v1->x * v2->z;
    result->z = v1->x * v2->y - v1->y * v2->x;
}

void vector3_dot(double *result, const Vector3_t *v1, const Vector3_t *v2)
{
    *result = v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

void vector3_scale(Vector3_t *result, const Vector3_t *v, double scale)
{
    result->x = v->x * scale;
    result->y = v->y * scale;
    result->z = v->z * scale;
}

void vector3_magnitude(double *result, const Vector3_t *v)
{
    *result = sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

void vector3_normalize(Vector3_t *result, const Vector3_t *v)
{
    double mag;
    vector3_magnitude(&mag, v);

    if (mag > MATH_EPSILON)
    {
        double inv_mag = 1.0 / mag;
        result->x = v->x * inv_mag;
        result->y = v->y * inv_mag;
        result->z = v->z * inv_mag;
    }
    else
    {
        result->x = 0.0;
        result->y = 0.0;
        result->z = 0.0;
    }
}

// ===== Quaternion Operations =====

void quaternion_create(Quaternion_t *result, double w, double x, double y, double z)
{
    result->w = w;
    result->x = x;
    result->y = y;
    result->z = z;
}

void quaternion_multiply(Quaternion_t *result, const Quaternion_t *q1, const Quaternion_t *q2)
{
    result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void quaternion_to_rotation_matrix(double *matrix, const Quaternion_t *q)
{
    double xx = q->x * q->x;
    double xy = q->x * q->y;
    double xz = q->x * q->z;
    double xw = q->x * q->w;
    double yy = q->y * q->y;
    double yz = q->y * q->z;
    double yw = q->y * q->w;
    double zz = q->z * q->z;
    double zw = q->z * q->w;

    matrix[0] = 1.0 - 2.0 * (yy + zz);
    matrix[1] = 2.0 * (xy - zw);
    matrix[2] = 2.0 * (xz + yw);

    matrix[3] = 2.0 * (xy + zw);
    matrix[4] = 1.0 - 2.0 * (xx + zz);
    matrix[5] = 2.0 * (yz - xw);

    matrix[6] = 2.0 * (xz - yw);
    matrix[7] = 2.0 * (yz + xw);
    matrix[8] = 1.0 - 2.0 * (xx + yy);
}

// ===== Coordinate Transformations =====

/**
 * @brief Convert geodetic position to ECEF vector
 * @param result Output ECEF position vector (meters)
 * @param pos Input geodetic position
 */
void geodetic_to_ecef(Vector3_t *result, const GeodeticPos_t *pos)
{
    /* Convert degrees to radians for math calculations */
    double lat_rad, lon_rad;
    deg_to_rad(&lat_rad, pos->lat_deg);
    deg_to_rad(&lon_rad, pos->lon_deg);

    /* WGS84 ellipsoid calculation */
    double sin_lat, cos_lat, sin_lon, cos_lon;
    safe_sin(&sin_lat, lat_rad);
    safe_cos(&cos_lat, lat_rad);
    safe_sin(&sin_lon, lon_rad);
    safe_cos(&cos_lon, lon_rad);

    /* Calculate radius of curvature in prime vertical (N) */
    double sin_lat_squared = sin_lat * sin_lat;
    double denominator = 1.0 - (WGS84_ECCENTRICITY_SQ * sin_lat_squared);

    double sqrt_denominator;
    safe_sqrt(&sqrt_denominator, denominator);
    double N = WGS84_SEMI_MAJOR_AXIS / sqrt_denominator;

    /* Convert to ECEF coordinates (Earth-Centered Earth-Fixed) */
    result->x = (N + pos->alt_m) * cos_lat * cos_lon;
    result->y = (N + pos->alt_m) * cos_lat * sin_lon;
    result->z = ((1.0 - WGS84_ECCENTRICITY_SQ) * N + pos->alt_m) * sin_lat;
}

/**
 * @brief Build local frame axes from launch and target points
 */
static void build_local_axes(Vector3_t *x_axis, Vector3_t *y_axis, Vector3_t *z_axis,
                             const GeodeticPos_t *launch, const GeodeticPos_t *target)
{
    Vector3_t launch_ecef, target_ecef, perturb_ecef;
    GeodeticPos_t perturb_launch = *launch;
    perturb_launch.alt_m += 1.0; /* +1 m for surface normal */

    geodetic_to_ecef(&launch_ecef, launch);
    geodetic_to_ecef(&target_ecef, target);
    geodetic_to_ecef(&perturb_ecef, &perturb_launch);

    /* z axis - surface normal */
    vector3_subtract(z_axis, &perturb_ecef, &launch_ecef);
    vector3_normalize(z_axis, z_axis);

    /* x axis - from launch to target */
    vector3_subtract(x_axis, &target_ecef, &launch_ecef);
    vector3_normalize(x_axis, x_axis);

    /* y axis - completes right-handed system */
    vector3_cross(y_axis, z_axis, x_axis);
    vector3_normalize(y_axis, y_axis);
}

/**
 * @brief Convert ECEF coordinates to Local Frame
 * @param result Output Position in Local Frame (meters)
 * @param r_ecef Input ECEF position vector
 * @param launch Launch point geodetic position
 * @param target Target Geodetic position
 */
void ecef_to_local(Vector3_t *result, const Vector3_t *r_ecef, const GeodeticPos_t *launch, const GeodeticPos_t *target)
{
    Vector3_t x_axis, y_axis, z_axis;
    build_local_axes(&x_axis, &y_axis, &z_axis, launch, target);

    Vector3_t launch_ecef;
    geodetic_to_ecef(&launch_ecef, launch);

    Vector3_t rel;
    vector3_subtract(&rel, r_ecef, &launch_ecef);

    double dot_x, dot_y, dot_z;
    vector3_dot(&dot_x, &x_axis, &rel);
    vector3_dot(&dot_y, &y_axis, &rel);
    vector3_dot(&dot_z, &z_axis, &rel);

    result->x = dot_x;
    result->y = dot_y;
    result->z = dot_z;
}

/**
 * @brief Convert Local frame velocity to ECEF frame
 * @param result Output velocity in ECEF frame
 * @param v_local Input velocity in Local frame
 * @param launch Launch point geodetic position
 * @param target Target point geodetic position
 */
void local_to_ecef_vel(Vector3_t *result, const Vector3_t *v_local, const GeodeticPos_t *launch, const GeodeticPos_t *target)
{
    Vector3_t x_axis, y_axis, z_axis;
    build_local_axes(&x_axis, &y_axis, &z_axis, launch, target);

    /* Matrix multiplication R_local->ecef (basis vectors as columns) */
    result->x = x_axis.x * v_local->x + y_axis.x * v_local->y + z_axis.x * v_local->z;
    result->y = x_axis.y * v_local->x + y_axis.y * v_local->y + z_axis.y * v_local->z;
    result->z = x_axis.z * v_local->x + y_axis.z * v_local->y + z_axis.z * v_local->z;
}

/**
 * @brief Convert ECEF frame velocity to Local frame
 * @param result Output velocity in Local frame
 * @param v_ecef Input velocity in ECEF frame
 * @param launch Launch point geodetic position
 * @param target Target point geodetic position
 */
void ecef_to_local_vel(Vector3_t *result, const Vector3_t *v_ecef, const GeodeticPos_t *launch, const GeodeticPos_t *target)
{
    Vector3_t x_axis, y_axis, z_axis;
    build_local_axes(&x_axis, &y_axis, &z_axis, launch, target);

    /* Matrix multiplication R_ECEF->Local (transpose of basis vectors) */
    double dot_x, dot_y, dot_z;
    vector3_dot(&dot_x, &x_axis, v_ecef);
    vector3_dot(&dot_y, &y_axis, v_ecef);
    vector3_dot(&dot_z, &z_axis, v_ecef);

    result->x = dot_x;
    result->y = dot_y;
    result->z = dot_z;
}

/**
 * @brief Convert Local frame to Body frame using 3-2-1 Euler rotation
 * 
 * Updated to include RbtoIITMb frame correction matrix:
 *   RbtoIITMb = [1   0   0]
 *               [0   0   1]
 *               [0  -1   0]
 * 
 * This transformation correctly maps:
 *   - Body X = Local X (longitudinal axis)
 *   - Body Y = Local Z (lateral → normal swap)
 *   - Body Z = -Local Y (normal → -lateral swap)
 * 
 * @param result Output vector in Body frame
 * @param v_local Input vector in Local frame
 * @param theta Pitch angle in radians
 * @param psi Yaw angle in radians
 * @param phi Roll angle in radians
 */
void local_to_body(Vector3_t *result, const Vector3_t *v_local, double theta, double psi, double phi)
{
    double sin_theta, cos_theta, sin_psi, cos_psi, sin_phi, cos_phi;

    safe_sin(&sin_theta, theta);
    safe_cos(&cos_theta, theta);
    safe_sin(&sin_psi, psi);
    safe_cos(&cos_psi, psi);
    safe_sin(&sin_phi, phi);
    safe_cos(&cos_phi, phi);

    /* Combined rotation matrix: Rx * Ry * Rz (standard 3-2-1 Euler) */
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
    /* This swaps rows to correct the body frame orientation */
    /* Row 0: stays the same (1*R0 + 0*R1 + 0*R2) */
    /* Row 1: becomes R2 (0*R0 + 0*R1 + 1*R2) */
    /* Row 2: becomes -R1 (0*R0 + (-1)*R1 + 0*R2) */
    result->x = R00 * v_local->x + R01 * v_local->y + R02 * v_local->z;
    result->y = R20 * v_local->x + R21 * v_local->y + R22 * v_local->z;
    result->z = -R10 * v_local->x - R11 * v_local->y - R12 * v_local->z;
}

// ===== Math Utilities =====

void deg_to_rad(double *result, double degrees)
{
    *result = degrees * MATH_DEG_TO_RAD;
}

void rad_to_deg(double *result, double radians)
{
    *result = radians * MATH_RAD_TO_DEG;
}

void safe_acos(double *result, double x)
{
    double clamped_x = x;
    if (x > 1.0)
    {
        clamped_x = 1.0;
    }
    else if (x < -1.0)
    {
        clamped_x = -1.0;
    }

    *result = acos(clamped_x);
}

void safe_atan2(double *result, double y, double x)
{
    if ((fabs(x) < MATH_EPSILON) && (fabs(y) < MATH_EPSILON))
    {
        *result = 0.0;
    }
    else
    {
        *result = atan2(y, x);
    }
}

void safe_sin(double *result, double x)
{
    *result = sin(x);
}

void safe_cos(double *result, double x)
{
    *result = cos(x);
}

void safe_sqrt(double *result, double x)
{
    if (x < 0.0)
    {
        *result = 0.0;
    }
    else
    {
        *result = sqrt(x);
    }
}

void sig_function(double *result, double m, double x)
{
    if (fabs(x) < MATH_EPSILON)
    {
        *result = 0.0;
        return;
    }

    double sign = (x >= 0.0) ? 1.0 : -1.0;
    double abs_x = fabs(x);

    *result = sign * pow(abs_x, m);
}

void clamp_double(double *result, double value, double min_val, double max_val)
{
    if (value < min_val)
    {
        *result = min_val;
    }
    else if (value > max_val)
    {
        *result = max_val;
    }
    else
    {
        *result = value;
    }
}

void double_equals(bool *result, double a, double b, double epsilon)
{
    double diff = fabs(a - b);
    *result = (diff <= epsilon);
}

void mod_double(double *result, double a, double d)
{
    double m = fmod(a, d);
    if ((m < 0.0 && d > 0.0) || (m > 0.0 && d < 0.0))
    {
        m += d;
    }
    *result = m;
}

// ===== Guidance Physics Models =====

/**
 * @brief ISA Standard Atmosphere model - Calculate speed of sound at altitude
 *
 * Implements the International Standard Atmosphere (ISA) model for the troposphere.
 * Valid for altitudes up to 11,000 meters.
 *
 * Mathematical Model:
 *   T(h) = T₀ + L × h
 *   a(h) = √(γ × R × T(h))
 *
 * Where:
 *   T₀ = 288.15 K (sea level temperature)
 *   L = -0.0065 K/m (temperature lapse rate)
 *   γ = 1.4 (specific heat ratio for air)
 *   R = 287.05 J/(kg·K) (specific gas constant for air)
 */
void math_atmosphere(double *result, double altitude_m)
{
    /* ISA model constants */
    const double T0 = 288.15; /* Sea level temperature (K) */
    const double L = -0.0065; /* Temperature lapse rate (K/m) */
    const double gamma = 1.4; /* Specific heat ratio */
    const double R = 287.05;  /* Specific gas constant (J/kg·K) */

    /* Calculate temperature at altitude */
    double T = T0 + L * altitude_m;

    /* Ensure temperature doesn't go negative */
    if (T < 0.0)
    {
        T = 0.0;
    }

    /* Calculate speed of sound: a = sqrt(gamma * R * T) */
    double sqrt_arg = gamma * R * T;
    double a_s;
    safe_sqrt(&a_s, sqrt_arg);

    *result = a_s;
}

/**
 * @brief Mach-dependent pitch acceleration limiter
 *
 * Implements asymmetric pitch acceleration limits based on Mach number.
 * Uses linear interpolation between lookup table points.
 * Reference: test/guidance_updated_clamp/pitch_limit.c
 *
 * Lookup Table (Transonic Region - Asymmetric Limits):
 *   Mach | Max Acc (m/s²) | Min Acc (m/s²)
 *   -----|----------------|----------------
 *   0.4  |  0.5           | -0.3
 *   0.6  |  2.0           | -1.0
 *   0.8  |  3.0           | -2.0
 *   0.95 | 10.0           | -8.0
 *   1.05 | 12.0           | -10.0
 *   1.1  | 15.0           | -13.0
 */
void math_pitch_limit(double *result, double mach, double pitch_acceleration)
{
    /* Lookup table data points (6 points covering transonic region) */
    const int NUM_MACH_POINTS = 6;

    const double mach_data[6] = {
        0.4, 0.6, 0.8, 0.95, 1.05, 1.1};

    const double acc_max_data[6] = {
        0.5, 2.0, 3.0, 10.0, 12.0, 15.0};

    const double acc_min_data[6] = {
        -0.3, -1.0, -2.0, -8.0, -10.0, -13.0};

    double acc_max;
    double acc_min;
    double output_acc;
    int idx_below;
    int idx_above;
    double mach_below;
    double mach_above;
    double t;
    int i;

    /* Handle cases outside the table range */
    if (mach <= mach_data[0])
    {
        /* Below minimum Mach in table */
        acc_max = acc_max_data[0];
        acc_min = acc_min_data[0];
    }
    else if (mach >= mach_data[NUM_MACH_POINTS - 1])
    {
        /* Above maximum Mach in table */
        acc_max = acc_max_data[NUM_MACH_POINTS - 1];
        acc_min = acc_min_data[NUM_MACH_POINTS - 1];
    }
    else
    {
        /* Find bracketing indices for interpolation */
        idx_below = 0;
        idx_above = NUM_MACH_POINTS - 1;

        for (i = 0; i < NUM_MACH_POINTS; i++)
        {
            if (mach_data[i] <= mach)
            {
                idx_below = i;
            }
            if (mach_data[i] >= mach && idx_above == NUM_MACH_POINTS - 1)
            {
                idx_above = i;
            }
        }

        /* Check if exact match */
        if (idx_below == idx_above)
        {
            acc_max = acc_max_data[idx_below];
            acc_min = acc_min_data[idx_below];
        }
        else
        {
            /* Linear interpolation */
            mach_below = mach_data[idx_below];
            mach_above = mach_data[idx_above];

            /* Interpolation parameter */
            t = (mach - mach_below) / (mach_above - mach_below);

            /* Interpolate max and min limits */
            acc_max = acc_max_data[idx_below] + t * (acc_max_data[idx_above] - acc_max_data[idx_below]);
            acc_min = acc_min_data[idx_below] + t * (acc_min_data[idx_above] - acc_min_data[idx_below]);
        }
    }

    /* Apply asymmetric limits */
    if (pitch_acceleration > acc_max)
    {
        output_acc = acc_max;
    }
    else if (pitch_acceleration < acc_min)
    {
        output_acc = acc_min;
    }
    else
    {
        output_acc = pitch_acceleration;
    }

    *result = output_acc;
}
