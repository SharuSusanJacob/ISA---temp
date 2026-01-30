/********************************************************************************
* ISA Flight Software
 * @file coordinate_transforms.c
* @brief Coordinate Frame Transformations Implementation
* @details Implementation of transformations between Geodetic, ECEF, and Local frames
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 3.0.1
*
* MISRA C: Compliant Implementation
*****************************************************************************/


#include "coordinate_transforms.h"
#include "math_utils.h"
#include <math.h>
#include <string.h>


//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

CoordTransformError_t geodetic_create(GeodeticPos_t* result, double lat_deg, double lon_deg, double alt_m)
{
    // Parameter validation (MISRA C standard)
    if (result == NULL) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // Validate latitude range [-90, +90]
    if ((lat_deg < -90.0) || (lat_deg > 90.0)) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // Validate longitude range [-180, +180]
    if ((lon_deg < -180.0) || (lon_deg > 180.0)) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // Create the geodetic postion
    result->lat_deg = lat_deg;
    result->lon_deg = lon_deg;
    result->alt_m = alt_m;

    return COORD_TRANSFORM_SUCCESS;
}

//=============================================================================
// GEODETIC ↔ ECEF TRANSFORMATIONS
//=============================================================================

CoordTransformError_t geodetic_to_ecef(Vector3_t* result, const GeodeticPos_t* pos)
{
    // Parameter validation (MISRA C standard)
    if ((result == NULL) || (pos == NULL)) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // Convert degrees to radians for math calculations
    double lat_rad, lon_rad;
    MathUtilsError_t math_error;

    math_error = deg_to_rad(&lat_rad, pos->lat_deg);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }
    math_error = deg_to_rad(&lon_rad, pos->lon_deg);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }


    // WGS84 ellipsoid calculation
    // Because earth is not a perfect sphere
    // AEROSPACE Significance: Earth is an oblate ellipsoid
    // Flattened at poles, bulged at equator
    // Because GPS coordinates follow this ellipsoid
    // Critical for precision guidance accuracy

    double sin_lat, cos_lat, sin_lon, cos_lon;

    math_error = safe_sin(&sin_lat, lat_rad);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    math_error = safe_cos(&cos_lat, lat_rad);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    math_error = safe_sin(&sin_lon, lon_rad);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    math_error = safe_cos(&cos_lon, lon_rad);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Calculate radius of curvature in prime vertical (N)
    // This varies with latitude due to earth's ellipsoid shape
    double sin_lat_squared = sin_lat * sin_lat;
    double denominator = 1.0 - (WGS84_ECCENTRICITY_SQ * sin_lat_squared);

    // Safety check for division
    if (denominator <= 0.0) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    double sqrt_denominator;
    math_error = safe_sqrt(&sqrt_denominator, denominator);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    double N = WGS84_SEMI_MAJOR_AXIS / sqrt_denominator;

    // Convert to ECEF coordinates (Earth-Centered Earth-Fixed)
    result->x = (N + pos->alt_m) * cos_lat * cos_lon;
    result->y = (N + pos->alt_m) * cos_lat * sin_lon;
    result->z = ((1.0 - WGS84_ECCENTRICITY_SQ) * N + pos->alt_m) * sin_lat;

    return COORD_TRANSFORM_SUCCESS;
}

CoordTransformError_t ecef_to_geodetic(GeodeticPos_t* result, const Vector3_t* r_ecef)
{
    // Parameter validation
    if ((result == NULL) || (r_ecef == NULL)) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // Extract ECEF Coordinates
    double X = r_ecef->x;
    double Y = r_ecef->y;
    double Z = r_ecef->z;

    // Calculate the longitude
    double lon_rad;
    MathUtilsError_t math_error = safe_atan2(&lon_rad, Y, X);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Calculate latitude and altitude (iterative process)
    double r;
    math_error = safe_sqrt(&r, (X * X) + (Y * Y));
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Initial guess for latitude
    double lat_rad;
    math_error = safe_atan2(&lat_rad, Z, r * (1.0 - WGS84_ECCENTRICITY_SQ));
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Iterative refinement (5 iterations sufficient for precisiin)
    double N, alt;
    for (int i = 0; i < 5; i++) {
        double sin_lat, cos_lat;

        math_error = safe_sin(&sin_lat, lat_rad);
        if (math_error != MATH_UTILS_SUCCESS) {
            return COORD_TRANSFORM_ERROR_MATH_ERROR;
        }

        math_error = safe_cos(&cos_lat, lat_rad);
        if (math_error != MATH_UTILS_SUCCESS) {
            return COORD_TRANSFORM_ERROR_MATH_ERROR;
        }

        double sin_lat_squared = sin_lat * sin_lat;
        double denominator = 1.0 - (WGS84_ECCENTRICITY_SQ * sin_lat_squared);

        if (denominator <= 0.0) {
            return COORD_TRANSFORM_ERROR_MATH_ERROR;
        }

        double sqrt_denominator;
        math_error = safe_sqrt(&sqrt_denominator, denominator);
        if (math_error != MATH_UTILS_SUCCESS) {
            return COORD_TRANSFORM_ERROR_MATH_ERROR;
        }

        N = WGS84_SEMI_MAJOR_AXIS / sqrt_denominator;

        // safety check for division by zero
        if (fabs(cos_lat) < MATH_EPSILON) {
            return COORD_TRANSFORM_ERROR_MATH_ERROR;
        }

        alt = (r / cos_lat) - N;

        // Update latitude estimate
        math_error = safe_atan2(&lat_rad, Z + (WGS84_ECCENTRICITY_SQ * N * sin_lat), r);
        if (math_error != MATH_UTILS_SUCCESS) {
            return COORD_TRANSFORM_ERROR_MATH_ERROR;
        }
    }

    // Convert back to degrees
    math_error = rad_to_deg(&result->lat_deg, lat_rad);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    math_error = rad_to_deg(&result->lon_deg, lon_rad);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    result->alt_m = alt;

    return COORD_TRANSFORM_SUCCESS;

}

//=============================================================================
// ECEF ↔ LOCAL TRANSFORMATIONS
//=============================================================================

CoordTransformError_t ecef_to_local(Vector3_t* result, const Vector3_t* r_ecef, const GeodeticPos_t* launch, const GeodeticPos_t* target)
{
    // Parameter validation
    if ((result == NULL) || (r_ecef == NULL) || (launch == NULL) || (target == NULL)) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // Step 1: Convert geodetic (lat, lon, alt) to ECEF

    // Launch point (origin)
    Vector3_t launch_ecef;
    CoordTransformError_t coord_error = geodetic_to_ecef(&launch_ecef, launch);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    // Target
    Vector3_t target_ecef;
    coord_error = geodetic_to_ecef(&target_ecef, target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    // Step 2: Compute the Local Frame Basis Vectors
    // Z-axis: Points radially outward from Earth's surface (normal to ellipsoid)
    GeodeticPos_t perturbed_launch = *launch;
    perturbed_launch.alt_m += 1.0;  // Perturbed point for normal
    Vector3_t perturbed_ecef;
    coord_error = geodetic_to_ecef(&perturbed_ecef, &perturbed_launch);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    Vector3_t z_local_ecef;
    Vector3Error_t vec_error = vector3_subtract(&z_local_ecef, &perturbed_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&z_local_ecef, &z_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // X-axis: Points from launch point to target (in ECEF)
    Vector3_t x_local_ecef;
    vec_error = vector3_subtract(&x_local_ecef, &target_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&x_local_ecef, &x_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Y-axis: Completes the right-handed system (cross product of Z and X)
    Vector3_t y_local_ecef;
    vec_error = vector3_cross(&y_local_ecef, &z_local_ecef, &x_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&y_local_ecef, &y_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Step 3: Construct Rotation Matrix (ECEF → Local)
    // R_ECEF_to_Local = [x_local_ecef, y_local_ecef, z_local_ecef]'
    // This is the transpose of the matrix with basis vectors as columns

    // Step 4: Apply Rotation to the ECEF Vector (LAUNCH-CENTERED)
    // vec_local = R_ECEF_to_Local * (vec_ecef - launch_ecef)
    Vector3_t relative_ecef;
    vec_error = vector3_subtract(&relative_ecef, r_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Matrix multiplication: result = R_transpose * relative_ecef
    double dot_x, dot_y, dot_z;
    vec_error = vector3_dot(&dot_x, &x_local_ecef, &relative_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_dot(&dot_y, &y_local_ecef, &relative_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_dot(&dot_z, &z_local_ecef, &relative_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    result->x = dot_x;  // X component in local frame
    result->y = dot_y;  // Y component in local frame
    result->z = dot_z;  // Z component in local frame

    return COORD_TRANSFORM_SUCCESS;
}

CoordTransformError_t local_to_ecef(Vector3_t* result, const Vector3_t* r_local, const GeodeticPos_t* launch, const GeodeticPos_t* target)
{
    // Parameter Validation (MISRA C)
    if ((result == NULL) || (r_local == NULL) || (launch == NULL) || (target == NULL)) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // Step 1: Convert geodetic positions to ECEF
    Vector3_t launch_ecef;
    CoordTransformError_t coord_error = geodetic_to_ecef(&launch_ecef, launch);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    Vector3_t target_ecef;
    coord_error = geodetic_to_ecef(&target_ecef, target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    // Step 2: Compute the Local Frame Basis Vectors
    // Z-axis: Points radially outward from Earth's surface
    GeodeticPos_t perturbed_launch = *launch;
    perturbed_launch.alt_m += 1.0;
    Vector3_t perturbed_ecef;
    coord_error = geodetic_to_ecef(&perturbed_ecef, &perturbed_launch);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    Vector3_t z_local_ecef;
    Vector3Error_t vec_error = vector3_subtract(&z_local_ecef, &perturbed_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&z_local_ecef, &z_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // X-axis: Points from launch point to target
    Vector3_t x_local_ecef;
    vec_error = vector3_subtract(&x_local_ecef, &target_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&x_local_ecef, &x_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Y-axis: Completes the right-handed system
    Vector3_t y_local_ecef;
    vec_error = vector3_cross(&y_local_ecef, &z_local_ecef, &x_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&y_local_ecef, &y_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Step 3: Apply rotation matrix transformation (Local → ECEF)
    // This is the inverse of eci_to_local transformation
    // Note: Functions are named "eci" for MATLAB compatibility,
    // but they actually work with ECEF coordinates

    // Matrix multiplication with basis vectors as columns
    Vector3_t x_component, y_component, z_component;
    vec_error = vector3_scale(&x_component, &x_local_ecef, r_local->x);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_scale(&y_component, &y_local_ecef, r_local->y);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_scale(&z_component, &z_local_ecef, r_local->z);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Sum the components
    Vector3_t temp;
    vec_error = vector3_add(&temp, &x_component, &y_component);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    Vector3_t relative_ecef;
    vec_error = vector3_add(&relative_ecef, &temp, &z_component);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Step 4: Add launch position to get absolute ECEF position (LAUNCH-CENTERED)
    vec_error = vector3_add(result, &relative_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    return COORD_TRANSFORM_SUCCESS;
}

//=============================================================================
// ECEF ↔ LOCAL VELOCITY TRANSFORMATIONS
//=============================================================================
CoordTransformError_t ecef_to_local_velocity(Vector3_t* result, const Vector3_t* v_ecef, const GeodeticPos_t* launch, const GeodeticPos_t* target)
{
    // Parameter Validation (MISRA C)
    if ((result == NULL) || (v_ecef == NULL) || (launch == NULL) || (target == NULL)) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // For velocity transformations, only rotation is needed (no translation)
    // Same rotation matrix as position transformation

    // Step 1: Convert geodetic positions to ECEF
    Vector3_t launch_ecef;
    CoordTransformError_t coord_error = geodetic_to_ecef(&launch_ecef, launch);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    Vector3_t target_ecef;
    coord_error = geodetic_to_ecef(&target_ecef, target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    // Step 2: Compute the Local Frame Basis Vectors
    // Z-axis: Points radially outward from Earth's surface
    GeodeticPos_t perturbed_launch = *launch;
    perturbed_launch.alt_m += 1.0;
    Vector3_t perturbed_ecef;
    coord_error = geodetic_to_ecef(&perturbed_ecef, &perturbed_launch);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    Vector3_t z_local_ecef;
    Vector3Error_t vec_error = vector3_subtract(&z_local_ecef, &perturbed_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&z_local_ecef, &z_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // X-axis: Points from launch point to target
    Vector3_t x_local_ecef;
    vec_error = vector3_subtract(&x_local_ecef, &target_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&x_local_ecef, &x_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Y-axis: Completes the right-handed system
    Vector3_t y_local_ecef;
    vec_error = vector3_cross(&y_local_ecef, &z_local_ecef, &x_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&y_local_ecef, &y_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Step 3: Apply rotation to velocity vector (no translation for velocities)
    double dot_x, dot_y, dot_z;
    vec_error = vector3_dot(&dot_x, &x_local_ecef, v_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_dot(&dot_y, &y_local_ecef, v_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_dot(&dot_z, &z_local_ecef, v_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    result->x = dot_x;  // X component in local frame
    result->y = dot_y;  // Y component in local frame
    result->z = dot_z;  // Z component in local frame

    return COORD_TRANSFORM_SUCCESS;
}

CoordTransformError_t local_to_ecef_velocity(Vector3_t* result, const Vector3_t* v_local, const GeodeticPos_t* launch, const GeodeticPos_t* target)
{
    // Parameter validation (MISRA C standard)
    if ((result == NULL) || (v_local == NULL) || (launch == NULL) || (target == NULL)) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // For velocity transformations, we only need rotation (no translation)
    // Use inverse rotation matrix (Local to ECEF)
    // Note: Functions are named "eci" for MATLAB compatibility,
    // but they actually work with ECEF coordinates

    // Step 1: Convert geodetic positions to ECEF
    Vector3_t launch_ecef;
    CoordTransformError_t coord_error = geodetic_to_ecef(&launch_ecef, launch);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    Vector3_t target_ecef;
    coord_error = geodetic_to_ecef(&target_ecef, target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    // Step 2: Compute the Local Frame Basis Vectors
    // Z-axis: Points radially outward from Earth's surface
    GeodeticPos_t perturbed_launch = *launch;
    perturbed_launch.alt_m += 1.0;
    Vector3_t perturbed_ecef;
    coord_error = geodetic_to_ecef(&perturbed_ecef, &perturbed_launch);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return coord_error;
    }

    Vector3_t z_local_ecef;
    Vector3Error_t vec_error = vector3_subtract(&z_local_ecef, &perturbed_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&z_local_ecef, &z_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // X-axis: Points from launch point to target
    Vector3_t x_local_ecef;
    vec_error = vector3_subtract(&x_local_ecef, &target_ecef, &launch_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&x_local_ecef, &x_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Y-axis: Completes the right-handed system
    Vector3_t y_local_ecef;
    vec_error = vector3_cross(&y_local_ecef, &z_local_ecef, &x_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_normalize(&y_local_ecef, &y_local_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Step 3: Apply rotation matrix transformation (Local → ECEF)
    // Matrix multiplication with basis vectors as columns
    Vector3_t x_component, y_component, z_component;
    vec_error = vector3_scale(&x_component, &x_local_ecef, v_local->x);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_scale(&y_component, &y_local_ecef, v_local->y);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_scale(&z_component, &z_local_ecef, v_local->z);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Sum the components
    Vector3_t temp;
    vec_error = vector3_add(&temp, &x_component, &y_component);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    vec_error = vector3_add(result, &temp, &z_component);
    if (vec_error != VECTOR3_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    return COORD_TRANSFORM_SUCCESS;
}

//=============================================================================
// LOCAL ↔ BODY FRAME TRANSFORMATIONS
//=============================================================================

/**
* @brief Convert vector from Local frame to Body frame using 3-2-1 Euler rotation
* @param result Output vector in Body frame (m/s²)
* @param v_local Input vector in Local frame (m/s²)
* @param theta Pitch angle in radians (rotation about y-axis)
* @param psi Yaw angle in radians (rotation about z-axis)
* @param phi Roll angle in radians (rotation about x-axis)
* @return CoordTransformError_t Success or error code
*
* Implementation matches MATLAB l2b.m function
* Rotation sequence: 1) Yaw (psi) about z-axis, 2) Pitch (theta) about y-axis, 3) Roll (phi) about x-axis
* Combined rotation: R_l2b = Rx * Ry * Rz
*/
CoordTransformError_t local_to_body(Vector3_t* result, const Vector3_t* v_local, double theta, double psi, double phi)
{
    // Parameter validation
    if ((result == NULL) || (v_local == NULL)) {
        return COORD_TRANSFORM_ERROR_INVALID_PARAM;
    }

    // Calculate trigonometric functions
    double sin_theta, cos_theta, sin_psi, cos_psi, sin_phi, cos_phi;
    MathUtilsError_t math_error;

    math_error = safe_sin(&sin_theta, theta);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    math_error = safe_cos(&cos_theta, theta);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    math_error = safe_sin(&sin_psi, psi);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    math_error = safe_cos(&cos_psi, psi);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    math_error = safe_sin(&sin_phi, phi);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    math_error = safe_cos(&cos_phi, phi);
    if (math_error != MATH_UTILS_SUCCESS) {
        return COORD_TRANSFORM_ERROR_MATH_ERROR;
    }

    // Build rotation matrices (3-2-1 Euler sequence)
    // First: Yaw rotation about z-axis
    // Rz = [cos(psi)  sin(psi)  0;
    //      -sin(psi)  cos(psi)  0;
    //       0         0         1]

    // Second: Pitch rotation about y-axis
    // Ry = [cos(theta)  0  -sin(theta);
    //       0           1   0;
    //       sin(theta)  0   cos(theta)]

    // Third: Roll rotation about x-axis
    // Rx = [1  0         0;
    //       0  cos(phi)  sin(phi);
    //       0 -sin(phi)  cos(phi)]

    // Combined rotation matrix: R_l2b = Rx * Ry * Rz
    // Compute matrix elements directly for efficiency
    double R00 = cos_theta * cos_psi;
    double R01 = cos_theta * sin_psi;
    double R02 = -sin_theta;

    double R10 = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi;
    double R11 = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi;
    double R12 = sin_phi * cos_theta;

    double R20 = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
    double R21 = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
    double R22 = cos_phi * cos_theta;

    // Apply rotation matrix transformation
    // vec_body = R_l2b * vec_local
    result->x = R00 * v_local->x + R01 * v_local->y + R02 * v_local->z;
    result->y = R10 * v_local->x + R11 * v_local->y + R12 * v_local->z;
    result->z = R20 * v_local->x + R21 * v_local->y + R22 * v_local->z;

    return COORD_TRANSFORM_SUCCESS;
}