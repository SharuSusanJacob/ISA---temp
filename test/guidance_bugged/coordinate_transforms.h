/******************************************************************************
* ISA Flight Software
* @file coordinate_transforms.h
* @brief Coordinate Frame Transformations for Guidance System
* @details Transforms between Geodetic, ECEF, and Local reference frames
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 3.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef COORDINATE_TRANSFORMS_H
#define COORDINATE_TRANSFORMS_H

#include <stdint.h>
#include <stdbool.h>
#include "vector3.h"

// Error codes (following MISRA C pattern)
typedef enum {
    COORD_TRANSFORM_SUCCESS = 0U,
    COORD_TRANSFORM_ERROR_INVALID_PARAM = 1U,
    COORD_TRANSFORM_ERROR_MATH_ERROR = 2U
} CoordTransformError_t;

// define WGS84 Earth Model constants (aerospace standard)
#define WGS84_SEMI_MAJOR_AXIS      6378137.0      // Earth's Semi-major axis in meters
#define WGS84_FLATTENING           (1.0/298.257223563)  // Earth's flattening
#define WGS84_ECCENTRICITY_SQ     (2.0 * WGS84_FLATTENING - WGS84_FLATTENING * WGS84_FLATTENING)  // Square of eccentricity

//=============================================================================
// GEODETIC POSITION STRUCTURE
//=============================================================================

/**
* @brief Geodetic Position (latitude, longitude, altitude)
* This is what gps gives - where you are on earths serface.
*/

typedef struct {
    double lat_deg;       // Latitude in degrees
    double lon_deg;      // Longitude in degrees
    double alt_m;        // Altitude above WGS84 ellipsoid in meters
} GeodeticPos_t;

//=============================================================================
// GEODETIC ↔ ECEF TRANSFORMATIONS 
//=============================================================================

/**
* @brief Convert geodetic position to ECEF vector
* @param result Output ECEF position vector (meters) (must not be NULL)
* @param pos Input geodetic position (must not be NULL)
* @return CoordTransformError_t Success or error code
*
* USAGE: Convert GPS position to physics calculations frame
* Used When:
* - Converting target position from lat/lon to guidance calculations
* - Converting projectile position
*/
CoordTransformError_t geodetic_to_ecef(Vector3_t* result, const GeodeticPos_t* pos);

/**
* @brief Convert ECEF coordinates to geodetic position
* @param result Output geodetic position (must not be NULL)
* @param r_ecef Input ECEF position vector (meters) (must not be NULL)
* @return CoordTransformError_t Success or error code
*
* USAGE: Convert physics results back to lat,long,alt
* Used for displaying results, logging trajectory
*/
CoordTransformError_t ecef_to_geodetic(GeodeticPos_t* result, const Vector3_t* r_ecef);

//=============================================================================
// ECEF ↔ LOCAL FRAME TRANSFORMATIONS
//=============================================================================

/**
* @brief Convert ECEF coordinates to Local Frame
* @param result Output Position in Local Frame (meters) (must not be NULL)
* @param r_ecef Input ECEF position vector (must not be NULL)
* @param launch Launch point geodetic position (must not be NULL)
* @param target Target Geodetic position (must not be NULL)
* @return CoordTransformError_t Success or error code
*
* GUIDANCE: Creates the Local frame used in guidance algorithm
* Local Frame Definition:
* - Origin: Launch point
* - X-axis: Points from launch point toward target
* - Z-axis: Points radially outward from Earth's surface (normal to ellipsoid)
* - Y-axis: Completes right handed system (cross(Z, X))
*/
CoordTransformError_t ecef_to_local(Vector3_t* result, const Vector3_t* r_ecef, const GeodeticPos_t* launch, const GeodeticPos_t* target);

/**
* @brief Convert Local frame position to ECEF
* @param result Output ECEF position vector (must not be NULL)
* @param r_local Input Local frame position (must not be NULL)
* @param launch Launch point geodetic position (must not be NULL)
* @param target Target geodetic position (must not be NULL)
* @return CoordTransformError_t Success or error code
*/
CoordTransformError_t local_to_ecef(Vector3_t* result, const Vector3_t* r_local, const GeodeticPos_t* launch, const GeodeticPos_t* target);


/**
* @brief Convert ECEF velocity to Local Frame velocity
* @param result Output velocity in Local Frame (m/s) (must not be NULL)
* @param v_ecef Input ECEF velocity vector (must not be NULL)
* @param launch Launch point geodetic position (must not be NULL)
* @param target Target geodetic position (must not be NULL)
* @return CoordTransformError_t Success or error code
*
* Guidance USAGE: Transform projectile velocity for guidance calculations
* Velocity transforms differently than position (no translation offset)
*/
CoordTransformError_t ecef_to_local_velocity(Vector3_t* result, const Vector3_t* v_ecef, const GeodeticPos_t* launch, const GeodeticPos_t* target);

/**
* @brief Convert Local frame velocity to ECEF velocity
* @param result Output ECEF velocity vector (must not be NULL)
* @param v_local Input Local frame velocity (must not be NULL)
* @param launch Launch point geodetic position (must not be NULL)
* @param target Target geodetic position (must not be NULL)
* @return CoordTransformError_t Success or error code
*
* Guidance USAGE: Transform guidance commands back to ECEF
*/
CoordTransformError_t local_to_ecef_velocity(Vector3_t* result, const Vector3_t* v_local, const GeodeticPos_t* launch, const GeodeticPos_t* target);

//=============================================================================
// LOCAL ↔ BODY FRAME TRANSFORMATIONS
//=============================================================================

/**
* @brief Convert vector from Local frame to Body frame
* @param result Output vector in Body frame (must not be NULL)
* @param v_local Input vector in Local frame (must not be NULL)
* @param theta Pitch angle in radians (rotation about y-axis)
* @param psi Yaw angle in radians (rotation about z-axis)
* @param phi Roll angle in radians (rotation about x-axis)
* @return CoordTransformError_t Success or error code
*
* Guidance USAGE: Transform guidance acceleration commands from Local to Body frame
* Uses 3-2-1 Euler rotation sequence (Roll-Pitch-Yaw)
* This is the input to DAP (Digital Autopilot)
*/
CoordTransformError_t local_to_body(Vector3_t* result, const Vector3_t* v_local, double theta, double psi, double phi);

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

/**
* @brief Create geodetic position structure
* @param result Output Geodetic position (must not be NULL)
* @param lat_deg Latitude in degrees
* @param lon_deg Longitude in degrees
* @param alt_m Altitude in meters
* @return CoordTransformError_t Success or error code
*/
CoordTransformError_t geodetic_create(GeodeticPos_t* result, double lat_deg, double lon_deg, double alt_m);

#endif /* COORDINATE_TRANSFORMS_H */
