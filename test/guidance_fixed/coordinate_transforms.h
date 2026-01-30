/******************************************************************************
* ISA Flight Software
 * @file coordinate_transforms.h
 * @brief Coordinate Transformation Functions Header
* @details Defines coordinate transformation function prototypes and WGS-84 constants
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 4.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef COORD_TRANS_H
#define COORD_TRANS_H

#include "vector3.h"

/* WGS-84 constants (same as MATLAB helper) */
#define WGS84_A 6378137.0           /* semi-major axis (m) */
#define WGS84_F (1.0/298.257223563) /* flattening */
#define WGS84_E2 (2*WGS84_F - WGS84_F*WGS84_F)

void ecef_to_local(Vector3* r_local,const Vector3* r_ecef,
                    double lat_o,double lon_o,double alt_o,
                    double lat_t,double lon_t,double alt_t);

void local_to_ecef_vel(Vector3* v_ecef,const Vector3* v_local,
                        double lat_o,double lon_o,double alt_o,
                        double lat_t,double lon_t,double alt_t);

void ecef_to_local_vel(Vector3* v_local,const Vector3* v_ecef,
                        double lat_o,double lon_o,double alt_o,
                        double lat_t,double lon_t,double alt_t);

#endif

