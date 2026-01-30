/******************************************************************************
* ISA Flight Software
 * @file coordinate_transforms.c
 * @brief Coordinate Transformation Functions Implementation
* @details Implements ECEF to Local and Local to ECEF coordinate transformations
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 4.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#include "coordinate_transforms.h"
#include "math_utils.h"
#include <math.h>

/* simple geodetic->ecef helper (WGS-84) */
static void geodetic_to_ecef(Vector3* r_ecef, double lat_deg,double lon_deg,double alt_m)
{
    double lat_rad = DEG2RAD(lat_deg);
    double lon_rad = DEG2RAD(lon_deg);
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);
    double n = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    r_ecef->x = (n + alt_m) * cos_lat * cos_lon;
    r_ecef->y = (n + alt_m) * cos_lat * sin_lon;
    r_ecef->z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat;
}

static void build_local_axes(Vector3* x_axis,Vector3* y_axis,Vector3* z_axis,
                             double lat_o,double lon_o,double alt_o,
                             double lat_t,double lon_t,double alt_t)
{
    Vector3 launch_ecef,target_ecef,perturb_ecef;
    geodetic_to_ecef(&launch_ecef,lat_o,lon_o,alt_o);
    geodetic_to_ecef(&target_ecef,lat_t,lon_t,alt_t);
    geodetic_to_ecef(&perturb_ecef,lat_o,lon_o,alt_o+1.0); /* +1 m for surface normal */

    /* z axis */
    v3_sub(z_axis,&perturb_ecef,&launch_ecef);
    v3_normed(z_axis,z_axis);
    /* x axis */
    v3_sub(x_axis,&target_ecef,&launch_ecef);
    v3_normed(x_axis,x_axis);
    /* y axis */
    v3_cross(y_axis,z_axis,x_axis);
    v3_normed(y_axis,y_axis);
}

void ecef_to_local(Vector3* r_local,const Vector3* r_ecef,
                    double lat_o,double lon_o,double alt_o,
                    double lat_t,double lon_t,double alt_t)
{
    Vector3 x_axis,y_axis,z_axis;
    build_local_axes(&x_axis,&y_axis,&z_axis,
                     lat_o,lon_o,alt_o,lat_t,lon_t,alt_t);

    Vector3 launch_ecef;
    geodetic_to_ecef(&launch_ecef,lat_o,lon_o,alt_o);

    Vector3 rel;
    v3_sub(&rel,r_ecef,&launch_ecef);

    r_local->x = v3_dot(&x_axis,&rel);
    r_local->y = v3_dot(&y_axis,&rel);
    r_local->z = v3_dot(&z_axis,&rel);
}

void local_to_ecef_vel(Vector3* v_ecef,const Vector3* v_local,
                        double lat_o,double lon_o,double alt_o,
                        double lat_t,double lon_t,double alt_t)
{
    Vector3 x_axis,y_axis,z_axis;
    build_local_axes(&x_axis,&y_axis,&z_axis,
                     lat_o,lon_o,alt_o,lat_t,lon_t,alt_t);
    /* matrix multiplication R_local->ecef (basis vectors as columns) */
    v_ecef->x = x_axis.x*v_local->x + y_axis.x*v_local->y + z_axis.x*v_local->z;
    v_ecef->y = x_axis.y*v_local->x + y_axis.y*v_local->y + z_axis.y*v_local->z;
    v_ecef->z = x_axis.z*v_local->x + y_axis.z*v_local->y + z_axis.z*v_local->z;
}

void ecef_to_local_vel(Vector3* v_local,const Vector3* v_ecef,
                        double lat_o,double lon_o,double alt_o,
                        double lat_t,double lon_t,double alt_t)
{
    Vector3 x_axis,y_axis,z_axis;
    build_local_axes(&x_axis,&y_axis,&z_axis,
                     lat_o,lon_o,alt_o,lat_t,lon_t,alt_t);
    /* matrix multiplication R_ECEF->Local (transpose of basis vectors) */
    v_local->x = v3_dot(&x_axis,v_ecef);
    v_local->y = v3_dot(&y_axis,v_ecef);
    v_local->z = v3_dot(&z_axis,v_ecef);
}

