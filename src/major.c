/******************************************************************************
* ISA Flight Software
* @file major.c
* @brief Major Cycle (10Hz) Implementation with Navigation,Guidance Module
* @details Implements 100ms cycle with direct guidance algorithm integration
* @author Integration Team, Ananhu Dev - Spacelabs
* @date 2025
* @version 1.0.2
*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "major.h" /* Include before system_state.h so GuidanceState_t is defined */
#include "math_utils.h"
#include "type_convert.h" /* Include for LSB to double conversion */
#include "system_state.h"
#include "pefcs_init.h" /* Include for PEFCS initialization constants */

// Global system state
SystemState_t systemState;

/* GNSS Fail-Safe Variables */
static bool gnss_data_valid = false;
static GnssEcefPosition_t previous_gnss_position = {0.0, 0.0, 0.0, false, 0U};
static GnssEcefVelocity_t previous_gnss_velocity = {0.0, 0.0, 0.0, false, 0U};

/* Global hardware input variables */
static GnssEcefPosition_t gnss_position_ecef = {0.0, 0.0, 0.0, false, 0U};
static GnssEcefVelocity_t gnss_velocity_ecef = {0.0, 0.0, 0.0, false, 0U};

//mijo cheta checkoutine ivide kettanam , launch point and target in ECEF frame ivide kettanam
static Vector3_t mission_target_ecef = {0.0, 0.0, 0.0};       /* Target in ECEF frame */
static Vector3_t mission_launch_point_ecef = {0.0, 0.0, 0.0}; /* Launch point in ECEF frame */

// Mijo cheta , raheese - Checkoutine Elevetion ivide theta_f il
static double guidance_theta_f = 0.0; /* Desired impact elevation angle (rad) */
// Mijo cheta , raheese - Checkoutine Azimuth ivide psi_f il
static double guidance_psi_f = 0.0; /* Desired impact azimuth angle (rad) */

/* External GNSS lock flag from hardware team */
volatile bool gnss_lock;

/* Forward declarations for static helper functions */
static void build_local_axes_from_ecef(Vector3_t *x_axis, Vector3_t *y_axis, Vector3_t *z_axis,
                                       const Vector3_t *origin_ecef, const Vector3_t *target_ecef);
static void ecef_to_local_using_axes(Vector3_t *result, const Vector3_t *r_ecef,
                                     const Vector3_t *origin_ecef, const Vector3_t *x_axis, const Vector3_t *y_axis, const Vector3_t *z_axis);
static void local_to_ecef_using_axes(Vector3_t *result, const Vector3_t *v_local,
                                     const Vector3_t *x_axis, const Vector3_t *y_axis, const Vector3_t *z_axis);

/**
 * @brief Process GNSS data with fail-safe handling
 *
 * This function processes GNSS data from hardware buffer with fail-safe
 * mechanism. If GNSS lock is lost, previous valid data is used.
 */
void process_gnss_data(void)
{
    /* Check GNSS lock status from hardware */
    if (gnss_lock)
    {
        /* GNSS data is valid - hardware team will parse and extract ECEF data */
        /* TODO: Hardware team has to  replace this with actual parsing */
        /* Example: parse_gnss_buffer_to_ecef(&gnss_position_ecef, &gnss_velocity_ecef); */

        /* Placeholder: Set some test values (hardware team will replace) */
        // raheese gnss in ecef frameil x,y,z ivide kettanam...ithe position
        /* COMMENTED OUT - Hardware team to implement
        convert_ecef_position_raw_to_double(&gnss_position_ecef.x_m,
                                            &gnss_position_ecef.y_m,
                                            &gnss_position_ecef.z_m,
                                            gnssDataBuffer, 39, 43, 47);

        convert_ecef_velocity_raw_to_double(&gnss_velocity_ecef.vx_ms,
                                            &gnss_velocity_ecef.vy_ms,
                                            &gnss_velocity_ecef.vz_ms,
                                            gnssDataBuffer, 51, 55, 59);
        */

        gnss_velocity_ecef.valid = true;

        /* Update fail-safe variables with current valid data */
        previous_gnss_position = gnss_position_ecef;
        previous_gnss_velocity = gnss_velocity_ecef;

        systemState.position_ecef.x = gnss_position_ecef.x_m;
        systemState.position_ecef.y = gnss_position_ecef.y_m;
        systemState.position_ecef.z = gnss_position_ecef.z_m;

        systemState.velocity_ecef.x = gnss_velocity_ecef.vx_ms;
        systemState.velocity_ecef.y = gnss_velocity_ecef.vy_ms;
        systemState.velocity_ecef.z = gnss_velocity_ecef.vz_ms;

        gnss_data_valid = true;
    }
    else
    {
        /* GNSS lock lost - use previous valid data (fail-safe) */
        if (gnss_data_valid)
        {
            /* Use previous valid data */
            gnss_position_ecef = previous_gnss_position;
            gnss_velocity_ecef = previous_gnss_velocity;
            /* Keep valid flags as true since we're using valid previous data */
        }
        else
        {
            /* No previous valid data available - set invalid */
            gnss_position_ecef.valid = false;
            gnss_velocity_ecef.valid = false;
        }
    }
}

/**
 * @brief Process GNSS ECEF data and convert to local frame
 *
 * This function processes the GNSS ECEF position and velocity data,
 * converts it to the local frame, and stores it in system state.
 * Only performs coordinate transformation - no guidance execution.
 */
void process_gnss_ecef_data(void)
{
    /* Check if GNSS data is valid */
    if (!gnss_position_ecef.valid || !gnss_velocity_ecef.valid)
    {
        /* Invalid GNSS data - cannot process */
        return;
    }

    /* Note: Mission data validity check removed - will be handled by guidance function */

    /* Convert GNSS ECEF position to Vector3_t */
    Vector3_t ecef_position;
    ecef_position.x = gnss_position_ecef.x_m;
    ecef_position.y = gnss_position_ecef.y_m;
    ecef_position.z = gnss_position_ecef.z_m;

    /* Convert GNSS ECEF velocity to Vector3_t */
    Vector3_t ecef_velocity;
    ecef_velocity.x = gnss_velocity_ecef.vx_ms;
    ecef_velocity.y = gnss_velocity_ecef.vy_ms;
    ecef_velocity.z = gnss_velocity_ecef.vz_ms;

    /* Build local frame axes from ECEF vectors (launch point and target already in ECEF) */
    Vector3_t x_axis, y_axis, z_axis;
    build_local_axes_from_ecef(&x_axis, &y_axis, &z_axis, &mission_launch_point_ecef, &mission_target_ecef);

    /* Convert ECEF position to Local frame */
    ecef_to_local_using_axes(&systemState.positionLocal, &ecef_position, &mission_launch_point_ecef, &x_axis, &y_axis, &z_axis);

    /* Convert ECEF velocity to Local frame */
    double dot_x, dot_y, dot_z;
    vector3_dot(&dot_x, &x_axis, &ecef_velocity);
    vector3_dot(&dot_y, &y_axis, &ecef_velocity);
    vector3_dot(&dot_z, &z_axis, &ecef_velocity);
    systemState.velocityLocal.x = dot_x;
    systemState.velocityLocal.y = dot_y;
    systemState.velocityLocal.z = dot_z;

    /* Store ECEF data for other modules */
    systemState.position_ecef = ecef_position;
    systemState.velocity_ecef = ecef_velocity;

    /* Note: Coordinate transformation success/failure is stored in systemState
     * Other modules can check the validity of positionLocal and velocityLocal
     */
}

/* ===== HARDWARE INTERFACE FUNCTIONS (For Hardware Team) ===== */

/**
 * @brief Set GNSS ECEF position data from hardware
 *
 * This function is called by the hardware team to provide GNSS position data.
 *
 * @param x_m ECEF X position in meters
 * @param y_m ECEF Y position in meters
 * @param z_m ECEF Z position in meters
 * @param timestamp GNSS timestamp
 */
void set_gnss_position_ecef(const uint8_t *buffer, uint16_t offset_x, uint16_t offset_y, uint16_t offset_z, bool valid, uint32_t timestamp)
{
    if (buffer == NULL)
    {
        gnss_position_ecef.valid = false;
        return;
    }

    /* Convert LSB values (signed int32, big-endian) to double (meters) */
    /* COMMENTED OUT - Hardware team to implement double version
    convert_ecef_position_lsb_to_double(&gnss_position_ecef.x_m, &gnss_position_ecef.y_m, &gnss_position_ecef.z_m,
                                        buffer, offset_x, offset_y, offset_z);
    */
    gnss_position_ecef.valid = valid;
    gnss_position_ecef.timestamp = timestamp;
}

/**
 * @brief Set GNSS ECEF velocity data from hardware
 *
 * This function is called by the hardware team to provide GNSS velocity data.
 *
 * @param vx_ms ECEF X velocity in m/s
 * @param vy_ms ECEF Y velocity in m/s
 * @param vz_ms ECEF Z velocity in m/s
 * @param valid Data validity flag
 * @param timestamp GNSS timestamp
 */
void set_gnss_velocity_ecef(const uint8_t *buffer, uint16_t offset_vx, uint16_t offset_vy, uint16_t offset_vz, bool valid, uint32_t timestamp)
{
    if (buffer == NULL)
    {
        gnss_velocity_ecef.valid = false;
        return;
    }

    /* Convert LSB values (signed int32, big-endian) to double (m/s) */
    /* COMMENTED OUT - Hardware team to implement double version
    convert_ecef_velocity_lsb_to_double(&gnss_velocity_ecef.vx_ms, &gnss_velocity_ecef.vy_ms, &gnss_velocity_ecef.vz_ms,
                                        buffer, offset_vx, offset_vy, offset_vz);
    */
    gnss_velocity_ecef.valid = valid;
    gnss_velocity_ecef.timestamp = timestamp;
}

/**
 * @brief Set mission target coordinates
 *
 * This function sets the target coordinates for the mission in ECEF frame.
 *
 * @param x_m Target X coordinate in ECEF frame (meters)
 * @param y_m Target Y coordinate in ECEF frame (meters)
 * @param z_m Target Z coordinate in ECEF frame (meters)
 */
void set_mission_target(double x_m, double y_m, double z_m)
{
    mission_target_ecef.x = (double)x_m;
    mission_target_ecef.y = (double)y_m;
    mission_target_ecef.z = (double)z_m;
}

/**
 * @brief Set mission launch point coordinates
 *
 * This function sets the launch point coordinates for the mission in ECEF frame.
 *
 * @param x_m Launch point X coordinate in ECEF frame (meters)
 * @param y_m Launch point Y coordinate in ECEF frame (meters)
 * @param z_m Launch point Z coordinate in ECEF frame (meters)
 */
void set_mission_launch_point(double x_m, double y_m, double z_m)
{
    mission_launch_point_ecef.x = (double)x_m;
    mission_launch_point_ecef.y = (double)y_m;
    mission_launch_point_ecef.z = (double)z_m;
}

/**
 * @brief Set guidance impact angles
 */
void set_guidance_impact_angles(double theta_f_rad, double psi_f_rad)
{
    guidance_theta_f = theta_f_rad;
    guidance_psi_f = psi_f_rad;
}

/**
 * @brief Initialize guidance module with PEFCS parameters
 *
 * Converts geodetic coordinates from PEFCS to ECEF and initializes
 * mission target, launch point, and impact angles.
 */
void guidance_init(void)
{
    /* Create geodetic structures for target and origin */
    GeodeticPos_t target_geodetic;
    target_geodetic.lat_deg = PEFCS_TARGET_LAT_DEG;
    target_geodetic.lon_deg = PEFCS_TARGET_LON_DEG;
    target_geodetic.alt_m = PEFCS_TARGET_ALT_M;

    GeodeticPos_t origin_geodetic;
    origin_geodetic.lat_deg = PEFCS_ORIGIN_LAT_DEG;
    origin_geodetic.lon_deg = PEFCS_ORIGIN_LON_DEG;
    origin_geodetic.alt_m = PEFCS_ORIGIN_ALT_M;

    /* Convert to ECEF */
    Vector3_t target_ecef, origin_ecef;
    geodetic_to_ecef(&target_ecef, &target_geodetic);
    geodetic_to_ecef(&origin_ecef, &origin_geodetic);

    /* Set mission parameters */
    set_mission_target(target_ecef.x, target_ecef.y, target_ecef.z);
    set_mission_launch_point(origin_ecef.x, origin_ecef.y, origin_ecef.z);

    /* Convert impact angles from degrees to radians and set */
    double theta_f_rad, psi_f_rad;
    deg_to_rad(&theta_f_rad, PEFCS_IMPACT_THETA_F_DEG);
    deg_to_rad(&psi_f_rad, PEFCS_IMPACT_PSI_F_DEG);
    set_guidance_impact_angles(theta_f_rad, psi_f_rad);
}

/* ===== GUIDANCE ALGORITHM IMPLEMENTATION ===== */

/**
 * @brief Calculate guidance acceleration command
 *
 * Implements Proportional Navigation (PN) + Impact Angle Control (IAC)
 * Based on test module guidance_fixed/guidance.c
 */
/**
  * @brief Build local frame axes from ECEF vectors
  * 
  * Uses geodetic normal perturbation method for Z-axis calculation,
  * matching test/guidance_updated_clamp/coordinate_transforms.c
  * This provides the true surface normal perpendicular to the WGS84 ellipsoid.
  */
static void build_local_axes_from_ecef(Vector3_t *x_axis, Vector3_t *y_axis, Vector3_t *z_axis,
                                       const Vector3_t *origin_ecef, const Vector3_t *target_ecef)
{
    /* Calculate perturbed origin position (+1m altitude) for geodetic normal */
    /* This approximates the local vertical direction perpendicular to ellipsoid */
    double origin_mag;
    vector3_magnitude(&origin_mag, origin_ecef);

    /* Create a perturbed point 1m higher along the geocentric radius */
    /* (Note: This is a simplified approximation. For exact geodetic normal,
     * we would need to convert ECEF->Geodetic, perturb alt, then convert back) */
    double scale_factor = (origin_mag + 1.0) / origin_mag;
    Vector3_t perturb_ecef;
    vector3_scale(&perturb_ecef, origin_ecef, scale_factor);

    /* z axis - surface normal (from origin to perturbed point) */
    vector3_subtract(z_axis, &perturb_ecef, origin_ecef);
    vector3_normalize(z_axis, z_axis);

    /* x axis - from origin to target */
    vector3_subtract(x_axis, target_ecef, origin_ecef);
    vector3_normalize(x_axis, x_axis);

    /* y axis - completes right-handed system */
    vector3_cross(y_axis, z_axis, x_axis);
    vector3_normalize(y_axis, y_axis);
}

/**
 * @brief Convert ECEF to local frame using pre-built axes
 */
static void ecef_to_local_using_axes(Vector3_t *result, const Vector3_t *r_ecef,
                                     const Vector3_t *origin_ecef, const Vector3_t *x_axis, const Vector3_t *y_axis, const Vector3_t *z_axis)
{
    Vector3_t rel;
    vector3_subtract(&rel, r_ecef, origin_ecef);

    double dot_x, dot_y, dot_z;
    vector3_dot(&dot_x, x_axis, &rel);
    vector3_dot(&dot_y, y_axis, &rel);
    vector3_dot(&dot_z, z_axis, &rel);

    result->x = dot_x;
    result->y = dot_y;
    result->z = dot_z;
}

/**
 * @brief Convert local frame to ECEF using pre-built axes
 */
static void local_to_ecef_using_axes(Vector3_t *result, const Vector3_t *v_local,
                                     const Vector3_t *x_axis, const Vector3_t *y_axis, const Vector3_t *z_axis)
{
    /* Matrix multiplication R_local->ecef (basis vectors as columns) */
    result->x = x_axis->x * v_local->x + y_axis->x * v_local->y + z_axis->x * v_local->z;
    result->y = x_axis->y * v_local->x + y_axis->y * v_local->y + z_axis->y * v_local->z;
    result->z = x_axis->z * v_local->x + y_axis->z * v_local->y + z_axis->z * v_local->z;
}

void calculate_guidance_acceleration(Vector3_t *accel_body,
                                     const Vector3_t *position_ecef, const Vector3_t *velocity_ecef,
                                     const Vector3_t *origin_ecef, const Vector3_t *target_ecef,
                                     double theta_f, double psi_f, double theta, double psi, double phi)
{
    /* Build local frame axes from ECEF vectors (matching test guidance.c) */
    Vector3_t x_axis, y_axis, z_axis;
    build_local_axes_from_ecef(&x_axis, &y_axis, &z_axis, origin_ecef, target_ecef);

    /* Convert projectile ECEF to local frame */
    Vector3_t r_m_local, v_m_local;
    ecef_to_local_using_axes(&r_m_local, position_ecef, origin_ecef, &x_axis, &y_axis, &z_axis);

    /* Convert velocity to local frame */
    double dot_x, dot_y, dot_z;
    vector3_dot(&dot_x, &x_axis, velocity_ecef);
    vector3_dot(&dot_y, &y_axis, velocity_ecef);
    vector3_dot(&dot_z, &z_axis, velocity_ecef);
    v_m_local.x = dot_x;
    v_m_local.y = dot_y;
    v_m_local.z = dot_z;

    /* Convert target to local frame */
    Vector3_t r_t_local;
    ecef_to_local_using_axes(&r_t_local, target_ecef, origin_ecef, &x_axis, &y_axis, &z_axis);

    /* Vector to target FROM projectile */
    Vector3_t r_vec;
    vector3_subtract(&r_vec, &r_t_local, &r_m_local);
    /* Add terminal phase offset to target z */
    r_vec.z += GUID_TERMINAL_DISTANCE_M;

    /* Calculate range */
    double r;
    vector3_magnitude(&r, &r_vec);

    /* Check terminal phase */
    if (r <= GUID_TERMINAL_DISTANCE_M)
    {
        /* Terminal phase - zero acceleration */
        accel_body->x = 0.0;
        accel_body->y = 0.0;
        accel_body->z = 0.0;

        /* Update System State */
        systemState.guidanceState.timeToGo = 0.0;
        systemState.guidanceState.terminalPhase = true;
        return;
    }

    /* Unit vector of line-of-sight */
    Vector3_t e_r;
    vector3_normalize(&e_r, &r_vec);

    /* Velocity magnitude and unit vector */
    double v;
    vector3_magnitude(&v, &v_m_local);
    if (v < GUID_MIN_VELOCITY_M_S)
    {
        v = GUID_MIN_VELOCITY_M_S;
    }
    Vector3_t e_m;
    vector3_normalize(&e_m, &v_m_local);

    /* Lead angle (sigma) */
    double cos_sigma;
    vector3_dot(&cos_sigma, &e_m, &e_r);
    if (cos_sigma > 1.0)
        cos_sigma = 1.0;
    if (cos_sigma < -1.0)
        cos_sigma = -1.0;
    double sigma = acos(cos_sigma);

    /* Time to go */
    double cos_sigma_safe = (cos_sigma > GUID_EPSILON) ? cos_sigma : GUID_EPSILON;
    double t_go = r / (v * cos_sigma_safe);

    /* Update System State */
    systemState.guidanceState.timeToGo = t_go;
    systemState.guidanceState.terminalPhase = false;

    /* Calculate line of sight rate vector */
    Vector3_t cross_v_r;
    vector3_cross(&cross_v_r, &v_m_local, &r_vec);
    double r_squared = r * r;
    if (r_squared < GUID_EPSILON)
        r_squared = GUID_EPSILON;

    Vector3_t omega_l;
    vector3_scale(&omega_l, &cross_v_r, 1.0 / r_squared);

    /* 3D Proportional Navigation (PN) */
    Vector3_t n_omega_l;
    vector3_scale(&n_omega_l, &omega_l, GUID_NAVIGATION_GAIN_N);
    Vector3_t a_p;
    vector3_cross(&a_p, &n_omega_l, &v_m_local);

    /* Impact Angle Control (IAC) calculations */
    Vector3_t cross_e_r_e_m;
    vector3_cross(&cross_e_r_e_m, &e_r, &e_m);
    double cross_norm;
    vector3_magnitude(&cross_norm, &cross_e_r_e_m);
    Vector3_t k_l;
    vector3_scale(&k_l, &cross_e_r_e_m, 1.0 / (cross_norm + GUID_EPSILON));

    double eta = sigma / (GUID_NAVIGATION_GAIN_N - 1.0);
    double mu = sigma + eta;

    /* Quaternion rotation */
    double half_mu = (-mu) / 2.0;
    double w = cos(half_mu);
    double x_q = sin(half_mu) * k_l.x;
    double y_q = sin(half_mu) * k_l.y;
    double z_q = sin(half_mu) * k_l.z;

    /* Rotation matrix from quaternion */
    double l_q[9];
    l_q[0] = w * w + x_q * x_q - y_q * y_q - z_q * z_q;
    l_q[1] = 2.0 * (x_q * y_q - w * z_q);
    l_q[2] = 2.0 * (x_q * z_q + w * y_q);
    l_q[3] = 2.0 * (x_q * y_q + w * z_q);
    l_q[4] = w * w - x_q * x_q + y_q * y_q - z_q * z_q;
    l_q[5] = 2.0 * (y_q * z_q - w * x_q);
    l_q[6] = 2.0 * (x_q * z_q - w * y_q);
    l_q[7] = 2.0 * (y_q * z_q + w * x_q);
    l_q[8] = w * w - x_q * x_q - y_q * y_q + z_q * z_q;

    /* Rotate velocity unit vector */
    Vector3_t e_p;
    e_p.x = l_q[0] * e_m.x + l_q[1] * e_m.y + l_q[2] * e_m.z;
    e_p.y = l_q[3] * e_m.x + l_q[4] * e_m.y + l_q[5] * e_m.z;
    e_p.z = l_q[6] * e_m.x + l_q[7] * e_m.y + l_q[8] * e_m.z;

    /* Desired impact direction */
    Vector3_t e_f;
    double sin_theta_f = sin(theta_f);
    double cos_theta_f = cos(theta_f);
    double sin_psi_f = sin(psi_f);
    double cos_psi_f = cos(psi_f);
    e_f.x = cos_theta_f * sin_psi_f; /* East */
    e_f.y = cos_theta_f * cos_psi_f; /* North */
    e_f.z = sin_theta_f;             /* Up */

    /* Impact angle error */
    double cos_delta;
    vector3_dot(&cos_delta, &e_f, &e_p);
    if (cos_delta > 1.0)
        cos_delta = 1.0;
    if (cos_delta < -1.0)
        cos_delta = -1.0;
    double delta = acos(cos_delta);

    Vector3_t cross_e_f_e_p;
    vector3_cross(&cross_e_f_e_p, &e_f, &e_p);
    double cross_norm_lf;
    vector3_magnitude(&cross_norm_lf, &cross_e_f_e_p);
    Vector3_t l_f;
    vector3_scale(&l_f, &cross_e_f_e_p, 1.0 / (cross_norm_lf + GUID_EPSILON));

    /* Sigma normalization */
    double sigma_not = sigma / GUID_MAX_LOOK_ANGLE_RAD;
    double abs_sigma_not = fabs(sigma_not);
    double abs_sigma_not_pow_d = pow(abs_sigma_not, GUID_PARAM_D);
    double f_sigma_not = 1.0 - abs_sigma_not_pow_d;

    /* Sig function for delta */
    double sig_m_delta, sig_n_delta;
    double sign_delta = (delta >= 0.0) ? 1.0 : -1.0;
    double abs_delta = fabs(delta);
    sig_m_delta = sign_delta * pow(abs_delta, GUID_PARAM_M);
    sig_n_delta = sign_delta * pow(abs_delta, GUID_PARAM_N);

    double h_delta = (GUID_PARAM_A * sig_m_delta) + (GUID_PARAM_B * sig_n_delta);
    double delta_dot = (-GUID_PARAM_K * f_sigma_not / t_go) * h_delta;

    /* Calculate l_m and l_p */
    Vector3_t l_m, l_p;
    vector3_cross(&l_m, &k_l, &e_m);
    vector3_cross(&l_p, &k_l, &e_p);

    /* IAC acceleration components */
    double a_p_scalar = ((-GUID_NAVIGATION_GAIN_N * (v * v) * sinf(sigma)) / r);
    Vector3_t cross_l_f_e_p;
    vector3_cross(&cross_l_f_e_p, &l_f, &e_p);
    double dot_cross_l_p;
    vector3_dot(&dot_cross_l_p, &cross_l_f_e_p, &l_p);
    double a_i_lm = (((GUID_NAVIGATION_GAIN_N - 1.0f) * GUID_PARAM_K * f_sigma_not * v * h_delta) / t_go) * dot_cross_l_p;

    double sin_eta = sin(eta);
    if (fabs(sin_eta) < GUID_EPSILON)
        sin_eta = GUID_EPSILON;
    double dot_cross_k_l;
    vector3_dot(&dot_cross_k_l, &cross_l_f_e_p, &k_l);
    double a_i_kl = ((GUID_PARAM_K * f_sigma_not * v * sinf(sigma) * h_delta) / (t_go * sin_eta)) * dot_cross_k_l;

    /* Final IAC acceleration command */
    Vector3_t a_p_times_l_m;
    vector3_scale(&a_p_times_l_m, &l_m, a_p_scalar + a_i_lm);
    Vector3_t a_i_kl_times_k_l;
    vector3_scale(&a_i_kl_times_k_l, &k_l, a_i_kl);
    Vector3_t a_iac;
    vector3_add(&a_iac, &a_p_times_l_m, &a_i_kl_times_k_l);

    /* Combine PN and IAC in local frame */
    Vector3_t accel_local;
    vector3_add(&accel_local, &a_p, &a_iac);

    /* === ATMOSPHERE MODEL & MACH NUMBER CALCULATION === */
    /* Calculate speed of sound at current altitude */
    double altitude = r_m_local.z; /* Local frame Z is altitude */
    double speed_of_sound;
    math_atmosphere(&speed_of_sound, altitude);

    /* Calculate Mach number */
    double mach = v / speed_of_sound;

    /* === LOCAL TO BODY FRAME TRANSFORMATION === */
    /* Convert acceleration command to body frame using updated local_to_body */
    Vector3_t accel_body_temp;
    local_to_body(&accel_body_temp, &accel_local, theta, psi, phi);

    /* === PITCH LIMITING === */
    /* Apply Mach-dependent limit to Y-component (pitch axis) */
    Vector3_t accel_body_limited;
    accel_body_limited.x = accel_body_temp.x;
    math_pitch_limit(&accel_body_limited.y, mach, accel_body_temp.y);
    accel_body_limited.z = accel_body_temp.z;

    /* === ACCELERATION RATE CLAMPING === */
    /* Get previous body acceleration from guidance state */
    static Vector3_t prev_accel = {0.0, 0.0, 0.0}; /* Persistent between calls */

    /* Calculate delta acceleration */
    Vector3_t delta_accel;
    vector3_subtract(&delta_accel, &accel_body_limited, &prev_accel);

    /* Clamp each component individually */
    /* X-axis clamping */
    if (fabs(delta_accel.x) > GUID_MAX_DELTA_ACC)
    {
        if (delta_accel.x > 0.0)
            accel_body->x = prev_accel.x + GUID_MAX_DELTA_ACC;
        else
            accel_body->x = prev_accel.x - GUID_MAX_DELTA_ACC;
    }
    else
    {
        accel_body->x = accel_body_limited.x;
    }

    /* Y-axis clamping */
    if (fabs(delta_accel.y) > GUID_MAX_DELTA_ACC)
    {
        if (delta_accel.y > 0.0)
            accel_body->y = prev_accel.y + GUID_MAX_DELTA_ACC;
        else
            accel_body->y = prev_accel.y - GUID_MAX_DELTA_ACC;
    }
    else
    {
        accel_body->y = accel_body_limited.y;
    }

    /* Z-axis clamping */
    if (fabs(delta_accel.z) > GUID_MAX_DELTA_ACC)
    {
        if (delta_accel.z > 0.0)
            accel_body->z = prev_accel.z + GUID_MAX_DELTA_ACC;
        else
            accel_body->z = prev_accel.z - GUID_MAX_DELTA_ACC;
    }
    else
    {
        accel_body->z = accel_body_limited.z;
    }

    /* Update previous acceleration for next cycle */
    prev_accel.x = accel_body->x;
    prev_accel.y = accel_body->y;
    prev_accel.z = accel_body->z;
}

/**
 * @brief Major cycle function - Called every 100ms (10Hz)
 *
 * This is the main entry point for the major cycle processing.
 * It processes GNSS data, converts to local frame, and executes guidance.
 */
void major_cycle(void)
{
    /* Step 1: Process GNSS data with fail-safe handling */
    process_gnss_data();

    /* Step 2: Process GNSS ECEF data and convert to local frame */
    process_gnss_ecef_data();

    /* Step 3: Execute guidance algorithm */
    if (gnss_position_ecef.valid && gnss_velocity_ecef.valid)
    {
        Vector3_t position_ecef;
        position_ecef.x = gnss_position_ecef.x_m;
        position_ecef.y = gnss_position_ecef.y_m;
        position_ecef.z = gnss_position_ecef.z_m;

        Vector3_t velocity_ecef;
        velocity_ecef.x = gnss_velocity_ecef.vx_ms;
        velocity_ecef.y = gnss_velocity_ecef.vy_ms;
        velocity_ecef.z = gnss_velocity_ecef.vz_ms;

        /* Calculate guidance acceleration */
        GuidanceState_t *guidState = &systemState.guidanceState;
        /* Get Euler angles from navigation state for body frame transformation */
        double theta = systemState.navigationState.attitude_e.pitch_rad;
        double psi = systemState.navigationState.attitude_e.yaw_rad;
        double phi = systemState.navigationState.attitude_e.roll_rad;

        /* Launch point and target are already stored as ECEF vectors */
        calculate_guidance_acceleration(&guidState->accelCmdBody,
                                        &position_ecef, &velocity_ecef,
                                        &mission_launch_point_ecef, &mission_target_ecef,
                                        guidance_theta_f, guidance_psi_f, theta, psi, phi);
    }

    /* Increment major cycle counter */
    systemState.majorCycleCount++;
}
