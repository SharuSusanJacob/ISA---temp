/******************************************************************************
* ISA Flight Software
 * @file guidance.c
 * @brief Projectile Flight Computer Guidance Module Implementation
* @details Implements proportional navigation with impact angle control
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 3.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#include "guidance.h"
#include "math_utils.h"
#include "quaternion.h"
#include <math.h>
#include <stdio.h>


//=============================================================================
// STATIC HELPER FUNCTIONS
//=============================================================================

/**
 * @brief Calculate proportional navigation acceleration
 * @param result Output acceleration vector (m/s²)
 * @param position_local Projectile position in Local frame (m)
 * @param velocity_local Projectile velocity in Local frame (m/s)
 * @return GuidanceError_t Success or error code
 */

static GuidanceError_t calculate_proportional_navigation(Vector3_t* result, const Vector3_t* position_local, const Vector3_t* velocity_local, const Vector3_t* target_local)
{
    // Parameter validation
    if ((result == NULL) || (position_local == NULL) || (velocity_local == NULL) || (target_local == NULL)) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Create vector R from projectile to target (with terminal phase offset)
    Vector3_t R;
    R.x = target_local->x - position_local->x;
    R.y = target_local->y - position_local->y;
    R.z = (target_local->z + GUID_TERMINAL_DISTANCE_M) - position_local->z;

    // Calculate range to target with terminal phase offset
    double r;
    Vector3Error_t vec_error = vector3_magnitude(&r, &R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Safety check for division by zero
    if (r < MATH_EPSILON) {
        result->x = 0.0;
        result->y = 0.0;
        result->z = 0.0;
        return GUID_SUCCESS;
    }

    // Calculate line-of-sight unit vector
    Vector3_t e_R;
    vec_error = vector3_normalize(&e_R, &R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate velocity unit vector
    double velocity_mag;
    vec_error = vector3_magnitude(&velocity_mag, velocity_local);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Safety check for velocity magnitude
    if (velocity_mag < GUID_MIN_VELOCITY_M_S) {
        velocity_mag = GUID_MIN_VELOCITY_M_S;
    }

    Vector3_t e_m;
    vec_error = vector3_normalize(&e_m, velocity_local);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate sigma (angle between velocity and LOS)
    double cos_sigma;
    vec_error = vector3_dot(&cos_sigma, &e_m, &e_R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Clamp cos_sigma to [-1, 1]
    if (cos_sigma > 1.0) {
        cos_sigma = 1.0;
    }
    if (cos_sigma < -1.0) {
        cos_sigma = -1.0;
    }

    double sigma;
    MathUtilsError_t math_error = safe_acos(&sigma, cos_sigma);
    if (math_error != MATH_UTILS_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate line of sight rate vector (omega_L = (cross(V_m_local, R)) / (r^2))
    Vector3_t omega_L_numerator;
    vec_error = vector3_cross(&omega_L_numerator, velocity_local, &R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Scale by 1/r²
    double r_squared = r * r;
    if (r_squared < MATH_EPSILON) {
        r_squared = MATH_EPSILON;
    }

    Vector3_t omega_L;
    vec_error = vector3_scale(&omega_L, &omega_L_numerator, 1.0 / r_squared);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate proportional navigation acceleration (A_P = cross(N * omega_L, V_m_local))
    Vector3_t N_omega_L;
    vec_error = vector3_scale(&N_omega_L, &omega_L, GUID_NAVIGATION_GAIN_N);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    vec_error = vector3_cross(result, &N_omega_L, velocity_local);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    return GUID_SUCCESS;
}

/**
 * @brief Calculate impact angle control acceleration
 * @param result Output acceleration vector (m/s²)
 * @param position_local Projectile position in Local frame (m)
 * @param velocity_local Projectile velocity in Local frame (m/s)
 * @param theta_f Desired impact elevation angle (rad)
 * @param psi_f Desired impact azimuth angle (rad)
 * @param time_to_go Estimated time to impact (s)
 * @return GuidanceError_t Success or error code
 */

static GuidanceError_t calculate_impact_angle_control(Vector3_t* result, const Vector3_t* position_local, const Vector3_t* velocity_local, double theta_f, double psi_f, double time_to_go, const Vector3_t* target_local)
{
    // Parameter validation
    if ((result == NULL) || (position_local == NULL) || (velocity_local == NULL) || (target_local == NULL)) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Safety check for time to go
    if (time_to_go < MATH_EPSILON) {
        result->x = 0.0;
        result->y = 0.0;
        result->z = 0.0;
        return GUID_SUCCESS;
    }

    // Calculate velocity magnitude
    double velocity_mag;
    Vector3Error_t vec_error = vector3_magnitude(&velocity_mag, velocity_local);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Safety check for velocity magnitude
    if (velocity_mag < GUID_MIN_VELOCITY_M_S) {
        velocity_mag = GUID_MIN_VELOCITY_M_S;
    }

    // Calculate current velocity direction (unit vector)
    Vector3_t e_m;
    vec_error = vector3_normalize(&e_m, velocity_local);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate desired impact velocity direction
    Vector3_t e_f;
    double sin_theta_f, cos_theta_f, sin_psi_f, cos_psi_f;

    MathUtilsError_t math_error = safe_sin(&sin_theta_f, theta_f);
    if (math_error != MATH_UTILS_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }
    math_error = safe_cos(&cos_theta_f, theta_f);
    if (math_error != MATH_UTILS_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }
    math_error = safe_sin(&sin_psi_f, psi_f);
    if (math_error != MATH_UTILS_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }
    math_error = safe_cos(&cos_psi_f, psi_f);
    if (math_error != MATH_UTILS_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Desired velocity direction in Local Frame (East-North-Up)
    e_f.x = cos_theta_f * cos_psi_f;  // East
    e_f.y = cos_theta_f * sin_psi_f;  // North
    e_f.z = sin_theta_f;              // Up

    // Calculate LOS vector (with terminal phase offset)
    Vector3_t R;
    R.x = target_local->x - position_local->x;
    R.y = target_local->y - position_local->y;
    R.z = (target_local->z + GUID_TERMINAL_DISTANCE_M) - position_local->z;

    // Calculate range
    double r;
    vec_error = vector3_magnitude(&r, &R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate LOS unit vector
    Vector3_t e_R;
    vec_error = vector3_normalize(&e_R, &R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate sigma (angle between velocity and LOS)
    double cos_sigma;
    vec_error = vector3_dot(&cos_sigma, &e_m, &e_R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Clamp cos_sigma to [-1, 1]
    if (cos_sigma > 1.0) {
        cos_sigma = 1.0;
    }
    if (cos_sigma < -1.0) {
        cos_sigma = -1.0;
    }

    // Calculate sigma
    double sigma;
    math_error = safe_acos(&sigma, cos_sigma);
    if (math_error != MATH_UTILS_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate k_L (rotation axis)
    Vector3_t e_R_cross_e_m;
    vec_error = vector3_cross(&e_R_cross_e_m, &e_R, &e_m);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    double e_R_cross_e_m_mag;
    vec_error = vector3_magnitude(&e_R_cross_e_m_mag, &e_R_cross_e_m);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Avoid division by zero (matching MATLAB: eps = 0.0001)
    double safe_mag = e_R_cross_e_m_mag + GUID_EPSILON;

    Vector3_t k_L;
    vec_error = vector3_scale(&k_L, &e_R_cross_e_m, 1.0 / safe_mag);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate eta and mu
    double eta = sigma / (GUID_NAVIGATION_GAIN_N - 1.0);
    double mu = sigma + eta;

    // Create quaternion components
    double w = cos((-mu) / 2.0);
    double x_q = sin((-mu) / 2.0) * k_L.x;
    double y_q = sin((-mu) / 2.0) * k_L.y;
    double z_q = sin((-mu) / 2.0) * k_L.z;

    // Create quaternion
    Quaternion_t q;
    QuaternionError_t quat_error = quaternion_create(&q, w, x_q, y_q, z_q);
    if (quat_error != QUATERNION_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Create rotation matrix from quaternion
    double L_q[9];
    quat_error = quaternion_to_rotation_matrix(L_q, &q);
    if (quat_error != QUATERNION_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate e_p = L_q * e_m
    Vector3_t e_p;
    e_p.x = L_q[0] * e_m.x + L_q[1] * e_m.y + L_q[2] * e_m.z;
    e_p.y = L_q[3] * e_m.x + L_q[4] * e_m.y + L_q[5] * e_m.z;
    e_p.z = L_q[6] * e_m.x + L_q[7] * e_m.y + L_q[8] * e_m.z;

    // Calculate V_p = v * e_p
    Vector3_t V_p;
    vec_error = vector3_scale(&V_p, &e_p, velocity_mag);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate delta (angle between e_f and e_p)
    double cos_delta;
    vec_error = vector3_dot(&cos_delta, &e_f, &e_p);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Clamp cos_delta to [-1, 1]
    if (cos_delta > 1.0) {
        cos_delta = 1.0;
    }
    if (cos_delta < -1.0) {
        cos_delta = -1.0;
    }

    double delta;
    math_error = safe_acos(&delta, cos_delta);
    if (math_error != MATH_UTILS_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate l_f
    Vector3_t e_f_cross_e_p;
    vec_error = vector3_cross(&e_f_cross_e_p, &e_f, &e_p);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    double e_f_cross_e_p_mag;
    vec_error = vector3_magnitude(&e_f_cross_e_p_mag, &e_f_cross_e_p);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Avoid division by zero (matching MATLAB: eps = 0.0001)
    double safe_mag_l_f = e_f_cross_e_p_mag + GUID_EPSILON;

    Vector3_t l_f;
    vec_error = vector3_scale(&l_f, &e_f_cross_e_p, 1.0 / safe_mag_l_f);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate sigma_not and f_sigma_not
    double sigma_not = sigma / GUID_MAX_LOOK_ANGLE_RAD;
    double abs_sigma_not = fabs(sigma_not);
    double abs_sigma_not_pow_d;

    // Calculate abs_sigma_not^d
    abs_sigma_not_pow_d = pow(abs_sigma_not, GUID_PARAM_D);

    double f_sigma_not = 1.0 - abs_sigma_not_pow_d;

    // Calculate delta_dot using sig function
    double sig_m_delta, sig_n_delta;
    math_error = sig_function(&sig_m_delta, GUID_PARAM_M, delta);
    if (math_error != MATH_UTILS_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    math_error = sig_function(&sig_n_delta, GUID_PARAM_N, delta);
    if (math_error != MATH_UTILS_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    double h_delta = (GUID_PARAM_A * sig_m_delta) + (GUID_PARAM_B * sig_n_delta);
    double delta_dot = (-GUID_PARAM_K * f_sigma_not / time_to_go) * h_delta;

    // Calculate omega_P = delta_dot * l_f
    Vector3_t omega_P;
    vec_error = vector3_scale(&omega_P, &l_f, delta_dot);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate A_F = cross(omega_P, V_p)
    Vector3_t A_F;
    vec_error = vector3_cross(&A_F, &omega_P, &V_p);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate l_m and l_P
    Vector3_t l_m, l_P;
    vec_error = vector3_cross(&l_m, &k_L, &e_m);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    vec_error = vector3_cross(&l_P, &k_L, &e_p);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate a_p
    double a_p = (-GUID_NAVIGATION_GAIN_N * (velocity_mag * velocity_mag) * sin(sigma)) / r;

    // Calculate cross(l_f, e_p)
    Vector3_t l_f_cross_e_p;
    vec_error = vector3_cross(&l_f_cross_e_p, &l_f, &e_p);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate dot(cross(l_f, e_p), l_P)
    double dot_cross_l_P;
    vec_error = vector3_dot(&dot_cross_l_P, &l_f_cross_e_p, &l_P);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate dot(cross(l_f, e_p), k_L)
    double dot_cross_k_L;
    vec_error = vector3_dot(&dot_cross_k_L, &l_f_cross_e_p, &k_L);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate a_I_lm
    double a_I_lm = (((GUID_NAVIGATION_GAIN_N - 1.0) * GUID_PARAM_K * f_sigma_not * velocity_mag * h_delta) / time_to_go) * dot_cross_l_P;

    // Calculate a_I_kl (avoid division by zero for sin(eta))
    double sin_eta = sin(eta);
    if (fabs(sin_eta) < MATH_EPSILON) {
        sin_eta = MATH_EPSILON;
    }

    double a_I_kl = ((GUID_PARAM_K * f_sigma_not * velocity_mag * sin(sigma) * h_delta) / (time_to_go * sin_eta)) * dot_cross_k_L;

    // Calculate final acceleration command
    Vector3_t a_p_plus_a_I_lm_times_l_m;
    vec_error = vector3_scale(&a_p_plus_a_I_lm_times_l_m, &l_m, a_p + a_I_lm);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    Vector3_t a_I_kl_times_k_L;
    vec_error = vector3_scale(&a_I_kl_times_k_L, &k_L, a_I_kl);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // A_M_local = (a_p + a_I_lm) * l_m + a_I_kl * k_L
    vec_error = vector3_add(result, &a_p_plus_a_I_lm_times_l_m, &a_I_kl_times_k_L);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    return GUID_SUCCESS;
}

/**
 * @brief Calculate drag acceleration
 * @param result Output drag acceleration vector (m/s²)
 * @param velocity_eci Velocity in ECI frame (m/s)
 * @return GuidanceError_t Success or error code
 */

static GuidanceError_t calculate_drag_acceleration(Vector3_t* result, const Vector3_t* velocity_eci)
{
    // Parameter validation
    if ((result == NULL) || (velocity_eci == NULL)) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Calculate velocity magnitude
    double velocity_mag;
    Vector3Error_t vec_error = vector3_magnitude(&velocity_mag, velocity_eci);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Safety check for velocity magnitude
    if (velocity_mag < MATH_EPSILON) {
        result->x = 0.0;
        result->y = 0.0;
        result->z = 0.0;
        return GUID_SUCCESS;
    }

    // Calculate drag force magnitude 
    // (D = 0.5 * ρ * v² * C_d * S_ref)
    double drag_force = 0.5 * GUID_AIR_DENSITY_KG_M3 * velocity_mag * velocity_mag *
        GUID_DRAG_COEFFICIENT * GUID_REFERENCE_AREA_M2;

    // Calculate drag acceleration components directly
    // MATLAB: A_D_ECI = [Dx/m_c; Dy/m_c; Dz/m_c];
    // Where Dx, Dy, Dz are components of drag force

    // Calculate drag acceleration direction (opposite to velocity)
    Vector3_t velocity_dir;
    vec_error = vector3_normalize(&velocity_dir, velocity_eci);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate drag force components
    double drag_accel_mag = drag_force / GUID_PROJECTILE_MASS_KG;

    // Scale by positive drag acceleration magnitude (same direction as velocity)
    // In MATLAB, drag is reported in the same direction as velocity
    vec_error = vector3_scale(result, &velocity_dir, drag_accel_mag);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    return GUID_SUCCESS;
}

/**
 * @brief Calculate gravity acceleration in ECEF frame
 * @param result Output gravity acceleration vector (m/s²)
 * @param position_ecef Position in ECEF frame (m)
 * @param launch Launch point geodetic position
 * @param target Target geodetic position
 * @return GuidanceError_t Success or error code
 */

static GuidanceError_t calculate_gravity_acceleration(Vector3_t* result, const Vector3_t* position_eci, const GeodeticPos_t* launch, const GeodeticPos_t* target)
{
    // Parameter validation
    if ((result == NULL) || (position_eci == NULL) || (launch == NULL) || (target == NULL)) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Implementation: grav_ECEF = Local_to_ECEF_vel([0; 0; -g], lat_O, lon_O, alt_O, lat_T, lon_T, alt_T);
    // Create gravity vector in local frame: [0, 0, -g]
    Vector3_t gravity_local;
    gravity_local.x = 0.0;
    gravity_local.y = 0.0;
    gravity_local.z = -GUID_GRAVITY_M_S2;  // Negative because local Z points up

    // Transform local gravity to ECEF
    CoordTransformError_t coord_error = local_to_ecef_velocity(result, &gravity_local, launch, target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    return GUID_SUCCESS;
}

/**
 * @brief Estimate time to impact
 * @param result Output time to impact (s)
 * @param position_local Position in Local frame (m)
 * @param velocity_local Velocity in Local frame (m/s)
 * @return GuidanceError_t Success or error code
 */

static GuidanceError_t calculate_time_to_go(double* result, const Vector3_t* position_local, const Vector3_t* velocity_local, const Vector3_t* target_local)
{
    // Parameter validation
    if ((result == NULL) || (position_local == NULL) || (velocity_local == NULL) || (target_local == NULL)) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Create vector R from projectile to target (with terminal phase offset)
    Vector3_t R;
    R.x = target_local->x - position_local->x;
    R.y = target_local->y - position_local->y;
    R.z = (target_local->z + GUID_TERMINAL_DISTANCE_M) - position_local->z;

    // Calculate range to target with terminal phase offset
    double r;
    Vector3Error_t vec_error = vector3_magnitude(&r, &R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate LOS unit vector
    Vector3_t e_R;
    vec_error = vector3_normalize(&e_R, &R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate velocity magnitude
    double v;
    vec_error = vector3_magnitude(&v, velocity_local);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Safety check for velocity magnitude
    if (v < GUID_MIN_VELOCITY_M_S) {
        v = GUID_MIN_VELOCITY_M_S;
    }

    // Calculate velocity unit vector
    Vector3_t e_m;
    vec_error = vector3_normalize(&e_m, velocity_local);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate cos(sigma) - angle between velocity and LOS
    double cos_sigma;
    vec_error = vector3_dot(&cos_sigma, &e_m, &e_R);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Safety check for division by zero and negative values
    if (fabs(cos_sigma) < MATH_EPSILON) {
        cos_sigma = MATH_EPSILON;
    }

    // For launch-centered coordinates, the target is far away and the projectile
    // is initially moving away from the origin, which makes cos(sigma) negative.
    // This causes time-to-go to be negative.

    // Calculate time-to-go based on distance and velocity
    // If cos(sigma) is positive (moving toward target), use r / (v * cos(sigma))
    // Otherwise, use a simple approximation r / v
    double t_go;
    if (cos_sigma > 0.0) {
        t_go = r / (v * cos_sigma);
    }
    else {
        t_go = r / v;
    }

    // Ensure time-to-go is never negative (especially important at end of simulation)
    *result = (t_go < 0.0) ? 0.0 : t_go;

    return GUID_SUCCESS;
}


//=============================================================================
// PUBLIC FUNCTIONS
//=============================================================================

GuidanceError_t guidance_init(GuidanceState_t* state)
{
    // Parameter validation
    if (state == NULL) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Initialize state structure to zero
    memset(state, 0, sizeof(GuidanceState_t));

    // Set default values
    state->guidance_active = false;
    state->terminal_phase = false;
    state->time_s = 0.0;
    state->integration_cycles = 0U;

    // Initialize GT3 flag fields
    state->gt3_check_active = false;
    state->gt3_printed = false;
    state->gt3_previous = INFINITY;
    state->consecutive_decreases = 0U;

    return GUID_SUCCESS;
}

GuidanceError_t guidance_set_launch(GuidanceState_t* state, const GeodeticPos_t* launch)
{
    // Parameter validation
    if ((state == NULL) || (launch == NULL)) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Copy launch information
    state->launch = *launch;

    return GUID_SUCCESS;
}

GuidanceError_t guidance_set_target(GuidanceState_t* state, const GeodeticPos_t* target)
{
    // Parameter validation
    if ((state == NULL) || (target == NULL)) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Copy target information
    state->target = *target;

    return GUID_SUCCESS;
}

GuidanceError_t guidance_set_impact_angles(GuidanceState_t* state, double theta_f_rad, double psi_f_rad)
{
    // Parameter validation
    if (state == NULL) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Validate impact angles
    if ((theta_f_rad < -MATH_PI) || (theta_f_rad > MATH_PI) || (psi_f_rad < -MATH_PI) || (psi_f_rad > MATH_PI)) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Set impact angles
    state->theta_f_rad = theta_f_rad;
    state->psi_f_rad = psi_f_rad;

    return GUID_SUCCESS;
}
GuidanceError_t guidance_execute(GuidanceState_t* state, double dt_s)
{
    // Parameter validation
    if (state == NULL) {
        return GUID_ERROR_INVALID_PARAM;
    }

    // Check if guidance is active
    if (!state->guidance_active) {
        // Guidance not active, no acceleration command
        state->acceleration_cmd_ecef.x = 0.0;
        state->acceleration_cmd_ecef.y = 0.0;
        state->acceleration_cmd_ecef.z = 0.0;
        return GUID_SUCCESS;
    }

    // Update guidance time
    state->time_s += dt_s;
    state->integration_cycles++;

    // Convert projectile position and velocity to Local frame
    CoordTransformError_t coord_error = ecef_to_local(&state->position_local, &state->position_ecef, &state->launch, &state->target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    coord_error = ecef_to_local_velocity(&state->velocity_local, &state->velocity_ecef, &state->launch, &state->target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Create vector R from projectile to target
    // We need to calculate target position in local frame
    Vector3_t target_ecef;
    CoordTransformError_t coord_error_target = geodetic_to_ecef(&target_ecef, &state->target);
    if (coord_error_target != COORD_TRANSFORM_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    Vector3_t target_local;
    coord_error_target = ecef_to_local(&target_local, &target_ecef, &state->launch, &state->target);
    if (coord_error_target != COORD_TRANSFORM_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // R = Target - Projectile (with terminal phase offset for guidance calculations)
    // This matches MATLAB: R = [R_t_local(1); R_t_local(2); R_t_local(3) + r_loop3_start] - R_m_local
    Vector3_t R_with_offset;
    R_with_offset.x = target_local.x - state->position_local.x;
    R_with_offset.y = target_local.y - state->position_local.y;
    R_with_offset.z = (target_local.z + GUID_TERMINAL_DISTANCE_M) - state->position_local.z;

    // Calculate range with offset (this is what MATLAB uses for terminal phase check)
    double r_with_offset;
    Vector3Error_t vec_error = vector3_magnitude(&r_with_offset, &R_with_offset);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Also calculate actual distance to target (for display/logging)
    Vector3_t R_actual;
    R_actual.x = target_local.x - state->position_local.x;
    R_actual.y = target_local.y - state->position_local.y;
    R_actual.z = target_local.z - state->position_local.z;

    vec_error = vector3_magnitude(&state->distance_to_target_m, &R_actual);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate time to go
    GuidanceError_t guid_error = calculate_time_to_go(&state->time_to_go_s, &state->position_local, &state->velocity_local, &target_local);
    if (guid_error != GUID_SUCCESS) {
        return guid_error;
    }

    // GT3 flag generation
    if (!state->gt3_printed) {
        if (!state->gt3_check_active && state->time_to_go_s <= GUID_GT3_THRESHOLD) {
            // First time we cross the threshold
            state->gt3_check_active = true;
            state->gt3_previous = state->time_to_go_s;
            state->consecutive_decreases = 1U;
        }
        else if (state->gt3_check_active) {
            if (state->time_to_go_s < state->gt3_previous) {
                state->consecutive_decreases++;
                state->gt3_previous = state->time_to_go_s;

                if (state->consecutive_decreases >= GUID_GT3_CONSECUTIVE_COUNT) {
                    printf("FLAG: GT3\n");
                    state->gt3_printed = true;  // Set flag to prevent repetition
                    state->gt3_check_active = false;  // Stop further checks
                    state->consecutive_decreases = 0U;
                }
            }
            else {
                // Reset if t_go doesn't decrease
                state->consecutive_decreases = 0U;
                state->gt3_previous = state->time_to_go_s;
            }
        }
    }

    // Check if it should enter the terminal phase
    // MATLAB checks: if r <= r_loop3_start, where r includes the offset
    if (!state->terminal_phase && (r_with_offset <= GUID_TERMINAL_DISTANCE_M)) {
        state->terminal_phase = true;
        printf("Entered terminal phase at t=%.2lf s, distance=%.2lf m\n",
            state->time_s, r_with_offset);
    }

    // Calculate guidance acceleration in local frame
    Vector3_t accel_guidance_local;

    if (!state->terminal_phase) {
        // Guided phase: Proportional navigation + Impact angle control
        Vector3_t accel_pn, accel_iac;

        // Calculate proportional navigation acceleration
        guid_error = calculate_proportional_navigation(&accel_pn, &state->position_local, &state->velocity_local, &target_local);
        if (guid_error != GUID_SUCCESS) {
            return guid_error;
        }

        // Calculate impact angle control acceleration
        guid_error = calculate_impact_angle_control(&accel_iac, &state->position_local, &state->velocity_local, state->theta_f_rad, state->psi_f_rad, state->time_to_go_s, &target_local);
        if (guid_error != GUID_SUCCESS) {
            return guid_error;
        }

        // Combine accelerations
        vec_error = vector3_add(&accel_guidance_local, &accel_pn, &accel_iac);
        if (vec_error != VECTOR3_SUCCESS) {
            return GUID_ERROR_MATH_ERROR;
        }
    }
    else {
        // Terminal phase: No guidance (ballistic)
        accel_guidance_local.x = 0.0;
        accel_guidance_local.y = 0.0;
        accel_guidance_local.z = 0.0;
    }

    // Store local acceleration command for output
    state->acceleration_cmd_local = accel_guidance_local;

    // Calculate Euler angles from velocity in local frame (for body frame transformation)
    // theta: pitch angle (elevation), psi: yaw angle (azimuth), phi: roll angle
    double velocity_mag;
    vec_error = vector3_magnitude(&velocity_mag, &state->velocity_local);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Safety check for velocity magnitude
    if (velocity_mag < GUID_MIN_VELOCITY_M_S) {
        velocity_mag = GUID_MIN_VELOCITY_M_S;
    }

    // Calculate theta (pitch) and psi (yaw) from velocity components
    // theta = asin(vz / v) - elevation angle
    // psi = atan2(vy, vx) - azimuth angle
    // phi = 0 - roll angle (assumed zero, matching MATLAB)
    double theta, psi;
    double phi = 0.0;  // Roll angle (assumed zero)

    double sin_theta = state->velocity_local.z / velocity_mag;

    // Clamp sin_theta to [-1, 1] for safe asin
    if (sin_theta > 1.0) {
        sin_theta = 1.0;
    }
    if (sin_theta < -1.0) {
        sin_theta = -1.0;
    }

    // Calculate theta (pitch angle) - safe because we clamped sin_theta
    theta = asin(sin_theta);

    // Calculate psi (yaw angle) using atan2 for proper quadrant handling
    psi = atan2(state->velocity_local.y, state->velocity_local.x);

    // Transform guidance acceleration from Local to Body frame
    // This is the input to DAP (Digital Autopilot)
    coord_error = local_to_body(&state->acceleration_cmd_body, &accel_guidance_local, theta, psi, phi);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Convert guidance acceleration to ECEF frame
    coord_error = local_to_ecef_velocity(&state->pure_guidance_ecef, &accel_guidance_local, &state->launch, &state->target);
    if (coord_error != COORD_TRANSFORM_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Calculate drag acceleration in ECEF frame
    guid_error = calculate_drag_acceleration(&state->drag_acceleration_ecef, &state->velocity_ecef);
    if (guid_error != GUID_SUCCESS) {
        return guid_error;
    }

    // Calculate gravity acceleration in ECEF frame
    guid_error = calculate_gravity_acceleration(&state->gravity_acceleration_ecef, &state->position_ecef, &state->launch, &state->target);
    if (guid_error != GUID_SUCCESS) {
        return guid_error;
    }

    // Combine accelerations as: A_total_ECEF = A_M_ECEF - A_D_ECEF + grav_ECEF
    // Since we're now storing drag in the same direction as velocity,
    // we need to subtract it (which means adding the negative)
    Vector3_t guidance_minus_drag;
    Vector3_t negative_drag;
    vec_error = vector3_scale(&negative_drag, &state->drag_acceleration_ecef, -1.0);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    vec_error = vector3_add(&guidance_minus_drag, &state->pure_guidance_ecef, &negative_drag);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    // Then add gravity
    vec_error = vector3_add(&state->acceleration_cmd_ecef, &guidance_minus_drag, &state->gravity_acceleration_ecef);
    if (vec_error != VECTOR3_SUCCESS) {
        return GUID_ERROR_MATH_ERROR;
    }

    return GUID_SUCCESS;
}

