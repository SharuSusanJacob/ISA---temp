/******************************************************************************
 * ISA Flight Software
 * @file guidance.c
 * @brief Projectile Flight Computer Guidance Module Implementation
 * @details Implements proportional navigation with impact angle control,
 *          atmospheric model, Mach-dependent pitch limiting, and rate clamping
 * @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
 * @version 5.0.1
 *
 * MISRA C: Compliant Implementation
 *****************************************************************************/

#include "guidance.h"
#include "coordinate_transforms.h"
#include "math_utils.h"
#include "atmosphere.h"
#include "pitch_limit.h"
#include <stdio.h>
#include <math.h>
#include <float.h>

/* Helper: geodetic to ECEF (same as geodeticToECI in MATLAB) */
static void geodetic_to_ecef_helper(Vector3 *r_ecef, double lat_deg, double lon_deg, double alt_m)
{
    double lat_rad = DEG2RAD(lat_deg);
    double lon_rad = DEG2RAD(lon_deg);
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);
    double e2 = WGS84_E2;
    double n = WGS84_A / sqrt(1.0 - e2 * sin_lat * sin_lat);
    r_ecef->x = (n + alt_m) * cos_lat * cos_lon;
    r_ecef->y = (n + alt_m) * cos_lat * sin_lon;
    r_ecef->z = (n * (1.0 - e2) + alt_m) * sin_lat;
}

void onboard_guidance_algorithm(
    double lat_o, double lon_o, double alt_o,
    double lat_t, double lon_t, double alt_t,
    double t_in,
    double x_0_ecef, double y_0_ecef, double z_0_ecef,
    double vx_0_ecef, double vy_0_ecef, double vz_0_ecef,
    const Vector3 *prev_body_accel,
    double theta_f, double psi_f,
    GuidanceOutput *output)
{
    /* Target states in ECEF frame (assumed stationary) */
    Vector3 r_t_0_ecef;
    geodetic_to_ecef_helper(&r_t_0_ecef, lat_t, lon_t, alt_t);

    /* Current projectile states */
    Vector3 r_m_ecef = {x_0_ecef, y_0_ecef, z_0_ecef};
    Vector3 v_m_ecef = {vx_0_ecef, vy_0_ecef, vz_0_ecef};

    /* Convert projectile states to local frame */
    Vector3 r_m_local;
    ecef_to_local(&r_m_local, &r_m_ecef, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);
    Vector3 v_m_local;
    ecef_to_local_vel(&v_m_local, &v_m_ecef, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

    /* Current states in local frame */
    double z = r_m_local.z;

    /* Convert target to local frame */
    Vector3 r_t_local;
    ecef_to_local(&r_t_local, &r_t_0_ecef, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

    /* Vector to target FROM projectile */
    Vector3 r_vec;
    r_vec.x = r_t_local.x - r_m_local.x;
    r_vec.y = r_t_local.y - r_m_local.y;
    r_vec.z = r_t_local.z - r_m_local.z;

    /* Magnitude of line-of-sight vector */
    double r = v3_norm(&r_vec);

    /* Unit vector OF line-of-sight vector */
    Vector3 e_r;
    v3_normed(&e_r, &r_vec);

    /* Unit vector of velocity vector */
    double v = v3_norm(&v_m_local);
    Vector3 e_m;
    v3_normed(&e_m, &v_m_local);

    /* Current theta and psi */
    double theta = asin(v_m_local.z / v);
    double psi = atan2(v_m_local.y, v_m_local.x);
    double phi = 0.0;

    /* Lead angle */
    double sigma = acos(v3_dot(&e_m, &e_r));

    /* === GUIDANCE LAW CALCULATIONS === */

    /* Calculate guidance commands (PNG) */
    Vector3 cross_v_r;
    v3_cross(&cross_v_r, &v_m_local, &r_vec);
    double r_squared = r * r;
    Vector3 omega_l;
    v3_scale(&omega_l, &cross_v_r, 1.0 / r_squared);

    /* 3D PNG COMMAND */
    Vector3 n_omega_l;
    v3_scale(&n_omega_l, &omega_l, N_GAIN);
    Vector3 a_p;
    v3_cross(&a_p, &n_omega_l, &v_m_local);

    /* Unit vector perpendicular to both e_r and e_m */
    Vector3 cross_e_r_e_m;
    v3_cross(&cross_e_r_e_m, &e_r, &e_m);
    double cross_norm = v3_norm(&cross_e_r_e_m);
    Vector3 k_l;
    v3_scale(&k_l, &cross_e_r_e_m, 1.0 / (cross_norm + EPS));

    double eta = sigma / (N_GAIN - 1.0);
    double mu = sigma + eta;

    /* Quaternion for rotation */
    double w = cos((-mu) / 2.0);
    double x_q = sin((-mu) / 2.0) * k_l.x;
    double y_q = sin((-mu) / 2.0) * k_l.y;
    double z_q = sin((-mu) / 2.0) * k_l.z;

    /* Compute the rotation matrix */
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

    Vector3 e_p;
    e_p.x = l_q[0] * e_m.x + l_q[1] * e_m.y + l_q[2] * e_m.z;
    e_p.y = l_q[3] * e_m.x + l_q[4] * e_m.y + l_q[5] * e_m.z;
    e_p.z = l_q[6] * e_m.x + l_q[7] * e_m.y + l_q[8] * e_m.z;

    /* Under constant speed assumption */
    Vector3 v_p;
    v3_scale(&v_p, &e_p, v);

    /* Desired impact direction unit vector */
    Vector3 e_f;
    e_f.x = cos(theta_f) * cos(psi_f);
    e_f.y = cos(theta_f) * sin(psi_f);
    e_f.z = sin(theta_f);

    /* Impact angle error */
    double delta = acos(v3_dot(&e_f, &e_p));
    Vector3 cross_e_f_e_p;
    v3_cross(&cross_e_f_e_p, &e_f, &e_p);
    double cross_norm_lf = v3_norm(&cross_e_f_e_p);
    Vector3 l_f;
    v3_scale(&l_f, &cross_e_f_e_p, 1.0 / (cross_norm_lf + EPS));

    /* Time to go */
    double t_go = r / (v * cos(sigma));

    double sigma_not = sigma / SIGMA_MAX_RAD;
    double f_sigma_not = 1.0 - pow(fabs(sigma_not), PARAM_D);

    /* Calculate delta_dot */
    double delta_dot = (-PARAM_K * f_sigma_not / t_go) *
                       ((PARAM_A * sig(PARAM_M, delta)) + (PARAM_B * sig(PARAM_N, delta)));

    /* Angular velocity vector of V_P */
    Vector3 omega_p;
    v3_scale(&omega_p, &l_f, delta_dot);
    Vector3 a_f;
    v3_cross(&a_f, &omega_p, &v_p);

    Vector3 l_m;
    v3_cross(&l_m, &k_l, &e_m);
    Vector3 l_p;
    v3_cross(&l_p, &k_l, &e_p);

    /* Calculate h_delta */
    double h_delta = (PARAM_A * sig(PARAM_M, delta)) + (PARAM_B * sig(PARAM_N, delta));

    /* Components of guidance acceleration command vector */
    double a_p_scalar = ((-N_GAIN * (v * v) * sin(sigma)) / r);
    Vector3 cross_l_f_e_p;
    v3_cross(&cross_l_f_e_p, &l_f, &e_p);
    double a_i_lm = (((N_GAIN - 1.0) * PARAM_K * f_sigma_not * v * h_delta) / t_go) *
                    v3_dot(&cross_l_f_e_p, &l_p);
    double a_i_kl = ((PARAM_K * f_sigma_not * v * sin(sigma) * h_delta) / (t_go * sin(eta))) *
                    v3_dot(&cross_l_f_e_p, &k_l);

    /* Guidance acceleration command vector in local frame */
    Vector3 a_m_local_cal;
    Vector3 a_p_times_l_m;
    v3_scale(&a_p_times_l_m, &l_m, a_p_scalar + a_i_lm);
    Vector3 a_i_kl_times_k_l;
    v3_scale(&a_i_kl_times_k_l, &k_l, a_i_kl);
    v3_add(&a_m_local_cal, &a_p_times_l_m, &a_i_kl_times_k_l);

    /* === ATMOSPHERIC MODEL & MACH NUMBER === */
    double a_s = atmosphere(z);
    double mach = v / a_s;

    /* === LOCAL TO BODY TRANSFORMATION === */
    Vector3 a_m_body_temp;
    l2b(&a_m_body_temp, &a_m_local_cal, theta, psi, phi);

    /* === PITCH LIMITING === */
    Vector3 a_m_body_cal;
    a_m_body_cal.x = a_m_body_temp.x;
    a_m_body_cal.y = pitch_limit(mach, a_m_body_temp.y);
    a_m_body_cal.z = a_m_body_temp.z;

    /* === ACCELERATION RATE CLAMPING === */
    Vector3 delta_acc;
    v3_sub(&delta_acc, &a_m_body_cal, prev_body_accel);

    Vector3 a_m_body_final;

    /* Clamp X component */
    if (fabs(delta_acc.x) > MAX_DELTA_ACC)
    {
        if (delta_acc.x > 0.0)
            a_m_body_final.x = prev_body_accel->x + MAX_DELTA_ACC;
        else
            a_m_body_final.x = prev_body_accel->x - MAX_DELTA_ACC;
    }
    else
    {
        a_m_body_final.x = a_m_body_cal.x;
    }

    /* Clamp Y component */
    if (fabs(delta_acc.y) > MAX_DELTA_ACC)
    {
        if (delta_acc.y > 0.0)
            a_m_body_final.y = prev_body_accel->y + MAX_DELTA_ACC;
        else
            a_m_body_final.y = prev_body_accel->y - MAX_DELTA_ACC;
    }
    else
    {
        a_m_body_final.y = a_m_body_cal.y;
    }

    /* Clamp Z component */
    if (fabs(delta_acc.z) > MAX_DELTA_ACC)
    {
        if (delta_acc.z > 0.0)
            a_m_body_final.z = prev_body_accel->z + MAX_DELTA_ACC;
        else
            a_m_body_final.z = prev_body_accel->z - MAX_DELTA_ACC;
    }
    else
    {
        a_m_body_final.z = a_m_body_cal.z;
    }

    /* === FILL OUTPUT STRUCTURE === */
    output->t_go = t_go;
    output->r = r;
    output->theta_deg = RAD2DEG(theta);
    output->psi_deg = RAD2DEG(psi);
    output->A_M_body = a_m_body_final;
}
