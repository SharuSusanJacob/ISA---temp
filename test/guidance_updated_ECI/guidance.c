/******************************************************************************
 * ISA Flight Software
 * @file guidance.c
 * @brief Projectile Flight Computer Guidance Module Implementation
 * @details Implements proportional navigation with impact angle control,
 *          atmospheric model, Mach-dependent pitch limiting, and rate clamping.
 *          Updated to use ECEF -> ECI -> Local frame flow with GPS time.
 * @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2026
 * @version 6.0.0
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

/* ========== CONSTANTS ========== */
#define OMEGA_EARTH 7.2921150e-5 /* Earth rotation rate [rad/s] */
#define GPS_EPOCH_JD 2444244.5   /* Julian Date of GPS Epoch (1980-01-06 00:00:00 UTC) */

/* ========== HELPER: Geodetic to ECEF (WGS-84) ========== */
/* Matches MATLAB geodeticToECI.m (which is actually geodetic to ECEF) */
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

/* ========== HELPER: GPS Time to Julian Date ========== */
/* Matches MATLAB ECEF_to_ECI_NAV.m logic */
static double gps_to_julian_date(int gnss_week, double tow, int leap_seconds)
{
    /* GPS epoch: 1980-01-06 00:00:00 UTC */
    /* Julian Date of GPS epoch = 2444244.5 */
    double gps_seconds = (double)gnss_week * 7.0 * 86400.0 + tow;
    double utc_seconds = gps_seconds - (double)leap_seconds;

    /* Convert UTC seconds since GPS epoch to Julian Date */
    double jd = GPS_EPOCH_JD + utc_seconds / 86400.0;
    return jd;
}

/* ========== HELPER: Julian Date to GMST (radians) ========== */
/* Matches MATLAB ECEF_to_ECI_NAV.m logic */
static double julian_date_to_gmst_rad(double jd)
{
    double t = (jd - 2451545.0) / 36525.0;
    double gmst_deg = 280.46061837 + 360.98564736629 * (jd - 2451545.0) + t * t * (0.000387933 - t) / 38710000.0;

    /* Normalize to [0, 360) degrees */
    gmst_deg = fmod(gmst_deg, 360.0);
    if (gmst_deg < 0.0)
    {
        gmst_deg += 360.0;
    }

    /* LST for longitude=0 is just GMST */
    double lst_rad = DEG2RAD(gmst_deg);
    return lst_rad;
}

/* ========== HELPER: ECEF to ECI Conversion ========== */
/* Port of MATLAB ECEF_to_ECI_NAV.m */
static void ecef_to_eci_nav(Vector3 *r_eci, Vector3 *v_eci,
                            const Vector3 *r_ecef, const Vector3 *v_ecef,
                            int gnss_week, double tow, int leap_seconds)
{
    /* Step 1: Compute Julian Date from GPS time */
    double jd = gps_to_julian_date(gnss_week, tow, leap_seconds);

    /* Step 2: Compute GMST (Local Sidereal Time at longitude=0) */
    double lst_rad = julian_date_to_gmst_rad(jd);

    /* Step 3: Build rotation matrix R_ecef2eci */
    double cos_lst = cos(lst_rad);
    double sin_lst = sin(lst_rad);

    /* R_ecef2eci = [cos(LST), -sin(LST), 0;
                     sin(LST),  cos(LST), 0;
                     0,         0,        1] */

    /* Step 4: r_ECI = R_ecef2eci * r_ECEF */
    r_eci->x = cos_lst * r_ecef->x - sin_lst * r_ecef->y;
    r_eci->y = sin_lst * r_ecef->x + cos_lst * r_ecef->y;
    r_eci->z = r_ecef->z;

    /* Step 5: v_ECI = R_ecef2eci * v_ECEF + omega_vec x r_ECI */
    /* omega_vec = [0; 0; omega_earth] */
    /* cross([0;0;w], [x;y;z]) = [-w*y; w*x; 0] */
    double v_rot_x = cos_lst * v_ecef->x - sin_lst * v_ecef->y;
    double v_rot_y = sin_lst * v_ecef->x + cos_lst * v_ecef->y;
    double v_rot_z = v_ecef->z;

    v_eci->x = v_rot_x + (-OMEGA_EARTH * r_eci->y);
    v_eci->y = v_rot_y + (OMEGA_EARTH * r_eci->x);
    v_eci->z = v_rot_z;
}

/* ========== HELPER: ECI to Local Frame (Position) ========== */
/* Port of MATLAB ECI_to_Local.m */
/* Note: Uses the same local frame definition (X: origin->target, Z: up, Y: right-hand) */
/* The key is that the rotation matrix is built from ECEF basis, then ECI ≈ ECEF for axes */
static void eci_to_local(Vector3 *r_local, const Vector3 *r_eci,
                         double lat_o, double lon_o, double alt_o,
                         double lat_t, double lon_t, double alt_t)
{
    /* Build local frame axes in ECEF (matching MATLAB) */
    Vector3 launch_ecef, target_ecef, perturb_ecef;
    geodetic_to_ecef_helper(&launch_ecef, lat_o, lon_o, alt_o);
    geodetic_to_ecef_helper(&target_ecef, lat_t, lon_t, alt_t);
    geodetic_to_ecef_helper(&perturb_ecef, lat_o, lon_o, alt_o + 1.0);

    /* Z-axis: radially outward (surface normal) */
    Vector3 z_local_ecef;
    v3_sub(&z_local_ecef, &perturb_ecef, &launch_ecef);
    v3_normed(&z_local_ecef, &z_local_ecef);

    /* X-axis: from launch to target */
    Vector3 x_local_ecef;
    v3_sub(&x_local_ecef, &target_ecef, &launch_ecef);
    v3_normed(&x_local_ecef, &x_local_ecef);

    /* Y-axis: completes right-hand system */
    Vector3 y_local_ecef;
    v3_cross(&y_local_ecef, &z_local_ecef, &x_local_ecef);
    v3_normed(&y_local_ecef, &y_local_ecef);

    /* R_ECI_to_Local = [x_local_ecef, y_local_ecef, z_local_ecef]' */
    /* MATLAB: vec_local = R_ECI_to_Local * (vec_eci - [x_O; y_O; z_O]) */
    /* Note: MATLAB comment says "assume ECI ≈ ECEF" for the rotation matrix definition */
    Vector3 rel;
    v3_sub(&rel, r_eci, &launch_ecef);

    r_local->x = v3_dot(&x_local_ecef, &rel);
    r_local->y = v3_dot(&y_local_ecef, &rel);
    r_local->z = v3_dot(&z_local_ecef, &rel);
}

/* ========== HELPER: ECI to Local Frame (Velocity) ========== */
/* Port of MATLAB ECI_to_Local_vel.m */
/* For velocity, we don't subtract the origin position - just rotate */
static void eci_to_local_vel(Vector3 *v_local, const Vector3 *v_eci,
                             double lat_o, double lon_o, double alt_o,
                             double lat_t, double lon_t, double alt_t)
{
    /* Build local frame axes in ECEF (matching MATLAB) */
    Vector3 launch_ecef, target_ecef, perturb_ecef;
    geodetic_to_ecef_helper(&launch_ecef, lat_o, lon_o, alt_o);
    geodetic_to_ecef_helper(&target_ecef, lat_t, lon_t, alt_t);
    geodetic_to_ecef_helper(&perturb_ecef, lat_o, lon_o, alt_o + 1.0);

    /* Z-axis */
    Vector3 z_local_ecef;
    v3_sub(&z_local_ecef, &perturb_ecef, &launch_ecef);
    v3_normed(&z_local_ecef, &z_local_ecef);

    /* X-axis */
    Vector3 x_local_ecef;
    v3_sub(&x_local_ecef, &target_ecef, &launch_ecef);
    v3_normed(&x_local_ecef, &x_local_ecef);

    /* Y-axis */
    Vector3 y_local_ecef;
    v3_cross(&y_local_ecef, &z_local_ecef, &x_local_ecef);
    v3_normed(&y_local_ecef, &y_local_ecef);

    /* MATLAB: vec_local = R_ECI_to_Local * vec_eci (no origin subtraction) */
    v_local->x = v3_dot(&x_local_ecef, v_eci);
    v_local->y = v3_dot(&y_local_ecef, v_eci);
    v_local->z = v3_dot(&z_local_ecef, v_eci);
}

/* ========== MAIN GUIDANCE ALGORITHM ========== */
void onboard_guidance_algorithm(
    double lat_o, double lon_o, double alt_o,
    double lat_t, double lon_t, double alt_t,
    int gnss_week, double tow, int leap_seconds,
    double x_0_ecef, double y_0_ecef, double z_0_ecef,
    double vx_0_ecef, double vy_0_ecef, double vz_0_ecef,
    const Vector3 *prev_body_accel,
    double theta_f, double psi_f,
    GuidanceOutput *output)
{
    /* ===== STEP 1: Target states in ECEF then ECI ===== */
    /* Target is assumed stationary in ECEF, convert to ECI via geodeticToECI */
    /* In MATLAB: R_t_0_ECI = geodeticToECI(lat_T, lon_T, alt_T) */
    /* Note: geodeticToECI is actually geodetic->ECEF, but target is fixed so ECI ≈ ECEF position */
    Vector3 r_t_0_eci;
    geodetic_to_ecef_helper(&r_t_0_eci, lat_t, lon_t, alt_t);
    /* V_t_0_ECI = [0; 0; 0] - target stationary */

    /* ===== STEP 2: Projectile ECEF -> ECI ===== */
    Vector3 r_m_ecef = {x_0_ecef, y_0_ecef, z_0_ecef};
    Vector3 v_m_ecef = {vx_0_ecef, vy_0_ecef, vz_0_ecef};

    Vector3 r_m_eci, v_m_eci;
    ecef_to_eci_nav(&r_m_eci, &v_m_eci, &r_m_ecef, &v_m_ecef, gnss_week, tow, leap_seconds);

    /* ===== STEP 3: ECI -> Local Frame ===== */
    Vector3 r_m_local;
    eci_to_local(&r_m_local, &r_m_eci, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

    Vector3 v_m_local;
    eci_to_local_vel(&v_m_local, &v_m_eci, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

    /* Current altitude for atmosphere */
    double z = r_m_local.z;

    /* Convert target to local frame */
    Vector3 r_t_local;
    eci_to_local(&r_t_local, &r_t_0_eci, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

    /* ===== STEP 4: Guidance Geometry ===== */
    /* Vector to target FROM projectile: R = R_t - R_m */
    Vector3 r_vec;
    r_vec.x = r_t_local.x - r_m_local.x;
    r_vec.y = r_t_local.y - r_m_local.y;
    r_vec.z = r_t_local.z - r_m_local.z;

    double r = v3_norm(&r_vec);

    Vector3 e_r;
    v3_normed(&e_r, &r_vec);

    double v = v3_norm(&v_m_local);
    Vector3 e_m;
    v3_normed(&e_m, &v_m_local);

    /* Current theta and psi */
    double theta = asin(v_m_local.z / v);
    double psi = atan2(v_m_local.y, v_m_local.x);
    double phi = 0.0;

    /* Lead angle */
    double sigma = acos(v3_dot(&e_m, &e_r));

    /* ===== GUIDANCE LAW CALCULATIONS ===== */

    /* PNG: omega_L = (V x R) / r^2 */
    Vector3 cross_v_r;
    v3_cross(&cross_v_r, &v_m_local, &r_vec);
    double r_squared = r * r;
    Vector3 omega_l;
    v3_scale(&omega_l, &cross_v_r, 1.0 / r_squared);

    /* A_P = N * omega_L x V */
    Vector3 n_omega_l;
    v3_scale(&n_omega_l, &omega_l, N_GAIN);
    Vector3 a_p;
    v3_cross(&a_p, &n_omega_l, &v_m_local);

    /* k_L = (e_R x e_m) / |e_R x e_m| */
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

    /* e_p = L_q * e_m */
    Vector3 e_p;
    e_p.x = l_q[0] * e_m.x + l_q[1] * e_m.y + l_q[2] * e_m.z;
    e_p.y = l_q[3] * e_m.x + l_q[4] * e_m.y + l_q[5] * e_m.z;
    e_p.z = l_q[6] * e_m.x + l_q[7] * e_m.y + l_q[8] * e_m.z;

    /* V_p = v * e_p */
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

    /* delta_dot calculation */
    double delta_dot = (-PARAM_K * f_sigma_not / t_go) *
                       ((PARAM_A * sig(PARAM_M, delta)) + (PARAM_B * sig(PARAM_N, delta)));

    /* omega_P = delta_dot * l_f */
    Vector3 omega_p;
    v3_scale(&omega_p, &l_f, delta_dot);
    Vector3 a_f;
    v3_cross(&a_f, &omega_p, &v_p);

    /* l_m = k_L x e_m, l_P = k_L x e_p */
    Vector3 l_m;
    v3_cross(&l_m, &k_l, &e_m);
    Vector3 l_p;
    v3_cross(&l_p, &k_l, &e_p);

    /* h_delta */
    double h_delta = (PARAM_A * sig(PARAM_M, delta)) + (PARAM_B * sig(PARAM_N, delta));

    /* Guidance acceleration components */
    double a_p_scalar = ((-N_GAIN * (v * v) * sin(sigma)) / r);
    Vector3 cross_l_f_e_p;
    v3_cross(&cross_l_f_e_p, &l_f, &e_p);
    double a_i_lm = (((N_GAIN - 1.0) * PARAM_K * f_sigma_not * v * h_delta) / t_go) *
                    v3_dot(&cross_l_f_e_p, &l_p);
    double a_i_kl = ((PARAM_K * f_sigma_not * v * sin(sigma) * h_delta) / (t_go * sin(eta))) *
                    v3_dot(&cross_l_f_e_p, &k_l);

    /* A_M_local_cal = (a_p + a_I_lm) * l_m + a_I_kl * k_L */
    Vector3 a_m_local_cal;
    Vector3 a_p_times_l_m;
    v3_scale(&a_p_times_l_m, &l_m, a_p_scalar + a_i_lm);
    Vector3 a_i_kl_times_k_l;
    v3_scale(&a_i_kl_times_k_l, &k_l, a_i_kl);
    v3_add(&a_m_local_cal, &a_p_times_l_m, &a_i_kl_times_k_l);

    /* ===== ATMOSPHERIC MODEL & MACH NUMBER ===== */
    double a_s = atmosphere(z);
    double mach = v / a_s;

    /* ===== LOCAL TO BODY TRANSFORMATION ===== */
    Vector3 a_m_body_temp;
    l2b(&a_m_body_temp, &a_m_local_cal, theta, psi, phi);

    /* ===== PITCH LIMITING ===== */
    Vector3 a_m_body_cal;
    a_m_body_cal.x = a_m_body_temp.x;
    a_m_body_cal.y = pitch_limit(mach, a_m_body_temp.y);
    a_m_body_cal.z = a_m_body_temp.z;

    /* ===== ACCELERATION RATE CLAMPING ===== */
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

    /* ===== FILL OUTPUT STRUCTURE ===== */
    output->t_go = t_go;
    output->r = r;
    output->theta_deg = RAD2DEG(theta);
    output->psi_deg = RAD2DEG(psi);
    output->A_M_body = a_m_body_final;
}
