/******************************************************************************
* ISA Flight Software
 * @file guidance.c
 * @brief Projectile Flight Computer Guidance Module Implementation
* @details Implements proportional navigation with impact angle control
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 4.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#include "guidance.h"
#include "coordinate_transforms.h"
#include "math_utils.h"
#include <stdio.h>
#include <math.h>
#include <float.h>

/* Helper: geodetic to ECEF (same as geodeticToECI in MATLAB) */
static void geodetic_to_ecef_helper(Vector3* r_ecef, double lat_deg, double lon_deg, double alt_m)
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

void onboard_guidance_algorithm(double lat_o, double lon_o, double alt_o,
    double lat_t, double lon_t, double alt_t,
    double x_0_ecef, double y_0_ecef, double z_0_ecef,
    double vx_0_ecef, double vy_0_ecef, double vz_0_ecef,
    double theta_f, double psi_f,
    FILE* csv_file)
{
    /* Target states in ECEF frame (assumed stationary) */
    Vector3 r_t_0_ecef;
    geodetic_to_ecef_helper(&r_t_0_ecef, lat_t, lon_t, alt_t);

    /* Initial projectile states at apogee (guidance start) */
    Vector3 r_m_ecef = { x_0_ecef, y_0_ecef, z_0_ecef };
    Vector3 v_m_ecef = { vx_0_ecef, vy_0_ecef, vz_0_ecef };

    /* Time elapsed from GUID_START */
    double t = 0.0;

    /* Conversion of states of projectile from ECEF to LOCAL frame */
    Vector3 r_m_local;
    ecef_to_local(&r_m_local, &r_m_ecef, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);
    Vector3 r_t_local;
    ecef_to_local(&r_t_local, &r_t_0_ecef, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

    Vector3 r_diff;
    v3_sub(&r_diff, &r_t_local, &r_m_local);
    double r = v3_norm(&r_diff);
    double r_prev = r;

    /* Desired impact direction unit vector */
    Vector3 e_f;
    e_f.x = cos(theta_f) * cos(psi_f);
    e_f.y = cos(theta_f) * sin(psi_f);
    e_f.z = sin(theta_f);

    /* Initial conditions */
    printf("GUIDANCE PHASE STARTED (APOGEE)\n");
    printf("Initial projectile ECEF position: (%.2f, %.2f, %.2f) m\n", r_m_ecef.x, r_m_ecef.y, r_m_ecef.z);
    printf("Initial projectile ECEF velocity: (%.2f, %.2f, %.2f) m/s\n", v_m_ecef.x, v_m_ecef.y, v_m_ecef.z);
    printf("Target Distance: %.2f m\n\n", r);

    /* Write CSV header if file is provided */
    if (csv_file != NULL)
    {
        fprintf(csv_file, "Time,"
            "Guidance_Command_Local_X,Guidance_Command_Local_Y,Guidance_Command_Local_Z,"
            "Guidance_Command_Body_X,Guidance_Command_Body_Y,Guidance_Command_Body_Z,"
            "Guidance_Command_ECEF_X,Guidance_Command_ECEF_Y,Guidance_Command_ECEF_Z,"
            "Drag_ECEF_X,Drag_ECEF_Y,Drag_ECEF_Z,"
            "Gravity_ECEF_X,Gravity_ECEF_Y,Gravity_ECEF_Z,"
            "Projectile_ECEF_Pos_X,Projectile_ECEF_Pos_Y,Projectile_ECEF_Pos_Z,"
            "Projectile_ECEF_Vel_X,Projectile_ECEF_Vel_Y,Projectile_ECEF_Vel_Z,"
            "Projectile_Local_Pos_X,Projectile_Local_Pos_Y,Projectile_Local_Pos_Z,"
            "Projectile_Local_Vel_X,Projectile_Local_Vel_Y,Projectile_Local_Vel_Z,"
            "Theta_deg,Psi_deg,"
            "Time_To_Go,Distance_To_Target,"
            "GT3_Flag\n");
    }

    /* GT3 FLAG GENERATION */
    int gt3_check_active = 0;
    double gt3_previous = DBL_MAX;
    int consecutive_decreases = 0;
    int gt3_printed = 0;

    /* Loop condition for the guidance phase */
    while (t < MAX_SIM_TIME && r <= r_prev && v3_norm(&v_m_ecef) > MIN_VEL)
    {
        /* Define position and velocity vectors of the projectile */
        ecef_to_local(&r_m_local, &r_m_ecef, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);
        Vector3 v_m_local;
        ecef_to_local_vel(&v_m_local, &v_m_ecef, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

        /* Current states in local frame */
        double x = r_m_local.x;
        double y = r_m_local.y;
        double z = r_m_local.z;
        double vx = v_m_local.x;
        double vy = v_m_local.y;
        double vz = v_m_local.z;

        /* Conversion of states of target from ECEF to LOCAL (Guidance) frame */
        ecef_to_local(&r_t_local, &r_t_0_ecef, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

        /* Vector to target FROM projectile */
        Vector3 r_vec;
        r_vec.x = r_t_local.x - r_m_local.x;
        r_vec.y = r_t_local.y - r_m_local.y;
        r_vec.z = (r_t_local.z + R_LOOP3_START) - r_m_local.z;

        /* Magnitude of line-of-sight vector */
        r = v3_norm(&r_vec);

        /* Unit vector OF line-of-sight vector */
        Vector3 e_r;
        v3_normed(&e_r, &r_vec);

        /* Unit vector of velocity vector */
        double v = v3_norm(&v_m_local);
        Vector3 e_m;
        v3_normed(&e_m, &v_m_local);

        /* current theta and psi */
        double theta = asin(vz / v);
        double psi = atan2(vy, vx);
        double phi = 0.0;

        /* Lead angle */
        double sigma = acos(v3_dot(&e_m, &e_r));

        /* Time to go (calculated every iteration, matching MATLAB) */
        double t_go = r / (v * cos(sigma));

        Vector3 a_m_local;

        /* Check if we should switch to terminal phase */
        if (r <= R_LOOP3_START)
        {
            /* Terminal phase - ballistic flight */
            a_m_local.x = 0.0;
            a_m_local.y = 0.0;
            a_m_local.z = 0.0;
            printf("Entered terminal phase at t=%.2f s, distance=%.2f m\n", t, r);
        }
        else
        {
            /* Guided phase - calculate acceleration command in local frame */

            /* Calculate guidance commands */
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

            /* Add eps value to avoid division by zero */
            Vector3 cross_e_r_e_m;
            v3_cross(&cross_e_r_e_m, &e_r, &e_m);
            double cross_norm = v3_norm(&cross_e_r_e_m);
            Vector3 k_l;
            v3_scale(&k_l, &cross_e_r_e_m, 1.0 / (cross_norm + EPS));

            double eta = sigma / (N_GAIN - 1.0);
            double mu = sigma + eta;

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

            /* Impact angle error */
            double delta = acos(v3_dot(&e_f, &e_p));
            Vector3 cross_e_f_e_p;
            v3_cross(&cross_e_f_e_p, &e_f, &e_p);
            double cross_norm_lf = v3_norm(&cross_e_f_e_p);
            Vector3 l_f;
            v3_scale(&l_f, &cross_e_f_e_p, 1.0 / (cross_norm_lf + EPS));

            /* t_go already calculated above, use it here */
            double sigma_not = sigma / SIGMA_MAX_RAD;

            /* d > 0 */
            double f_sigma_not = 1.0 - pow(fabs(sigma_not), PARAM_D);

            /* a > 0, b > 0, 0 < m < 1, n > 1 */
            /* Calculate delta_dot with expression inline (matching MATLAB line 152) */
            double delta_dot = (-PARAM_K * f_sigma_not / t_go) * ((PARAM_A * sig(PARAM_M, delta)) + (PARAM_B * sig(PARAM_N, delta)));

            /* angular velocity vector of V_P */
            Vector3 omega_p;
            v3_scale(&omega_p, &l_f, delta_dot);
            Vector3 a_f;
            v3_cross(&a_f, &omega_p, &v_p);

            Vector3 l_m;
            v3_cross(&l_m, &k_l, &e_m);
            Vector3 l_p;
            v3_cross(&l_p, &k_l, &e_p);

            /* Calculate h_delta separately (matching MATLAB line 161) */
            double h_delta = (PARAM_A * sig(PARAM_M, delta)) + (PARAM_B * sig(PARAM_N, delta));

            /* components of guidance acceleration command vector */
            double a_p_scalar = ((-N_GAIN * (v * v) * sin(sigma)) / r);
            Vector3 cross_l_f_e_p;
            v3_cross(&cross_l_f_e_p, &l_f, &e_p);
            double a_i_lm = (((N_GAIN - 1.0) * PARAM_K * f_sigma_not * v * h_delta) / t_go) * v3_dot(&cross_l_f_e_p, &l_p);
            double a_i_kl = ((PARAM_K * f_sigma_not * v * sin(sigma) * h_delta) / (t_go * sin(eta))) * v3_dot(&cross_l_f_e_p, &k_l);

            /* Guidance acceleration command vector */
            Vector3 a_p_times_l_m;
            v3_scale(&a_p_times_l_m, &l_m, a_p_scalar + a_i_lm);
            Vector3 a_i_kl_times_k_l;
            v3_scale(&a_i_kl_times_k_l, &k_l, a_i_kl);
            v3_add(&a_m_local, &a_p_times_l_m, &a_i_kl_times_k_l);
        }

        /* Convert guidance acceleration to ECEF frame */
        Vector3 a_m_ecef;
        local_to_ecef_vel(&a_m_ecef, &a_m_local, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

        /* Calculate drag in ECEF frame */
        double v_ecef = v3_norm(&v_m_ecef);
        double q_dyn = 0.5 * RHO * v_ecef * v_ecef;
        double d = (q_dyn * C_D * S_REF);
        double dx = d * (v_m_ecef.x) / v_ecef;
        double dy = d * (v_m_ecef.y) / v_ecef;
        double dz = d * (v_m_ecef.z) / v_ecef;

        /* Drag */
        Vector3 a_d_ecef;
        a_d_ecef.x = dx / M_C;
        a_d_ecef.y = dy / M_C;
        a_d_ecef.z = dz / M_C;

        /* Gravity */
        Vector3 grav_local = { 0.0, 0.0, -G_CONST };
        Vector3 grav_ecef;
        local_to_ecef_vel(&grav_ecef, &grav_local, lat_o, lon_o, alt_o, lat_t, lon_t, alt_t);

        /* Total acceleration in ECEF frame */
        Vector3 a_total_ecef;
        Vector3 neg_a_d_ecef;
        v3_scale(&neg_a_d_ecef, &a_d_ecef, -1.0);
        Vector3 temp;
        v3_add(&temp, &a_m_ecef, &neg_a_d_ecef);
        v3_add(&a_total_ecef, &temp, &grav_ecef);

        /* Integrate acceleration to get velocity in ECEF */
        Vector3 v_increment;
        v3_scale(&v_increment, &a_total_ecef, DT);
        v3_add(&v_m_ecef, &v_m_ecef, &v_increment);

        /* Integrate velocity to get position in ECEF */
        Vector3 r_increment;
        v3_scale(&r_increment, &v_m_ecef, DT);
        v3_add(&r_m_ecef, &r_m_ecef, &r_increment);

        /* Calculate body frame command for display (matching MATLAB line 173) */
        Vector3 a_m_body;
        l2b(&a_m_body, &a_m_local, theta, psi, phi);

        /* Display current states */
        printf("Time: %.2f s\n", t);
        printf("GUIDANCE COMMAND LOCAL: (%.2f, %.2f, %.2f) m/s^2\n", a_m_local.x, a_m_local.y, a_m_local.z);
        printf("GUIDANCE COMMAND BODY: (%.2f, %.2f, %.2f) m/s^2\n", a_m_body.x, a_m_body.y, a_m_body.z);
        printf("GUIDANCE COMMAND ECEF: (%.2f, %.2f, %.2f) m/s^2\n", a_m_ecef.x, a_m_ecef.y, a_m_ecef.z);
        printf("DRAG IN ECEF: (%.2f, %.2f, %.2f) m/s^2\n", a_d_ecef.x, a_d_ecef.y, a_d_ecef.z);
        printf("GRAVITY IN ECEF: (%.2f, %.2f, %.2f) m/s^2\n", grav_ecef.x, grav_ecef.y, grav_ecef.z);
        printf("Projectile ECI position: (%.2f, %.2f, %.2f) m\n", r_m_ecef.x, r_m_ecef.y, r_m_ecef.z);
        printf("Projectile ECI velocity: (%.2f, %.2f, %.2f) m/s\n", v_m_ecef.x, v_m_ecef.y, v_m_ecef.z);
        printf("Projectile LOCAL position: (%.2f, %.2f, %.2f) m\n", x, y, z);
        printf("Projectile LOCAL velocity: (%.2f, %.2f, %.2f) m/s\n", vx, vy, vz);
        printf("Theta: %.2f deg, Psi: %.2f deg\n", RAD2DEG(theta), RAD2DEG(psi));
        printf("Time to go : %.2f s\n\n", t_go);
        printf("Distance to target: %.2f m\n\n", r);

        /* GT3 GENERATION */
        int gt3_flag_this_iter = 0;
        if (!gt3_printed)
        {
            if (!gt3_check_active && t_go <= GT3_THRESHOLD)
            {
                gt3_check_active = 1;
                gt3_previous = t_go;
                consecutive_decreases = 1;
            }
            else if (gt3_check_active)
            {
                if (t_go < gt3_previous)
                {
                    consecutive_decreases++;
                    gt3_previous = t_go;

                    if (consecutive_decreases >= GT3_CONSECUTIVE)
                    {
                        printf("FLAG: GT3 \n");
                        gt3_flag_this_iter = 1;
                        gt3_printed = 1;
                        gt3_check_active = 0;
                        consecutive_decreases = 0;
                    }
                }
                else
                {
                    consecutive_decreases = 0;
                    gt3_previous = t_go;
                }
            }
        }

        /* Write data to CSV file if provided */
        if (csv_file != NULL)
        {
            fprintf(csv_file, "%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,"
                "%.6f,%.6f,"
                "%d\n",
                t,
                a_m_local.x, a_m_local.y, a_m_local.z,
                a_m_body.x, a_m_body.y, a_m_body.z,
                a_m_ecef.x, a_m_ecef.y, a_m_ecef.z,
                a_d_ecef.x, a_d_ecef.y, a_d_ecef.z,
                grav_ecef.x, grav_ecef.y, grav_ecef.z,
                r_m_ecef.x, r_m_ecef.y, r_m_ecef.z,
                v_m_ecef.x, v_m_ecef.y, v_m_ecef.z,
                x, y, z,
                vx, vy, vz,
                RAD2DEG(theta), RAD2DEG(psi),
                t_go, r,
                gt3_flag_this_iter);
        }

        /* Check if we've passed the target (distance starts increasing) */
        if (r > r_prev)
        {
            printf("Target impact at t=%.2f s\n", t);
            printf("Final ECEF position: (%.2f, %.2f, %.2f) m\n", r_m_ecef.x, r_m_ecef.y, r_m_ecef.z);
            printf("Final distance to target: %.2f m\n", r_prev);
            break;
        }

        /* Update for next iteration */
        r_prev = r;
        t = t + DT;
    }

    /* Check why we exited the loop */
    if (t >= MAX_SIM_TIME)
    {
        printf("Simulation time exceeded.\n");
    }
    else if (v3_norm(&v_m_ecef) <= MIN_VEL)
    {
        printf("Projectile velocity too low.\n");
    }
    else if (r_m_local.z < 0)
    {
        printf("Projectile altitude became negative.\n");
    }
}

