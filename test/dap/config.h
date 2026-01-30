/******************************************************************************
 * ISA Flight Software
 *
 * File: config.h
 * Description: Configuration parameters for the guidance system
 *****************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

#include "types.h"


static const DAPParameters_t DAP_DEFAULT = {
    /* Roll controller constants */
    .kp_roll = 11.8899,            /* Proportional gain for roll */
    .ki_roll = 2.0,                /* Integrator gain for roll */
    .kr_roll = 0.0841,             /* Rate gain for roll */
    .integratorR_min_rad = -0.1745, /* Integrator min limit in radians */
    .integratorR_max_rad = 0.1745,  /* Integrator max limit in radians */
    .phi_min_rad = -0.014677,       /* Roll angle min limit in radians */
    .phi_max_rad = 0.014677,        /* Roll angle max limit in radians */
    /* Pitch controller constants */
    .kp_pitch = 5.94495,            /* Proportional gain for pitch */
    .ki_pitch = 4.0,                /* Integrator gain for pitch */
    .kr_pitch = 0.0841,             /* Rate gain for pitch */
    .K_LPF_Pitch=0.0555,           /* Low-pass filter gain for pitch */
    .wC_pitch=0.8,                  /* Cut-off frequency for pitch */
    .pitch_a=1.5,                   /* Lag filter a constant */
    .pitch_b=0.03,                   /* Lag filter b constant */
    .theta_min_rad = -0.014677,       /* Pitch angle min limit in radians */
    .theta_max_rad = 0.014677,        /* Pitch angle max limit in radians */
    .integratorP_min_rad = -0.08726, /* Integrator min limit in radians */
    .integratorP_max_rad = 0.08726,  /* Integrator max limit in radians */
    /* Yaw controller constants */
    .kp_yaw = 11.8899,
    .ki_yaw = 2.0,
    .kr_yaw = 0.0841,             /* Rate gain for yaw */
    .K_LPF_Yaw=-0.05,           /* Low-pass filter gain for yaw */
    .wC_yaw=0.2,                  /* Cut-off frequency for yaw */
    .yaw_a=0.1,                   /* Lag filter a constant */
    .yaw_b=0.01,                   /* Lag filter b constant */     
    .psi_min_rad = -0.014677,       /* Yaw angle min limit in radians */
    .psi_max_rad = 0.014677,        /* Yaw angle max limit in radians */
    .integratorY_min_rad = -0.08726, /* Integrator min limit in radians */
    .integratorY_max_rad = 0.08726,  /* Integrator max limit in radians */
};

#endif /* CONFIG_H */

