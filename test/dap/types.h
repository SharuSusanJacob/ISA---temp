/******************************************************************************
 * ISA Flight Software
 *
 * File: types.h
 * Description: Common data type definitions for the guidance system
 *****************************************************************************/

#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>

 /**
  * @brief Mathematical constants
  */
#define PI                  3.14159265358979323846f
#define DEG_TO_RAD          (PI / 180.0f)
#define RAD_TO_DEG          (180.0f / PI)

  /**
   * @brief DAP parameters structure
   */
typedef struct {
    /* Roll controller constants */
    double kp_roll;                  /* Proportional gain for roll */
    double ki_roll;                  /* Integrator gain for roll */
    double kr_roll;                  /* Rate gain for roll */
    double integratorR_min_rad;      /* Integrator min limit in radians */
    double integratorR_max_rad;      /* Integrator max limit in radians */
    double phi_min_rad;             /* Roll angle min limit in radians */
    double phi_max_rad;             /* Roll angle max limit in radians */
    /* Pitch controller constants */
    double kp_pitch;                /* Proportional gain for pitch */
    double ki_pitch;                /* Integrator gain for pitch */
    double kr_pitch;               /* Rate gain for pitch */
    double integratorP_min_rad;      /* Integrator min limit in radians */
    double integratorP_max_rad;      /* Integrator max limit in radians */
    double K_LPF_Pitch;             /* Low-pass filter gain for pitch */
    double wC_pitch;                /* Cut-off frequency for pitch */
    double pitch_a;                 /* Lag filter a constant */
    double pitch_b;                 /* Lag filter b constant */    
    double theta_min_rad;           /* Pitch angle min limit in radians */
    double theta_max_rad;           /* Pitch angle max limit in radians */
    /* Yaw controller constants */
    double kp_yaw;                  /* Proportional gain for yaw */
    double ki_yaw;                  /* Integrator gain for yaw */
    double kr_yaw;                 /* Rate gain for yaw */
    double integratorY_min_rad;      /* Integrator min limit in radians */
    double integratorY_max_rad;      /* Integrator max limit in radians */
    double K_LPF_Yaw;               /* Low-pass filter gain for yaw */
    double wC_yaw;                  /* Cut-off frequency for yaw */
    double yaw_a;                   /* Lag filter a constant */
    double yaw_b;                   /* Lag filter b constant */
    double psi_min_rad;           /* Yaw angle min limit in radians */
    double psi_max_rad;           /* Yaw angle max limit in radians */
} DAPParameters_t;

/**
 * @brief DAP output structure for canard deflection commands
 */
typedef struct {
    double delta3_rad;  /* Canard 3 deflection angle in radians */
    double delta6_rad;  /* Canard 6 deflection angle in radians */
    double delta9_rad;  /* Canard 9 deflection angle in radians */
    double delta12_rad; /* Canard 12 deflection angle in radians */
} DAPOutput_t;

/**
 * @brief Pitch/Yaw output structure for control commands
 */
typedef struct {
    double delta1_rad;  /* Canard 1 deflection angle in radians */
    double delta2_rad;  /* Canard 2 deflection angle in radians */
} PYOutput_t;


#endif /* TYPES_H */

