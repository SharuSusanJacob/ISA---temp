/******************************************************************************
* ISA Flight Software
* @file pefcs_init.h
* @brief Pre-Flight Electronic Checkout System (PEFCS) Initialization Parameters
* @details Default mission-specific parameters for all flight modules
*          These values are loaded on OBC reset and can be updated by PEFCS
* @author Integration Team, Spacelabs
* @date 2025
* @version 1.0
*****************************************************************************/

#ifndef PEFCS_INIT_H
#define PEFCS_INIT_H

#include "math_utils.h"

/* ===== NAVIGATION INITIALIZATION PARAMETERS ===== */

/**
 * @brief Initial attitude angles from PEFCS (degrees)
 * 
 * These are converted to radians during initialization
 */
#define PEFCS_INITIAL_AZIMUTH_DEG 0.0    /* Initial Yaw angle (degrees) */
#define PEFCS_INITIAL_ELEVATION_DEG 25.0 /* Initial Pitch angle (degrees) */

/**
 * @brief Magnetic field reference in NED frame (milliGauss)
 * 
 * These values define the local magnetic field vector for roll estimation
 * Should be measured/calibrated at launch site during checkout
 */
#define PEFCS_MAG_REF_NORTH_MG 407.18455450466178 /* North component (mG) */
#define PEFCS_MAG_REF_EAST_MG -12.541435991209887 /* East component (mG) */
#define PEFCS_MAG_REF_DOWN_MG 28.342885818553878  /* Down component (mG) */

/**
 * @brief Magnetometer hard iron calibration offsets (milliGauss)
 * 
 * Hard iron correction: measured_field = raw_field - hard_iron_offset
 */
#define PEFCS_MAG_HARD_IRON_X 0.0
#define PEFCS_MAG_HARD_IRON_Y 0.0
#define PEFCS_MAG_HARD_IRON_Z 0.0

/**
 * @brief Magnetometer soft iron calibration matrix (3x3)
 * 
 * Soft iron correction: calibrated_field = soft_iron_matrix * offset_corrected_field
 * Default is identity matrix (no correction)
 */
#define PEFCS_MAG_SOFT_IRON_XX 1.0
#define PEFCS_MAG_SOFT_IRON_XY 0.0
#define PEFCS_MAG_SOFT_IRON_XZ 0.0
#define PEFCS_MAG_SOFT_IRON_YX 0.0
#define PEFCS_MAG_SOFT_IRON_YY 1.0
#define PEFCS_MAG_SOFT_IRON_YZ 0.0
#define PEFCS_MAG_SOFT_IRON_ZX 0.0
#define PEFCS_MAG_SOFT_IRON_ZY 0.0
#define PEFCS_MAG_SOFT_IRON_ZZ 1.0

/**
 * @brief Gyroscope bias offsets (rad/s)
 * 
 * Measured during pre-flight calibration
 */
#define PEFCS_GYRO_OFFSET_X 0.0
#define PEFCS_GYRO_OFFSET_Y 0.0
#define PEFCS_GYRO_OFFSET_Z 0.0

/**
 * @brief Accelerometer bias offsets (m/s²)
 * 
 * Measured during pre-flight calibration
 */
#define PEFCS_ACCEL_OFFSET_X 0.0
#define PEFCS_ACCEL_OFFSET_Y 0.0
#define PEFCS_ACCEL_OFFSET_Z 0.0

/* ===== GUIDANCE INITIALIZATION PARAMETERS ===== */

/**
 * @brief Target position in Geodetic coordinates
 * 
 * Target for 20 km range mission (Sea)
 * Reference: test/guidance_updated_clamp/guidance_test.c
 */
#define PEFCS_TARGET_LAT_DEG 8.50853578598926
#define PEFCS_TARGET_LON_DEG 76.70458281218454
#define PEFCS_TARGET_ALT_M 0.0

/**
 * @brief Launch origin in Geodetic coordinates
 * 
 * Reference point for local coordinate transformations
 * Reference: test/guidance_updated_clamp/guidance_test.c
 */
#define PEFCS_ORIGIN_LAT_DEG 8.529175797045042
#define PEFCS_ORIGIN_LON_DEG 76.88543289537785
#define PEFCS_ORIGIN_ALT_M 0.0

/**
 * @brief Desired final impact angles (radians)
 * 
 * theta_f: Impact pitch/elevation angle (-73.0 degrees)
 * psi_f: Impact azimuth angle (0.0 degrees)
 * Reference: test/guidance_updated_clamp/guidance_test.c
 */
#define PEFCS_IMPACT_THETA_F_DEG -73.0
#define PEFCS_IMPACT_PSI_F_DEG 0.0

/* ===== DAP (DIGITAL AUTOPILOT) INITIALIZATION PARAMETERS ===== */

/* Roll Controller Parameters */
#define PEFCS_DAP_KP_ROLL 11.8899
#define PEFCS_DAP_KI_ROLL 2.0
#define PEFCS_DAP_KR_ROLL 0.0841

/* Pitch Controller Parameters */
#define PEFCS_DAP_KP_PITCH 5.94495
#define PEFCS_DAP_KI_PITCH 4.0
#define PEFCS_DAP_KR_PITCH 0.0841
#define PEFCS_DAP_K_LPF_PITCH 0.0555
#define PEFCS_DAP_WC_PITCH 0.8
#define PEFCS_DAP_PITCH_A 1.5
#define PEFCS_DAP_PITCH_B 0.03

/* Yaw Controller Parameters */
#define PEFCS_DAP_KP_YAW 11.8899
#define PEFCS_DAP_KI_YAW 2.0
#define PEFCS_DAP_KR_YAW 0.0841
#define PEFCS_DAP_K_LPF_YAW -0.05
#define PEFCS_DAP_WC_YAW 0.2
#define PEFCS_DAP_YAW_A 0.1
#define PEFCS_DAP_YAW_B 0.01

/* ===== SEQUENCER INITIALIZATION PARAMETERS ===== */

/**
 * @brief Roll rate thresholds already defined in minor.h
 * 
 * SEQ_ROLL_RATE_T1_THRESHOLD: 7.0 rps
 * SEQ_ROLL_RATE_T2_THRESHOLD: 2.0 rps
 * These are system constants, not PEFCS-configurable
 */

#endif /* PEFCS_INIT_H */
