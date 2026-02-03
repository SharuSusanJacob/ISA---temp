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
#define PEFCS_TARGET_LAT_DEG 8.67176
#define PEFCS_TARGET_LON_DEG 76.8854
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
#define PEFCS_IMPACT_THETA_F_DEG -65.0
#define PEFCS_IMPACT_PSI_F_DEG 0.0

/**
 * @brief GPS-UTC leap seconds offset
 * 
 * Number of leap seconds between GPS time and UTC time.
 * Used for ECEF→ECI coordinate transformations.
 * Valid as of 2017-01-01 (update if leap second is added)
 */
#define PEFCS_GPS_LEAP_SECONDS 18

/* ===== DAP (DIGITAL AUTOPILOT) INITIALIZATION PARAMETERS ===== */

/* Roll Controller Parameters */
#define PEFCS_DAP_KP_ROLL 11.8899
#define PEFCS_DAP_KI_ROLL 2.0
#define PEFCS_DAP_KR_ROLL 0.0841
#define PEFCS_DAP_INTEGRATOR_R_MIN -0.1745
#define PEFCS_DAP_INTEGRATOR_R_MAX 0.1745
#define PEFCS_DAP_PHI_MIN -0.014677
#define PEFCS_DAP_PHI_MAX 0.014677

/* Pitch Controller Parameters */
#define PEFCS_DAP_KP_PITCH 5.94495
#define PEFCS_DAP_KI_PITCH 4.0
#define PEFCS_DAP_KR_PITCH 0.0841
#define PEFCS_DAP_K_LPF_PITCH 0.055556
#define PEFCS_DAP_WC_PITCH 0.8
#define PEFCS_DAP_PITCH_A 1.5
#define PEFCS_DAP_PITCH_B 0.03
#define PEFCS_DAP_THETA_MIN -0.014677
#define PEFCS_DAP_THETA_MAX 0.014677
#define PEFCS_DAP_INTEGRATOR_P_MIN -0.08726
#define PEFCS_DAP_INTEGRATOR_P_MAX 0.08726

/* Yaw Controller Parameters */
#define PEFCS_DAP_KP_YAW 11.8899
#define PEFCS_DAP_KI_YAW 2.0
#define PEFCS_DAP_KR_YAW 0.0841
#define PEFCS_DAP_K_LPF_YAW -0.05
#define PEFCS_DAP_WC_YAW 0.2
#define PEFCS_DAP_YAW_A 0.1
#define PEFCS_DAP_YAW_B 0.01
#define PEFCS_DAP_PSI_MIN -0.014677
#define PEFCS_DAP_PSI_MAX 0.014677
#define PEFCS_DAP_INTEGRATOR_Y_MIN -0.08726
#define PEFCS_DAP_INTEGRATOR_Y_MAX 0.08726

/* Actuator Deadband and Limit Parameters */
#define PEFCS_ACTUATOR_DEADBAND_RAD 0.00081806 /* 0.05 degrees in radians */
#define PEFCS_ACTUATOR_MAX_DEFLECTION_RAD 0.104712 /* 6 degrees in radians */


/* ===== SEQUENCER INITIALIZATION PARAMETERS ===== */
#define PEFCS_SEQ_CONFIRMATION_CYCLES 3U
#define PEFCS_SEQ_ROLL_RATE_T1_THRESHOLD 7.0
#define PEFCS_SEQ_ROLL_RATE_T2_THRESHOLD 2.0

/* Sequencer Timing Configuration (in cycles) */
#define PEFCS_SEQ_T1_WINDOW_IN_TIME 10U
#define PEFCS_SEQ_T1_WINDOW_OUT_TIME 1400U
#define PEFCS_SEQ_T2_WINDOW_IN_TIME 10U
#define PEFCS_SEQ_T2_WINDOW_OUT_TIME 1000U
#define PEFCS_SEQ_T3_WINDOW_IN_TIME 10U
#define PEFCS_SEQ_T3_WINDOW_OUT_TIME 4800U /* Updated to 48s limit */

/* Sequencer Timing Configuration (in seconds) */
#define PEFCS_SEQ_T_PROXIMITY 3.5

/* Sequencer Flag Delays (in cycles) */
#define PEFCS_SEQ_CANARD_DEPLOY_FLAG_DELAY 0U
#define PEFCS_SEQ_CANARD_CONTROL_ON_FLAG_DELAY 0U
#define PEFCS_SEQ_FSA_FLAG_DELAY 0U
#define PEFCS_SEQ_GUID_START_FLAG_DELAY 0U

#endif /* PEFCS_INIT_H */
