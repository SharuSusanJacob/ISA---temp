/******************************************************************************
* ISA Flight Software
* @file major.h
* @brief Major Cycle (10Hz) Interface
* @details Defines the interface for the 100ms cycle operations
* @author Integration Team, Spacelabs
* @date 2025
* @version 1.0.4
*****************************************************************************/

#ifndef MAJOR_H
#define MAJOR_H

#include <stdint.h>
#include <stdbool.h>
#include "math_utils.h" /* Include for Vector3_t definition */

// Guidance constants (matching test module)
#define GUID_NAVIGATION_GAIN_N 3.0
#define GUID_PARAM_A 0.100001
#define GUID_PARAM_B 0.100001
#define GUID_PARAM_M 0.90001
#define GUID_PARAM_N 1.30001
#define GUID_PARAM_D 7.5
#define GUID_PARAM_K 21.00000001
#define GUID_MAX_LOOK_ANGLE_DEG 73.0
#define GUID_MAX_LOOK_ANGLE_RAD (GUID_MAX_LOOK_ANGLE_DEG * MATH_DEG_TO_RAD)
#define GUID_TERMINAL_DISTANCE_M 100.0
#define GUID_MIN_VELOCITY_M_S 1.0
#define GUID_EPSILON 0.0001
#define GUID_MAX_DELTA_ACC 1.0 /* Maximum acceleration change per timestep (m/s²) */

// Guidance state structure for output
typedef struct
{
    Vector3_t accelCmdBody; /* Commanded acceleration in Body frame (m/s²) */
    double timeToGo;        /* Estimated time to impact (s) */
    bool terminalPhase;     /* True if in terminal phase */
} GuidanceState_t;

/* ===== HARDWARE SENSOR INPUT VARIABLES (Placeholder for Hardware Team) ===== */

/**
 * @brief GNSS ECEF position data from hardware
 *
 * This structure contains the raw ECEF position data from GNSS hardware.
 * The hardware team will populate these values from the actual GNSS receiver.
 */
typedef struct
{
    double x_m;         /* ECEF X position in meters */
    double y_m;         /* ECEF Y position in meters */
    double z_m;         /* ECEF Z position in meters */
    bool valid;         /* GNSS data validity flag */
    uint32_t timestamp; /* GNSS data timestamp */
} GnssEcefPosition_t;

/**
 * @brief GNSS ECEF velocity data from hardware
 *
 * This structure contains the raw ECEF velocity data from GNSS hardware.
 * The hardware team will populate these values from the actual GNSS receiver.
 */
typedef struct
{
    double vx_ms;       /* ECEF X velocity in m/s */
    double vy_ms;       /* ECEF Y velocity in m/s */
    double vz_ms;       /* ECEF Z velocity in m/s */
    bool valid;         /* GNSS velocity validity flag */
    uint32_t timestamp; /* GNSS velocity timestamp */
} GnssEcefVelocity_t;

/**
 * @brief Mission target and launch point data
 *
 * These structures contain the mission-specific target and launch point
 * coordinates that will be provided by the mission planning system.
 */
typedef struct
{
    double lat_deg; /* Target latitude in degrees */
    double lon_deg; /* Target longitude in degrees */
    double alt_m;   /* Target altitude in meters */
} MissionTarget_t;

typedef struct
{
    double lat_deg; /* Launch point latitude in degrees */
    double lon_deg; /* Launch point longitude in degrees */
    double alt_m;   /* Launch point altitude in meters */
} MissionLaunchPoint_t;

/**
 * @brief Major cycle function - Called every 100ms (10Hz)
 *
 * This is the main entry point for the major cycle processing.
 * It executes guidance algorithms at 10Hz.
 *
 * @return void
 */
void major_cycle(void);

/**
 * @brief Initialize guidance module with PEFCS parameters
 *
 * Reads geodetic coordinates and impact angles from PEFCS,
 * converts them to ECEF, and initializes guidance state.
 *
 * @return void
 */
void guidance_init(void);

/* ===== Hardware Interface Functions ===== */

/**
 * @brief Set GNSS ECEF position data from hardware buffer
 *
 * This function is called by the hardware team to provide GNSS position data.
 * Converts LSB values (signed int32, big-endian) from buffer to double (meters).
 *
 * @param buffer Pointer to uint8_t buffer containing GNSS data
 * @param offset_x Byte offset for X position LSB (4 bytes, big-endian)
 * @param offset_y Byte offset for Y position LSB (4 bytes, big-endian)
 * @param offset_z Byte offset for Z position LSB (4 bytes, big-endian)
 * @param timestamp GNSS timestamp
 */
void set_gnss_position_ecef(const uint8_t *buffer, uint16_t offset_x, uint16_t offset_y, uint16_t offset_z, bool valid, uint32_t timestamp);

/**
 * @brief Set GNSS ECEF velocity data from hardware buffer
 *
 * This function is called by the hardware team to provide GNSS velocity data.
 * Converts LSB values (signed int32, big-endian) from buffer to double (m/s).
 *
 * @param buffer Pointer to uint8_t buffer containing GNSS data
 * @param offset_vx Byte offset for X velocity LSB (4 bytes, big-endian)
 * @param offset_vy Byte offset for Y velocity LSB (4 bytes, big-endian)
 * @param offset_vz Byte offset for Z velocity LSB (4 bytes, big-endian)
 * @param timestamp GNSS timestamp
 */
void set_gnss_velocity_ecef(const uint8_t *buffer, uint16_t offset_vx, uint16_t offset_vy, uint16_t offset_vz, bool valid, uint32_t timestamp);

/**
 * @brief Set mission target coordinates
 *
 * This function sets the target coordinates for the mission in ECEF frame.
 *
 * @param x_m Target X coordinate in ECEF frame (meters)
 * @param y_m Target Y coordinate in ECEF frame (meters)
 * @param z_m Target Z coordinate in ECEF frame (meters)
 */
void set_mission_target(double x_m, double y_m, double z_m);

/**
 * @brief Set mission launch point coordinates
 *
 * This function sets the launch point coordinates for the mission in ECEF frame.
 *
 * @param x_m Launch point X coordinate in ECEF frame (meters)
 * @param y_m Launch point Y coordinate in ECEF frame (meters)
 * @param z_m Launch point Z coordinate in ECEF frame (meters)
 */
void set_mission_launch_point(double x_m, double y_m, double z_m);

/**
 * @brief Set guidance impact angles
 *
 * This function sets the desired impact angles for guidance.
 *
 * @param theta_f_rad Desired impact elevation angle in radians
 * @param psi_f_rad Desired impact azimuth angle in radians
 */
void set_guidance_impact_angles(double theta_f_rad, double psi_f_rad);

/**
 * @brief Process GNSS data with fail-safe handling
 *
 * This function processes GNSS data from hardware buffer with fail-safe
 * mechanism. If GNSS lock is lost, previous valid data is used.
 *
 * @return void
 */
void process_gnss_data(void);

/**
 * @brief Process GNSS ECEF data and convert to local frame
 *
 * This function processes the GNSS ECEF position and velocity data,
 * converts it to the local frame, and stores it in system state.
 *
 * @return void
 */
void process_gnss_ecef_data(void);

/**
 * @brief Calculate guidance acceleration command
 *
 * This function implements the guidance algorithm (PN + IAC) and outputs
 * acceleration command in body frame.
 *
 * @param accel_body Output acceleration command in body frame (m/s²)
 * @param position_ecef Input projectile position in ECEF frame (m)
 * @param velocity_ecef Input projectile velocity in ECEF frame (m/s)
 * @param origin_ecef Launch point in ECEF frame (m)
 * @param target_ecef Target point in ECEF frame (m)
 * @param theta_f Desired impact elevation angle (rad)
 * @param psi_f Desired impact azimuth angle (rad)
 * @param theta Pitch angle in radians (from navigation)
 * @param psi Yaw angle in radians (from navigation)
 * @param phi Roll angle in radians (from navigation)
 */
void calculate_guidance_acceleration(Vector3_t *accel_body,
                                     const Vector3_t *position_ecef, const Vector3_t *velocity_ecef,
                                     const Vector3_t *origin_ecef, const Vector3_t *target_ecef,
                                     double theta_f, double psi_f, double theta, double psi, double phi);

#endif /* MAJOR_H */