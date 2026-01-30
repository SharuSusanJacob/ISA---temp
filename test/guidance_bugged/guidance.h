/******************************************************************************
* ISA Flight Software
* @file guidance.h
* @brief Projectile Flight Computer Guidance Module
* @details Implements proportional navigation with impact angle control
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 3.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>  // for memset
#include "vector3.h"
#include "coordinate_transforms.h"

// Erro codes (following MISRA C patterns)

typedef enum {
    GUID_SUCCESS = 0U,
    GUID_ERROR_INVALID_PARAM = 1U,
    GUID_ERROR_INVALID_STATE = 2U,
    GUID_ERROR_MATH_ERROR = 3U,
    GUID_ERROR_CONVERGENCE = 4U
} GuidanceError_t;


// Physical constants (embedded fixed precision)

#define GUID_GRAVITY_M_S2           9.8         // Earth gravity (m/s²)
#define GUID_AIR_DENSITY_KG_M3      1.225      // Air density (kg/m³)
#define GUID_PROJECTILE_MASS_KG     47.0        // Projectile mass (kg)
#define GUID_DRAG_COEFFICIENT       0.2808      // Drag coefficient
#define GUID_PROJECTILE_DIAMETER_M  0.155       // Diameter (m)
#define GUID_REFERENCE_AREA_M2      (3.14159 * GUID_PROJECTILE_DIAMETER_M * GUID_PROJECTILE_DIAMETER_M / 4.0)


// Guuidance parameters 

#define GUID_NAVIGATION_GAIN_N      3.0         // Proportional navigation gain
#define GUID_PARAM_A                0.100001    // Impact angle control parameter
#define GUID_PARAM_B                0.100001    // Impact angle control parameter
#define GUID_PARAM_M                0.90001     // Impact angle control parameter (0 < m < 1)
#define GUID_PARAM_N                1.30001     // Impact angle control parameter (n > 1.4)
#define GUID_PARAM_D                7.5         // Look angle weighting parameter
#define GUID_PARAM_K                21.00000001 // Impact angle control gain
#define GUID_MAX_LOOK_ANGLE_DEG     73.0         // Maximum look angle in degrees
#define GUID_MAX_LOOK_ANGLE_RAD     (GUID_MAX_LOOK_ANGLE_DEG * MATH_DEG_TO_RAD)


// Simualtion parameters , may be have to remove this for the flight software
#define GUID_TERMINAL_DISTANCE_M    100.0       // Switch to terminal phase distance
#define GUID_MIN_VELOCITY_M_S       1.0         // Minimum projectile velocity threshold m/s
#define GUID_INTEGRATION_STEP_S     0.001       // Integration time step (matching MATLAB)
#define GUID_MAX_SIMULATION_TIME_S  200.0      // Maximum simulation time


// GT3 flag parameters (matching MATLAB)
#define GUID_GT3_THRESHOLD           3.5        // Initial t_go value threshold
#define GUID_GT3_CONSECUTIVE_COUNT   3           // Required consecutive decreases

// Epsilon for division protection (matching MATLAB)
#define GUID_EPSILON                 0.0001      // Epsilon for cross product division (matching MATLAB eps)

// Guidance state structure

typedef struct {
    // Input states from navigation - ECEF frame
    Vector3_t position_ecef;        // Current position in ECEF frame (m)
    Vector3_t velocity_ecef;        // Current velocity in ECEF frame (m/s)

    //Target and launch information
    GeodeticPos_t launch;          // Launch position (lat/lon/alt)
    GeodeticPos_t target;          // Target position (lat/lon/alt)

    // Desired impact angles
    double theta_f_rad;             // Desired impact elevation angle (rad)
    double psi_f_rad;               // Desired impact azimuth angle (rad)

    // Internal state
    double time_s;                  // Current guidance time (s)
    bool terminal_phase;           // True if in terminal phase
    bool guidance_active;          // True if guidance is active

    // Output Commands - Multiple frames
    Vector3_t acceleration_cmd_ecef; // Total commanded acceleration in ECEF frame (m/s²)
    Vector3_t acceleration_cmd_local; // Commanded acceleration in Local frame (m/s²)
    Vector3_t acceleration_cmd_body; // Commanded acceleration in Body frame (m/s²) - Input to DAP
    Vector3_t pure_guidance_ecef; // Pure guidance acceleration in ECEF frame (before drag/gravity) (m/s²)
    Vector3_t drag_acceleration_ecef;  // Drag acceleration in ECEF frame (m/s²)
    Vector3_t gravity_acceleration_ecef; // Gravity acceleration in ECEF frame (m/s²)
    double time_to_go_s;             // Estimated time to impact (s)

    // GT3 flag generation
    bool gt3_check_active;         // Flag to indicate we're monitoring t_go
    bool gt3_printed;              // Flag to prevent repeated GT3 messages
    double gt3_previous;            // Store previous t_go value
    uint8_t consecutive_decreases;  // Count consecutive decreases in t_go

    // Internal working variables
    double distance_to_target_m;     // Current distance to target (m)
    uint32_t integration_cycles;    // Number of integration cycles
    Vector3_t position_local;       // Current position in Local frame (m)
    Vector3_t velocity_local;       // Current velocity in Local frame (m/s)

} GuidanceState_t;

// Initialize guidance module (called once at startup)
GuidanceError_t guidance_init(GuidanceState_t* state);

// Main guidance update function (called every MAJOR cycle - 100ms, 10Hz)
// this is the priomary guidance algorithm execution
GuidanceError_t guidance_execute(GuidanceState_t* state, double dt_s);

// Set launch position (called during mission setup)
GuidanceError_t guidance_set_launch(GuidanceState_t* state, const GeodeticPos_t* launch);

// Set target position (called when target is known)
GuidanceError_t guidance_set_target(GuidanceState_t* state, const GeodeticPos_t* target);

// Set Impact angles (called during mission setup)
GuidanceError_t guidance_set_impact_angles(GuidanceState_t* state, double theta_f_rad, double psi_f_rad); //

#endif /* GUIDANCE_H */
