/******************************************************************************
 * ISA Flight Software
 *
 * File: dap.h
 * Description: Digital Autopilot module interface
 *****************************************************************************/

#ifndef DAP_H
#define DAP_H

#include "types.h"
#include "vector3.h"
#include <stdbool.h>

 /**
  * @brief DAP module interface for flight control
  */

  /**
   * @brief Execute the Digital Autopilot control algorithm
   *
   * @param rollAngle_rad Current roll angle in radians
   * @param accelerationY_mps2 Current Y-axis acceleration in m/s^2
   * @param accelerationZ_mps2 Current Z-axis acceleration in m/s^2
   * @param rollRate_radps Current roll rate in rad/s
   * @param pitchRate_radps Current pitch rate in rad/s
   * @param yawRate_radps Current yaw rate in rad/s
   * @param accelerationYCommand_mps2 Commanded Y-axis acceleration in m/s^2
   * @param accelerationZCommand_mps2 Commanded Z-axis acceleration in m/s^2
   * @param dapParams DAP controller parameters
   * @param timeStep_s Time step in seconds
   * @param canardControlFlag Flag indicating if canard control is enabled
   * @return DAPOutput_t Control commands for roll, pitch, and yaw
   */

DAPOutput_t DAP_Execute(
    double rollAngle_rad,
    double accelerationY_mps2,
    double accelerationZ_mps2,
    double rollRate_radps,
    double pitchRate_radps,
    double yawRate_radps,
    double accelerationYCommand_mps2,
    double accelerationZCommand_mps2,
    DAPParameters_t dapParams,
    double timeStep_s,
    bool canardControlFlag
);

/**
 * @brief Compute roll control command
 *
 * @param rollAngle_rad Current roll angle in radians
 * @param rollRate_radps Current roll rate in rad/s
 * @param integratorValue Current integrator value
 * @param dapParams DAP controller parameters
 * @param timeStep_s Time step in seconds
 * @return DAPOutput_t Roll control command
 */
DAPOutput_t ComputeDeltaRollCommand(
    double rollAngle_rad,
    double rollRate_radps,
    DAPParameters_t dapParams,
    double timeStep_s
);

/**
 * @brief Compute pitch control command
 *
 * @param accelerationY_mps2 Current Y-axis acceleration in m/s^2
 * @param pitchRate_radps Current pitch rate in rad/s
 * @param accelerationYCommand_mps2 Commanded Y-axis acceleration in m/s^2
 * @param dapParams DAP controller parameters
 * @param timeStep_s Time step in seconds
 * @return PYOutput_t Pitch control command
 */
PYOutput_t ComputeDeltaPitchCommand(
    double accelerationY_mps2,
    double pitchRate_radps,
    double accelerationYCommand_mps2,
    DAPParameters_t dapParams,
    double timeStep_s
);
PYOutput_t ComputeDeltaPitchCommand_Kadam(
    double accelerationY_mps2,
    double pitchRate_radps,
    double accelerationYCommand_mps2,
    DAPParameters_t dapParams,
    double timeStep_s
);

/**
 * @brief Compute yaw control command
 *
 * @param accelerationZ_mps2 Current Z-axis acceleration in m/s^2
 * @param yawRate_radps Current yaw rate in rad/s
 * @param accelerationZCommand_mps2 Commanded Z-axis acceleration in m/s^2
 * @param dapParams DAP controller parameters
 * @param timeStep_s Time step in seconds
 * @return PYOutput_t Yaw control command
 */
PYOutput_t ComputeDeltaYawCommand(
    double accelerationZ_mps2,
    double yawRate_radps,
    double accelerationZCommand_mps2,
    DAPParameters_t dapParams,
    double timeStep_s
);

PYOutput_t ComputeDeltaYawCommand_Kadam(
    double accelerationZ_mps2,
    double yawRate_radps,
    double accelerationZCommand_mps2,
    DAPParameters_t dapParams,
    double timeStep_s
);

#endif /* DAP_H */

