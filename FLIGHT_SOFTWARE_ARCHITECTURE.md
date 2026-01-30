# Flight Software Overview

This document provides a comprehensive overview of the Integrated Flight Software architecture, covering its workflow from start to finish, system inputs, control flags, and a detailed technical explanation of the core modules.

## 1. System Workflow: Start to Finish

The flight software operates on a multi-rate architecture with two primary loops:
*   **Minor Cycle (100Hz)**: Handles fast control loops (Navigation, Sequencer, DAP).
*   **Major Cycle (10Hz)**: Handles complex trajectory calculations (GNSS, Guidance).

### Phase 1: Initialization (OBC Reset)
*   **Trigger**: Power-on or Soft Reset.
*   **Action**: 
    1.  `set_obc_reset(true)` initializes the system.
    2.  `load_pefcs_defaults()` loads mission-specific parameters (Magnetic field reference, initial attitude, sensor offsets) from `pefcs_init.h`.
    3.  All state variables, counters, and flags are zeroed.
    4.  **T0 Set**: The sequencer immediately sets the `isT0Set` flag to mark the start of system initialization.

### Phase 2: Active Flight Loops

#### A. Minor Cycle (Every 10ms)
1.  **Sensor Data Acquisition & Processing**:
    *   **Inputs**: Reads raw LSB values from **Accelerometer**, **Gyroscope**, and **Magnetometer**.
    *   **Conversion**: Converts LSBs to `double` precision physical units.
    *   **Health Check**: Validates sensor data. If a sensor fails, the system holds the last valid value (Fail-Safe), preventing erroneous control spikes.
    *   **Calibration**: Applies Hard Iron/Soft Iron corrections (Mag) and Zero Offsets (Accel/Gyro).
2.  **Navigation (Attitude Estimation)**:
    *   **Roll**: Estimated using a trigonometric solution (`atan2`) combining Magnetometer data (`Mag_Z`, `Mag_Y`) with the known Earth magnetic field vectors (`M_North`, `M_Down`).
    *   **Pitch/Yaw**: High-frequency integration. Body angular rates (`p, q, r`) are transformed via Euler Kinematics to update Pitch (`theta`) and Yaw (`psi`) angles.
    *   **Output**: Current Attitude (Roll, Pitch, Yaw) in Body Frame.
3.  **Sequencer (Flight Phase Management)**:
    *   Monitors `Roll Rate` and `Time-to-Go` to determine the flight phase (Boost, Mid-course, Terminal).
    *   Sets global **Flags** to trigger vehicle actions (Deploy Canards, Start Guidance, Arm Fuze).
4.  **Digital Autopilot (DAP)**:
    *   **Input**: Guidance commands (from Major Cycle) + Current Navigation State.
    *   **Logic**: Computes fin deflections to stabilize the projectile (Roll) and track acceleration commands (Pitch/Yaw).
    *   **Output**: `ActuatorCommands` for the Fin Servo Actuators (FSA).

#### B. Major Cycle (Every 100ms)
1.  **GNSS Processing**:
    *   **Input**: Reads ECEF Position & Velocity from GNSS Receiver.
    *   **Fail-Safe**: If Signal Lost (Unlock), predicts position using Dead Reckoning based on the last known velocity.
2.  **Guidance Algorithm**:
    *   **Input**: Current Position/Velocity (Local Frame) + Origin Target and Mission Target
    *   **Logic**: Uses **Impact Angle Control (IAC)** mixed with **Proportional Navigation (PN)**.
        *   Calculates `Time-to-Go (t_go)` to impact.
        *   Generates a lateral acceleration command (`accelCmdBody`) to nullify the miss distance while satisfying impact angle constraints.
    *   **Output**: `accelCmdBody` sent to DAP.

---

## 2. System Inputs

The software ingests data from three primary hardware sources:

### A. Inertial Measurement Unit (IMU)
*   **Accelerometer**: Raw X, Y, Z (LSB) -> Converted to `m/s²`.
*   **Gyroscope**: Raw X, Y, Z (LSB) -> Converted to `rad/s`.
*   **Magnetometer**: Raw X, Y, Z (LSB) -> Converted to `mG` (Milli-Gauss).

### B. GNSS Receiver
*   **Position**: ECEF X, Y, Z (Meters).
*   **Velocity**: ECEF VX, VY, VZ (m/s).
*   **Status**: Lock / No-Lock flag.

### C. Pre-Flight Configuration (PEFCS)
*   Loaded from `pefcs_init.h`:
    *   **Initial Attitude**: Initial Roll, Pitch, Yaw.
    *   **Sensor Calibration**: Hard/Soft Iron offsets, Gyro/Accel biases.
    *   **Magnetic Reference**: Expected Earth magnetic field vector at launch site.

---

## 3. System Flags

The **Sequencer** manages the mission state using the following flags (defined in `SystemState_t`):

| Flag Name | Set Condition | Purpose |
| :--- | :--- | :--- |
| `isT0Set` | **OBC Reset**: Initially set to true | Marks T0 (System Init / Launch). |
| `isT1Set` |  Roll Rate < 7.0 rps | Marks T1 (End of boost phase). |
| `isT2Set` |  Roll Rate < 2.0 rps | Marks T2 (Start of controlled flight). |
| `isT3Set` | **Terminal**: `tGo` < 3.5s | Marks T3 (Terminal guidance phase). |
| `fsaActivateFlag` | **Timer (T2 Phase)** | Flag to Fuze Systems (triggered at 2rps event). |
| `canardDeployFlag` | Timer (based on T1) | Unlocks/Deploys canard fins. |
| `canardControlFlag` | Timer (based on T1) | Enables DAP control loop logic. |
| `guidStartFlag` | Timer (based on T1) | Enables Major Cycle Guidance calculations. |
| `proximitySensorFlag` | `tGo` < Threshold | Arms proximity fuze. |

---

## 4. Module Deep Dives

### A. Digital Autopilot (DAP) Logic
The DAP runs at **100Hz** and is responsible for stability and command tracking.

1.  **Roll Control (Stabilization)**
    *   **Goal**: Maintain 0 rad/s roll rate (de-spin) or hold a specific roll angle.
    *   **Logic**: Feedback Controller (Rate Damping + Integral).
    *   **Formula**:
        ```c
        Error = Command - Roll_Angle
        Rate_Term = Kr * Roll_Rate
        Integral_Term += (Error - Rate_Term) * dt
        Command = (Kp * (Error - Rate_Term)) + (Ki * Integral_Term)
        ```
    *   **Actuation**: Deflects all 4 fins symmetrically (`delta3, delta6, delta9, delta12`).

2.  **Pitch & Yaw Control (Acceleration Tracking)**
    *   **Goal**: Track the Acceleration Command (`accelCmdBody`) calculated by Guidance.
    *   **Logic**: Acceleration-Feedback Controller.
    *   **Formula**:
        ```c
        // 1. Calculate Rate Derivative (Angular Acceleration)
        Rate_Dot = (Rate_Now - Rate_Prev) / dt

        // 2. Compensation Term (Lever Arm Effect)
        Accel_Comp = Measured_Accel + (Lever_Arm * Rate_Dot)

        // 3. Acceleration Error Calculation
        // Subtracts the "Rate Damping" (Kr*Rate) and "Acceleration Feedback" (Ka*Accel)
        Accel_Error = Cmd_Accel - (Kr * Rate) - (Ka * Accel_Comp)

        // 4. Final Steering Command
        Command = Accel_Error * Ks
        ```
    *   **Actuation**: Deflects fins differentially (`delta1, delta2`) for Pitch/Yaw moments.

### B. Guidance Logic (IAC + PN)
The Guidance algorithm runs at **10Hz** and generates the acceleration commands.

*   **Logic**:
    1.  Computes `Range` and `Line-of-Sight (LOS)` unit vector to target.
    2.  Calculates `Time-to-Go (t_go)` based on Range and Closing Velocity.
    3.  **Proportional Navigation (PN)**: Calculates acceleration proportional to the LOS rotation rate (`omega`). Ideally keeps the LOS angle constant (collision course).
    4.  **Impact Angle Control (IAC)**: Adds an acceleration bias to ensure the projectile hits the target at specific Azimuth/Elevation angles (`psi_f`, `theta_f`).
    5.  **Pitch Limiter**: Limits the output acceleration based on Mach number (Pitch Limit Curve) to prevent aerodynamic stall or structural failure.
    6.  **Rate Limiter**: Clamps the rate of change of acceleration (`GUID_MAX_DELTA_ACC`) to ensure smooth fin movements.
