# ISA Flight Software Requirements

## 1. System Overview

The ISA Flight Software is an embedded guidance system for a precision-guided projectile. The system follows a classic GNC (Guidance, Navigation, Control) architecture and runs on an ATMEL SAM V71 microcontroller.

### 1.1 Hardware Platform

- **MCU**: ATMEL SAM V71 (ARM Cortex-M7, 300MHz with FPU)
- **System Clock**: 300 MHz from 12 MHz external oscillator with PLL
- **Sensors**:
  - IMU (accelerometer, gyroscope, magnetometer) via I2C/SPI
  - GPS/GNSS receiver
  - Additional sensors via SDI interface
- **Interfaces**:
  - I2C/SPI for IMU
  - RS422 for various peripherals
  - GPIO for discrete I/O
  - UART for telemetry and debugging
- **Actuators**: Canards for flight control
- **Memory**: Flash memory and RAM with optimization for embedded deployment

### 1.2 Software Architecture

The system consists of four primary modules:

1. **Sequencer Module**: Manages mission phases and state transitions
2. **Navigation Module**: Processes sensor data and determines position/attitude
3. **Guidance Module**: Implements trajectory planning and guidance algorithms
4. **Digital Autopilot (DAP)**: Implements control laws and generates actuator commands

### 1.3 Timing Architecture

The software operates on a dual-cycle timing structure:

- **Minor Cycles**: 10 ms (100 Hz) for high-frequency, time-critical operations
- **Major Cycles**: 100 ms (10 Hz) for less time-critical operations
- Each major cycle consists of 10 minor cycles

## 2. Technical Requirements

### 2.1 Performance Requirements

- **Real-time Execution**: Deterministic timing with sub-millisecond response for critical functions
- **Navigation Update Rate**: 100Hz (10ms cycle)
- **GNSS Processing Rate**: 10Hz (100ms cycle)
- **Memory Optimization**: Minimized flash and RAM usage with stack overflow protection
- **FPU Utilization**: Optimized floating-point calculations using Cortex-M7 FPU

### 2.2 Compliance Requirements

- **MISRA C:2023**: Full compliance with all 221 guidelines
- **Static Analysis**: Integration with static code analysis tools
- **Safety Features**:
  - Watchdog timers
  - Stack protection
  - Deadlock prevention
  - Memory safety guarantees

## 3. Functional Requirements

### 3.1 Sequencer Module

- Manage the execution sequence from pre-launch to impact
- Handle state transitions based on timing and sensor inputs
- Execute the detailed flight sequence timeline as per specification
- Provide fault detection and recovery mechanisms
- Manage boot sequence and system initialization

#### 3.1.1 Sequencer State Machine

The sequencer shall implement a state machine that transitions through the following states:

1. **INIT**: Initial state after OBC reset (T0)
2. **T0_WINDOW**: Monitoring window after T0 to check roll rate < 7rps
3. **T1_REACHED**: Canard deployment state when roll rate criteria met
4. **T1_WINDOW**: Monitoring window after T1 to check roll rate < 2rps
5. **T2_REACHED**: Roll control activation state when roll rate criteria met
6. **PITCH_YAW_ON**: State after T2+Δt4 when pitch/yaw control is activated
7. **GUIDANCE_ON**: State after T2+2s when guidance is activated
8. **T2_WINDOW**: Monitoring window after guidance activation to check for GT3 flag
9. **T3_REACHED**: Proximity sensor activation state (3.5s before target)
10. **TERMINAL**: Final state during terminal approach to target

#### 3.1.2 Sequencer Inputs

| Parameter     | Elements | Data Type | Units | Source   |
| ------------- | -------- | --------- | ----- | -------- |
| Roll Rate     | 1        | double    | rps   | IMU      |
| GT3 Flag      | 1        | Bool      | -     | Guidance |
| System Time   | 1        | double    | s     | System   |
| System Status | 1        | uint32_t  | -     | System   |

#### 3.1.3 Sequencer Outputs

| Parameter         | Elements | Data Type | Units | Destination |
| ----------------- | -------- | --------- | ----- | ----------- |
| Navigation Enable | 1        | Bool      | -     | Navigation  |
| Roll Control Flag | 1        | Bool      | -     | DAP         |
| Pitch/Yaw Flag    | 1        | Bool      | -     | DAP         |
| Guidance Flag     | 1        | Bool      | -     | Guidance    |
| Proximity Flag    | 1        | Bool      | -     | Sensors     |
| Mission Time      | 1        | double    | s     | All Modules |
| Sequencer State   | 1        | enum      | -     | System      |

#### 3.1.4 Timing Requirements

- The sequencer shall operate at the minor cycle frequency (100Hz)
- State transitions shall be evaluated every minor cycle
- Sensor conditions shall be confirmed over 3 consecutive minor cycles before state transitions
- Timing windows shall be enforced with ±1ms precision

#### 3.1.5 Mission Timeline Requirements

| Event ID | Event Description   | Trigger Condition                          | Action                              |
| -------- | ------------------- | ------------------------------------------ | ----------------------------------- |
| T0       | OBC Reset           | System initialization                      | Begin roll rate monitoring          |
| T1       | Canard Deployment   | Roll rate < 7rps for 3 consecutive cycles  | Activate navigation, deploy canards |
| T2       | Roll Control        | Roll rate < 2rps for 3 consecutive cycles  | Activate roll control               |
| T2+Δt4   | Pitch/Yaw Control   | T2 + Δt4 seconds elapsed                   | Activate pitch/yaw control          |
| T2+2s    | Guidance Activation | T2 + 2 seconds elapsed                     | Activate guidance module            |
| T3       | Proximity Sensor    | GT3 flag received for 3 consecutive cycles | Activate proximity sensor           |

#### 3.1.6 Fault Handling Requirements

- The sequencer shall implement timeout detection for each state transition
- If a state transition condition is not met within the specified window, a fault shall be raised
- The sequencer shall provide mechanisms to handle sensor data anomalies
- Critical state transitions shall have fallback mechanisms if primary conditions are not met

### 3.2 Navigation Module

- Process IMU data (accelerometer, gyroscope, magnetometer)
- Process GNSS data for position and velocity
- Perform sensor fusion and filtering
- Compute position, velocity, attitude, and body rates
- Implement coordinate transformations between reference frames (Body, ECEF, ECI)
- Provide data to Guidance and DAP modules

#### 3.2.1 Navigation Inputs

| Parameter                        | Elements | Data Type | Units | Source |
| -------------------------------- | -------- | --------- | ----- | ------ |
| Linear Acceleration (Body Frame) | 3        | double    | m/s²  | IMU    |
| Velocity Increment (Body Frame)  | 3        | double    | m/s   | IMU    |
| Incremental Angles (Body Frame)  | 3        | double    | rad   | IMU    |
| Angular Rates                    | 3        | double    | rad/s | IMU    |
| Position (ECEF Frame)            | 3        | double    | m     | GNSS   |
| Velocity (ECEF Frame)            | 3        | double    | m/s   | GNSS   |

#### 3.2.2 Navigation Outputs

| Parameter   | Elements | Data Type | Units | Destination |
| ----------- | -------- | --------- | ----- | ----------- |
| Position    | 3        | double    | m     | Guidance    |
| Velocity    | 3        | double    | m/s   | Guidance    |
| Quaternions | 4        | double    | -     | DAP         |
| Body Rates  | 3        | double    | rad/s | DAP         |

### 3.3 Guidance Module

- Receive position and velocity from Navigation
- Implement trajectory planning and optimization
- Execute the guidance algorithm (proportional navigation with impact angle control)
- Generate commanded angles for the projectile
- Activate at T2+2 seconds and deactivate 100m above target

#### 3.3.1 Guidance Algorithm

- Implement proportional navigation guidance law with impact angle control
- Handle coordinate transformations between ECI, ECEF, and local frames
- Calculate acceleration commands based on current state and target position
- Account for drag and gravitational effects
- Switch between guidance phases (initial, mid-course, terminal)

### 3.4 Digital Autopilot (DAP) Module

- Receive attitude data from Navigation
- Receive commanded angles from Guidance
- Implement control laws for roll, pitch, and yaw
- Generate actuator commands for the canards
- Ensure stability during flight
- Handle roll control activation at T2
- Handle pitch/yaw control activation at T2+Δt4

### 3.5 Cycle Allocation by Module

The software executes all functions in a dual-cycle model. Tasks listed below are assigned to minor (10 ms) and major (100 ms) cycles with an explicit execution order to guarantee determinism and bounded latency.

#### 3.5.1 Minor Cycle Tasks (10 ms, 100 Hz)

Execution order per cycle:

1. Navigation (Minor)
2. Guidance (Minor)
3. Sequencer
4. DAP (Control/Actuation)
5. Telemetry (Minor frame)

| Order | Module     | Responsibilities                                                                             | Primary Inputs                                                 | Outputs                                                   | Rate   |
| ----- | ---------- | -------------------------------------------------------------------------------------------- | -------------------------------------------------------------- | --------------------------------------------------------- | ------ |
| 1     | Navigation | Gyro/accel processing; quaternion propagation; body rates; magnetometer roll (as applicable) | IMU incremental angles/velocities, angular rates, magnetometer | Attitude quaternion, body rates, specific accel           | 100 Hz |
| 2     | Guidance   | Command update/streaming using last major-cycle solution; FOV/safety checks                  | NAV state (ECI/Body), last major-cycle guidance state          | Commanded angles/accelerations to DAP                     | 100 Hz |
| 3     | Sequencer  | State evaluation and flag updates; timing window checks; mission timeline gating             | System time, IMU roll rate, GT3 flag, system status            | Control flags (roll/pitch-yaw/guidance enable, proximity) | 100 Hz |
| 4     | DAP        | Control laws (roll/pitch/yaw); actuator command compute; saturations/slew/dead-zone handling | NAV quaternion/body rates; Guidance commands; flags            | Canard PWM commands (±6° range)                           | 100 Hz |
| 5     | Telemetry  | Package and send minor telemetry subset; watchdog kick                                       | Module status/counters                                         | RS-422 minor frame                                        | 100 Hz |

Notes:

- IMU data are serviced in interrupt or DMA context and consumed in this minor cycle.
- Stack and timing monitors run in the minor cycle to enforce bounds and log peaks.

#### 3.5.2 Major Cycle Tasks (100 ms, 10 Hz)

| Order | Module       | Responsibilities                                                                               | Primary Inputs                       | Outputs                                    | Rate  |
| ----- | ------------ | ---------------------------------------------------------------------------------------------- | ------------------------------------ | ------------------------------------------ | ----- |
| 1     | Navigation   | GNSS ECEF position/velocity processing; ECEF→ECI transform (GMST); gravity model; state fusion | GNSS P/V in ECEF; minor-cycle state  | ECI position/velocity; altitude; Mach      | 10 Hz |
| 2     | Guidance     | 3-D guidance solve with impact-angle constraint; path/command profile update                   | NAV ECI P/V; target; sequencer flags | Updated guidance state and command profile | 10 Hz |
| 3     | Telemetry    | Package and send major telemetry frame (e.g., 1024 bytes)                                      | System and module data               | RS-422 major frame                         | 10 Hz |
| 4     | Housekeeping | Fault management; health summaries; memory/timing budget snapshots                             | Module health/timers                 | Logs and status                            | 10 Hz |

Cycle execution order within a frame:

- Minor cycle order: Navigation (Minor) → Guidance (Minor) → Sequencer → DAP → Telemetry.
- Major cycle order: Navigation (Major) → Guidance (Major) → Telemetry → Housekeeping.

## 4. Mission Profile

The projectile follows a detailed sequence from pre-launch to impact:

1. Pre-launch operations (system checks, initialization)
2. Fire sensing (T0) - Roll estimation begins
3. OBC reset and thermal battery power-on
4. FSA (Fire Sensing Assembly) activation
5. Canard deployment at T1
6. Roll control activation at T2
7. Pitch/Yaw control activation at T2+Δt4
8. Guidance activation at T2+2 seconds
9. Proximity sensor activation 3.5s before target
10. Guidance deactivation 100m above target

## 5. Development Standards

### 5.1 Coding Standards

- Follow MISRA C:2023 guidelines
- Use consistent naming conventions and code structure
- Implement comprehensive error handling
- Provide detailed documentation for all functions and modules
- Follow static analysis recommendations

### 5.2 Testing Requirements

- Unit testing for all modules and functions
- Integration testing for module interactions
- System testing with hardware-in-the-loop
- Performance benchmarking and optimization
- MISRA compliance verification

## 6. Deliverables

- MISRA C:2023 compliant source code
- Detailed documentation package
- Test reports and compliance certificates
- Build scripts and deployment instructions
- Maintenance and troubleshooting guides

## 7. Development and Integration Workplan

1. Module Development (in isolation)

   - Sequencer state machine with unit tests for transitions and timing windows.
   - Navigation: IMU pipeline (100 Hz), GNSS pipeline (10 Hz), frame transforms, quaternion math.
   - Guidance: 3-D PNG with impact-angle control and FOV handling; target and constraint interfaces.
   - DAP: Control loops (roll/pitch/yaw), actuator models (saturation/slew/dead-zone), gain sets.

2. Interface Contracts

   - Define headers and data structures for inter-module IO (inputs/outputs listed in Sections 3.2–3.4 and 3.5).
   - Freeze message/telemetry schemas (minor/major frames) and HAL driver APIs.

3. Minor-Cycle Integration (100 Hz)

   - Integrate Navigation (Minor) → Guidance (Minor) → Sequencer → DAP → Telemetry ordering.
   - Establish timing/stack monitors and watchdog servicing.

4. Major-Cycle Integration (10 Hz)

   - Integrate Navigation (Major) GNSS path and Guidance (Major) solver; add Telemetry and Housekeeping.
   - Validate ECEF↔ECI transforms (GMST) and gravity model in closed-loop simulations.

5. Timing and Memory Budgets

   - Verify worst-case execution time per task fits 10 ms minor / 100 ms major deadlines with margin.
   - Enforce fixed memory usage; enable stack guards and bounds checks.

6. Hardware Abstraction and Drivers

   - RS-422, I2C/SPI, GPIO, PWM drivers aligned with cycle consumption points.
   - Interrupt/DMA data acquisition for IMU and GNSS; ring buffers for cycle handoff.

7. Integration Testing

   - Module unit tests → subsystem tests → full software-in-the-loop → hardware-in-the-loop.
   - Telemetry/command loopback and actuator bench tests.

8. Handoff to Hardware Team

   - Consolidate dual-cycle build with all modules enabled; generate binaries and flashing artifacts.
   - Provide configuration, versioning, and test vectors for microcontroller trials.

9. On-Target Testing and Iteration
   - Bench, captive-carry, and range testing per mission profile; collect RS-422 logs.
   - Iterate gains and parameters within defined interfaces; maintain compliance and timing.
