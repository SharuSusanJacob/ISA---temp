# ISA Flight Software - Module Integration Checklist

**Project:** ISA Flight Software  
**Author:** Integration Team, Spacelabs  
**Date:** November 11, 2025  
**Version:** 1.0  

---

## Overview

This document tracks the integration status of all flight software modules from standalone test implementations into the unified `src/minor.c` and `src/major.c` integrated system.

**Integration Architecture:**
- **Test Modules:** Individual standalone modules in `test/` directory
- **Integrated System:** Unified implementation in `src/` directory
- **Shared State:** Global `SystemState_t` in `system_state.h`
- **Timing:** Minor cycle (100Hz), Major cycle (10Hz)

---

## Module Integration Status

### ✅ 1. Sequencer Module

**Status:** COMPLETE ✅  
**Test Module:** `test/sequencer/sequencer.c`  
**Integrated Module:** `src/minor.c` (lines 771-1177)  
**Integration Date:** November 11, 2025  

#### Changes Made
- [x] Type alignment: Changed `tGo` from `uint32_t` (cycles) to `float` (seconds)
- [x] Threshold update: `SEQ_T_PROXIMITY` changed from `100U` cycles to `1.0f` seconds
- [x] Integrated with `SystemState_t` via `systemState.sequencerState`
- [x] Outputs mapped to `systemState.flags.*` for public access
- [x] Reads `rollRateFp` from `systemState.rollRateFp` (calculated in gyro processing)
- [x] Reads `timeToGo` from `systemState.guidanceState.timeToGo` (float, seconds)
- [x] Static helper functions: `is_roll_rate_ok_for_t1()`, `is_roll_rate_ok_for_t2()`
- [x] Phase logic: `process_t1_logic()`, `process_t2_logic()`, `process_t3_logic()`
- [x] Main execution: `execute_sequencer()` called in minor cycle

#### Interface
**Inputs:**
- `systemState.rollRateFp` (float, rps) - from gyroscope processing
- `systemState.guidanceState.timeToGo` (float, seconds) - from guidance (major cycle)
- `systemState.sequencerState.isOBCReset` (bool) - OBC reset status

**Outputs:**
- `systemState.flags.isT0Set`, `isT1Set`, `isT2Set`, `isT3Set` (phase flags)
- `systemState.flags.fsaActivateFlag` (FSA activation)
- `systemState.flags.canardDeployFlag` (canard deployment)
- `systemState.flags.canardControlFlag` (canard control enable)
- `systemState.flags.guidStartFlag` (guidance start)
- `systemState.flags.proximitySensorFlag` (proximity sensor enable)

#### Testing Status
- [x] Unit tests available in `test/sequencer/`
- [x] Type consistency verified between test and integrated modules
- [ ] Integration testing with full system
- [ ] Hardware-in-the-loop (HIL) testing

#### Known Issues
- None currently identified

---

### 🔄 2. Navigation Module

**Status:** IN PROGRESS 🔄  
**Test Module:** `test/navigation/navigation.c`  
**Integrated Module:** `src/minor.c` (lines 592-769)  
**Integration Date:** TBD  

#### Integration Tasks
- [ ] Verify sensor data processing consistency
- [ ] Check attitude estimation algorithms
- [ ] Validate magnetometer calibration constants
- [ ] Verify gyroscope integration logic
- [ ] Confirm 3-cycle confirmation logic
- [ ] Check NED to body frame transformations
- [ ] Validate roll/pitch/yaw angle calculations

#### Interface
**Inputs:**
- Accelerometer data (3-axis, m/s²)
- Gyroscope data (3-axis, rad/s)
- Magnetometer data (3-axis, mG)
- Incremental angle data (3-axis, rad)
- Sensor health status

**Outputs:**
- `systemState.navigationState.attitude_e` (Euler angles)
- `systemState.attitudeTelemetryData.attitude_rad` (roll, pitch, yaw)
- `systemState.navigationState.mag_3_cycles_confirmed`
- `systemState.navigationState.gyro_3_cycles_confirmed`

#### Testing Status
- [ ] Unit tests reviewed
- [ ] Type consistency verification
- [ ] Integration testing
- [ ] HIL testing

#### Known Issues
- Needs verification of magnetometer attitude estimation vs gyro integration priority
- Hard iron and soft iron calibration constants need validation

---

### ✅ 3. DAP (Digital Autopilot) Module

**Status:** COMPLETE ✅  
**Test Module:** `test/dap/dap.c`  
**Integrated Module:** `src/minor.c` (lines 1179-1405)  
**Integration Date:** November 11, 2025  

#### Changes Made
- [x] Fixed redundant canard control flag check in integrated module
- [x] Removed unreachable else block in integrated module
- [x] Added critical division-by-zero protection to test module
- [x] Verified algorithm correctness (all three axes match)
- [x] Confirmed parameter passing (pointer method is more efficient)
- [x] Verified integrator anti-windup logic
- [x] Checked rate derivative calculations with safety guards

#### Integration Tasks
- [x] Verify roll control algorithm implementation - **IDENTICAL**
- [x] Check pitch control algorithm implementation - **IDENTICAL (with safety improvement)**
- [x] Validate yaw control algorithm implementation - **IDENTICAL (with safety improvement)**
- [x] Confirm integrator anti-windup logic - **CORRECT**
- [x] Verify rate derivative calculations - **CORRECT (with safety guards added)**
- [x] Check canard deflection mapping - **CORRECT**
- [x] Validate control command limits - **CORRECT**

#### Interface
**Inputs:**
- `systemState.guidanceState.accelCmdBodyY` (pitch command, m/s²)
- `systemState.guidanceState.accelCmdBodyZ` (yaw command, m/s²)
- `systemState.accelerometerData.accel_current` (Y and Z axes)
- `systemState.angularRates` (roll, pitch, yaw rates)
- `systemState.navigationState.attitude_e.roll_rad` (roll angle)
- `systemState.dapParams` (controller gains and limits)
- `systemState.flags.canardControlFlag` (enable/disable)

**Outputs:**
- `systemState.actuatorCommands.x` (delta3_rad - Canard 3)
- `systemState.actuatorCommands.y` (delta6_rad - Canard 6)
- `systemState.actuatorCommands.z` (delta9_rad - Canard 9)
- Roll integrator state (internal)
- Previous pitch/yaw rates (internal)

#### Testing Status
- [x] Algorithm correctness verified
- [x] Type consistency verified
- [x] Safety checks added
- [ ] Integration testing with full system
- [ ] HIL testing

#### Key Improvements Made
1. **Critical Safety Fix:** Added division-by-zero protection in test module
   - Prevents NaN/Inf from zero or very small timesteps
   - Uses `safe_timeStep` with minimum value of 1e-6f
2. **Code Quality:** Removed redundant check and unreachable code in integrated module
   - Cleaner control flow
   - More maintainable
3. **Parameter Efficiency:** Integrated module uses pointer passing (more efficient than test module)

#### Known Issues
- ⚠️ **Actuator command structure limitation:** `delta12_rad` (4th canard) is not stored
  - Current `actuatorCommands` is Vector3_t (only 3 components)
  - Need to extend structure or add separate field for 4-canard system
  - Marked with TODO comment in code
- ✅ **Guidance/DAP timing mismatch:** Guidance updates at 10Hz, DAP at 100Hz - **ACCEPTABLE**
  - DAP uses same command for 10 cycles - verified as intentional design

---

### ⏳ 4. Guidance Module

**Status:** PENDING ⏳  
**Test Module:** `test/guidance/guidance.c`  
**Integrated Module:** `src/major.c`  
**Integration Date:** TBD  

#### Integration Tasks
- [ ] Review guidance algorithm implementation
- [ ] Verify trajectory calculations
- [ ] Check acceleration command generation
- [ ] Validate time-to-go calculation
- [ ] Confirm coordinate frame conversions
- [ ] Verify target tracking logic
- [ ] Check proportional navigation implementation

#### Interface
**Inputs:**
- Position (ECEF or local frame)
- Velocity (ECEF or local frame)
- Target position/velocity
- Current attitude
- Mission parameters

**Outputs:**
- `systemState.guidanceState.accelCmdBodyY` (pitch acceleration command)
- `systemState.guidanceState.accelCmdBodyZ` (yaw acceleration command)
- `systemState.guidanceState.timeToGo` (float, seconds)
- Additional guidance telemetry

#### Testing Status
- [ ] Unit tests reviewed
- [ ] Type consistency verification
- [ ] Integration testing
- [ ] HIL testing

#### Known Issues
- None identified yet - module pending review

---

### ⏳ 5. Sensor Processing Module

**Status:** PARTIALLY INTEGRATED ⏳  
**Test Module:** Various in `test/`  
**Integrated Module:** `src/minor.c` (lines 46-509)  
**Integration Date:** TBD  

#### Integration Tasks
- [x] Accelerometer data processing
- [x] Gyroscope data processing
- [x] Magnetometer data processing
- [x] Incremental velocity processing
- [x] Incremental angle processing
- [ ] Verify all sensor health checks
- [ ] Validate LSB to physical unit conversions
- [ ] Check flash storage interfaces
- [ ] Verify telemetry interfaces

#### Interface
**Inputs:**
- `systemState.imuData.*_raw` (raw LSB values from hardware)
- `systemState.navigationState.sensorHealth` (health status bits)
- Sensor calibration offsets

**Outputs:**
- `systemState.accelerometerData.accel_current` (m/s²)
- `systemState.gyroscopeData.gyro_current` (rad/s)
- `systemState.magnetometerData.mag_current` (mG)
- `systemState.incrementalVelocityData.delta_v_converted` (m/s)
- `systemState.incrementalAngleData.delta_theta_obc` (rad)
- `systemState.rollRateFp` (rps, for sequencer)
- `systemState.angularRates` (rad/s, for DAP)

#### Testing Status
- [x] Basic implementation complete
- [ ] Unit tests reviewed
- [ ] Conversion factor validation
- [ ] Integration testing
- [ ] HIL testing

#### Known Issues
- Flash storage functions are placeholders
- Telemetry functions are placeholders
- Need hardware team input for actual interfaces

---

## Critical Integration Points

### 1. Type Consistency ✅
- **Sequencer `tGo`:** Now consistent as `float` (seconds) in both test and integrated modules
- **Roll rate:** Consistent as `float` (rps) across all modules
- **Timing counters:** Consistent as `uint32_t` (cycles)

### 2. Timing Synchronization
- **Minor Cycle:** 100Hz (10ms) - Navigation, Sequencer, DAP
- **Major Cycle:** 10Hz (100ms) - Guidance
- **DAP Command Hold:** DAP uses same guidance command for 10 consecutive minor cycles

### 3. Coordinate Frames
- **Body Frame:** X-forward, Y-right, Z-down
- **NED Frame:** North-East-Down
- **ECEF Frame:** Earth-Centered Earth-Fixed
- **Transformations:** Verify all frame conversions are consistent

### 4. Data Flow
```
Hardware IMU → Sensor Processing → Navigation → 
                ↓                      ↓
            Sequencer              DAP ← Guidance (10Hz)
                ↓                      ↓
            Flags/Phases         Actuator Commands
```

---

## Verification Checklist

### Code Quality
- [ ] All modules follow MISRA C guidelines
- [ ] Consistent naming conventions
- [ ] Comprehensive error handling
- [ ] Input parameter validation
- [ ] No magic numbers (use #defines)
- [ ] Adequate comments and documentation

### Safety Critical Items
- [ ] Division by zero checks
- [ ] Array bounds checking
- [ ] Integer overflow protection
- [ ] Floating point comparison tolerances
- [ ] Sensor fault handling
- [ ] Graceful degradation modes

### Performance
- [ ] Minor cycle execution time < 10ms
- [ ] Major cycle execution time < 100ms
- [ ] No dynamic memory allocation
- [ ] Efficient algorithms
- [ ] Minimal stack usage

### Testing
- [ ] Unit tests for all modules
- [ ] Integration tests with system state
- [ ] Boundary condition testing
- [ ] Failure mode testing
- [ ] Timing validation
- [ ] HIL testing with hardware

---

## Next Steps

1. **Complete Sequencer Integration Testing** ✅
   - Run unit tests with updated type signatures
   - Verify timing windows and phase transitions
   - Test with realistic roll rate profiles

2. **Navigation Module Verification**
   - Review attitude estimation algorithms
   - Validate sensor fusion logic
   - Test 3-cycle confirmation logic

3. **DAP Module Verification**
   - Review control algorithms
   - Validate gain schedules
   - Test actuator command generation

4. **Guidance Module Integration**
   - Review guidance algorithms
   - Integrate into major cycle
   - Verify interface with DAP

5. **System Integration Testing**
   - Full system simulation
   - Hardware-in-the-loop testing
   - Flight test preparation

---

## Contact Information

**Integration Lead:** Ananthu Dev, Project Engineer  
**Organization:** Spacelabs  
**Date:** 2025  

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-11-11 | Integration Team | Initial checklist creation, Sequencer module completed |

---

**END OF DOCUMENT**

