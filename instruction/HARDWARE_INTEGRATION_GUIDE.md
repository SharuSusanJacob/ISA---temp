# Flight Software Hardware Integration & Reversion Guide

**Document Purpose**: This guide provides the hardware team with the necessary instructions to revert the test-mode bypasses and integrate real sensor data/conversions into the flight software codebase.

---

## 1. Exit Test Mode & Centralized Reset
The flight software is currently controlled by a global `testMode` flag. When true, it bypasses hardware sensor processing and relies on CSV injection.

**Action Required**:
- In `fswtest.c` (or your target production `main.c`), set `systemState.testMode = false;`.
- **CRITICAL**: Do NOT set `isOBCReset` or `isT0Set` manually. Instead, map your hardware reset signal (e.g., from power-on or a specific pin) to call `set_obc_reset(true);`. 
- This function (in `minor.c`) now handles:
  1. Loading all PEFCS parameters (Mission timings, DAP limits, Calibration offsets).
  2. Zeroing all DAP/Nav filters and integrators.
  3. Clearing Guidance persistence state (`prevAccel`).
  4. Setting the Sequencer to T0 phase.

---

## 2. Sensor Data Injection & LSB Conversions
In `src/major.c` and `src/minor.c`, the raw hardware interfaces are currently bypassed or commented out.

### GNSS Data (`src/major.c`)
- **Location**: `set_gnss_position_ecef` and `set_gnss_velocity_ecef` (Lines 188-231).
- **Status**: LSB-to-Double conversion functions are currently commented out. 
- **Action Required**: 
  - Uncomment the calls to `convert_ecef_position_lsb_to_double` and `convert_ecef_velocity_lsb_to_double`.
  - Implement these conversion functions in `src/type_convert.c` (ensure big-endian signed 32-bit to double conversion matches the ICD).
- **Critical Note on Coordinate Frames**: 
  - **WARNING**: The variables are named `ECEF`, but the Guidance Algorithm follows a legacy design that **expects ECI (Earth-Centered Inertial)** coordinates.
  - If your hardware provides true ECEF, you must convert it to ECI before calling `set_gnss_...`, or provide ECI directly if your receiver supports it.

### IMU/Magnetometer Data (`src/minor.c`)
- **Location**: `minor_cycle` (Lines 1850-1885).
- **Status**: When `testMode` is false, it calls `process_accelerometer_data()`, `process_gyroscope_data()`, etc.
- **Action Required**: 
  - Implement the "Read Hardware Register" logic in each of these functions.
  - Ensure the raw LSB values are passed to `src/type_convert.c` for proper unit conversion to physical units (m/s², rad/s, mG).

---

## 3. Mission Timing & Configurability
Mission events are no longer hardcoded. All timings are now accessed via `systemState.sequencerParams`.

- **Source**: `include/pefcs_init.h`
- **Integration Note**: The hardware team can now update mission durations (T1 Out, T2 Out, T3 Window) simply by updating the `PEFCS_SEQ_*` macros. No logic changes are required in `minor.c`.

---

## 4. Local Coordinate Frame Pre-computation
To match the precision of the reference implementation, the Local Frame axes are pre-computed once in `guidance_init()`.

- **Location**: `guidance_init` in `src/major.c`.
- **Action Required**: Ensure `guidance_init()` is called **after** the first valid GNSS lock is obtained at the launch pad, OR ensure the `PEFCS_ORIGIN_...` constants in `pefcs_init.h` are updated via the checkout system with exact launch coordinates.

---

## 5. Timing Constants
- **Major Cycle**: 10Hz (100ms).
- **Minor Cycle**: 100Hz (10ms).
- **Integration Step**: `INTEGRATION_STEP_SIZE` is fixed at `0.01`.
- **Warning**: Ensure the hardware timer/scheduler accurately triggers `minor_cycle` every 10ms. If the timing drifts, navigation integration will accumulate errors.

---

## Summary of Reversion Checklist
1. [ ] Set `systemState.testMode = false`.
2. [ ] Map hardware reset to `set_obc_reset(true)`.
3. [ ] Uncomment conversion calls in `src/major.c`.
4. [ ] Implement LSB-to-SI-unit logic in `src/type_convert.c`.
5. [ ] Map `gnss_lock` and IMU registers to real hardware peripherals.
6. [ ] Implement register reading in `src/minor.c`'s `process_..._data` functions.
7. [ ] Verify that `gnssDataBuffer` parsing is operational.
