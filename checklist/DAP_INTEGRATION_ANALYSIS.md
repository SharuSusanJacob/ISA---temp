# DAP Module Integration Analysis

**Date:** November 11, 2025  
**Modules Compared:**  
- Test Module: `test/dap/dap.c`  
- Integrated Module: `src/minor.c` (lines 1179-1415)

---

## Overview

Comparison between standalone DAP test module and integrated DAP implementation in minor cycle.

---

## Key Differences Found

### 1. ✅ Parameter Passing Method

**Test Module (`dap.c`):**
```c
DAPOutput_t ComputeDeltaRollCommand(
    float rollAngle_rad,
    float rollRate_radps,
    DAPParameters_t dapParams,  // ← Pass by VALUE
    float timeStep_s
)
```

**Integrated Module (`minor.c`):**
```c
static DAPOutput_t compute_delta_roll_command(
    float rollAngle_rad,
    float rollRate_radps,
    const DapParams_t* dapParams,  // ← Pass by POINTER (const)
    float timeStep_s
)
```

**Analysis:**
- ✅ **Integrated version is BETTER** - passing by pointer is more efficient
- Structure is 64 bytes (16 floats × 4 bytes)
- Passing by value copies entire structure onto stack
- Passing by pointer only copies 4-8 bytes (pointer size)
- **Action:** Update test module to use pointer passing

---

### 2. ✅ Division-by-Zero Protection (CRITICAL SAFETY IMPROVEMENT)

**Test Module (`dap.c`) - NO PROTECTION:**
```c
PYOutput_t ComputeDeltaPitchCommand(..., float timeStep_s)
{
    // ...
    qDot = (currentPitchrate - previousPitchrate) / timeStep_s;  // ← UNSAFE!
    // ...
}
```

**Integrated Module (`minor.c`) - HAS PROTECTION:**
```c
static PYOutput_t compute_delta_pitch_command(..., float timeStep_s)
{
    // ...
    /* Safety check for time step */
    float safe_timeStep = timeStep_s;
    if (safe_timeStep < MATH_EPSILON) {  // ← SAFETY CHECK!
        safe_timeStep = MATH_EPSILON;
    }
    
    /* Calculate pitch rate derivative */
    qDot = (currentPitchrate - previousPitchrate) / safe_timeStep;  // ← SAFE!
    // ...
}
```

**Analysis:**
- ✅ **Integrated version has critical safety improvement**
- Protects against division by zero or very small timestep
- `MATH_EPSILON` likely defined as 1e-6 or similar small value
- **Action:** Add this protection to test module

**Impact if missing:**
- If `timeStep_s == 0.0`, causes divide-by-zero → `NaN` → invalid commands
- If `timeStep_s` very small (e.g., 1e-9), causes numerical instability
- Could result in actuator saturation or loss of control

---

### 3. ⚠️ Type Name Difference (Not a problem)

**Test Module:** `DAPParameters_t`
**Integrated Module:** `DapParams_t`

**Analysis:**
- Just naming convention difference
- Both structures have identical members
- No functional impact

---

### 4. ✅ Control Flow Structure

**Test Module (`dap.c`):**
```c
DAPOutput_t DAP_Execute(
    float rollAngle_rad,
    float accelerationY_mps2,
    float accelerationZ_mps2,
    float rollRate_radps,
    float pitchRate_radps,
    float yawRate_radps,
    float accelerationYCommand_mps2,
    float accelerationZCommand_mps2,
    DAPParameters_t dapParams,
    float timeStep_s,
    bool canardControlFlag
)
{
    DAPOutput_t deltar = { 0.0, 0.0, 0.0, 0.0 };
    DAPOutput_t out = { 0.0, 0.0, 0.0, 0.0 };
    PYOutput_t deltap = { 0.0, 0.0 };
    PYOutput_t deltay = { 0.0, 0.0 };

    if (canardControlFlag != false)  // ← Direct parameter check
    {
        if (rollRate_radps != 0.0f)
        {   
            deltar = ComputeDeltaRollCommand(...);
            out = deltar;
        }
        else
        {
            deltap = ComputeDeltaPitchCommand(...);
            deltay = ComputeDeltaYawCommand(...);
            out.delta3_rad = deltap.delta1_rad;
            out.delta6_rad = deltap.delta2_rad;
            out.delta9_rad = deltay.delta1_rad;
            out.delta12_rad = deltay.delta2_rad;
        }
        return out;  // ← Early return
    }
    else
    {
        out.delta3_rad = 0.0;
        out.delta6_rad = 0.0;
        out.delta9_rad = 0.0;
        out.delta12_rad = 0.0;
        return out;  // ← Early return
    }
}
```

**Integrated Module (`minor.c`):**
```c
static void execute_dap(float dt_s)
{
    /* Only execute DAP if canard control is enabled */
    if (!systemState.flags.canardControlFlag) {  // ← Early exit guard
        /* Canard control disabled - set zero actuator commands */
        systemState.actuatorCommands.x = 0.0f;
        systemState.actuatorCommands.y = 0.0f;
        systemState.actuatorCommands.z = 0.0f;
        return;  // ← Early exit
    }

    /* Read inputs from systemState */
    float accelerationYCommand = systemState.guidanceState.accelCmdBodyY;
    float accelerationZCommand = systemState.guidanceState.accelCmdBodyZ;
    float accelerationY = systemState.accelerometerData.accel_current.y;
    float accelerationZ = systemState.accelerometerData.accel_current.z;
    float rollRate = systemState.angularRates.x;
    float pitchRate = systemState.angularRates.y;
    float yawRate = systemState.angularRates.z;
    float rollAngle = systemState.navigationState.attitude_e.roll_rad;
    const DapParams_t* dapParams = &systemState.dapParams;

    /* Execute DAP algorithm */
    DAPOutput_t dapOutput;
    DAPOutput_t deltar = { 0.0f, 0.0f, 0.0f, 0.0f };
    PYOutput_t deltap = { 0.0f, 0.0f };
    PYOutput_t deltay = { 0.0f, 0.0f };

    if (systemState.flags.canardControlFlag) {  // ← Redundant check!
        /* Canard control enabled - execute DAP */
        if (rollRate != 0.0f) {
            /* Roll rate is non-zero - use roll control */
            deltar = compute_delta_roll_command(rollAngle, rollRate, dapParams, dt_s);
            dapOutput = deltar;
        }
        else {
            /* Roll rate is zero - use pitch and yaw control */
            deltap = compute_delta_pitch_command(accelerationY, pitchRate,
                accelerationYCommand, dapParams, dt_s);
            deltay = compute_delta_yaw_command(accelerationZ, yawRate,
                accelerationZCommand, dapParams, dt_s);

            dapOutput.delta3_rad = deltap.delta1_rad;
            dapOutput.delta6_rad = deltap.delta2_rad;
            dapOutput.delta9_rad = deltay.delta1_rad;
            dapOutput.delta12_rad = deltay.delta2_rad;
        }
    }
    else {  // ← UNREACHABLE CODE! Already returned above.
        /* Canard control disabled - zero commands */
        dapOutput.delta3_rad = 0.0f;
        dapOutput.delta6_rad = 0.0f;
        dapOutput.delta9_rad = 0.0f;
        dapOutput.delta12_rad = 0.0f;
    }

    /* Map DAP output to actuator commands */
    systemState.actuatorCommands.x = dapOutput.delta3_rad;
    systemState.actuatorCommands.y = dapOutput.delta6_rad;
    systemState.actuatorCommands.z = dapOutput.delta9_rad;
}
```

**Analysis:**
- ⚠️ **Integrated version has REDUNDANT CHECK**
- Line 1344-1350: Early exit if `!canardControlFlag`
- Line 1380: Checks `canardControlFlag` AGAIN
- Else block (1400-1406) is **UNREACHABLE CODE**
- **Action:** Remove redundant check and unreachable else block

---

### 5. ✅ Actuator Command Storage

**Test Module:**
- Returns `DAPOutput_t` structure with all 4 canard deflections
- Caller responsible for storing commands

**Integrated Module:**
- Stores directly to `systemState.actuatorCommands`
- Only stores 3 canards (x, y, z)
- `delta12_rad` is lost (noted in TODO comment)

**Analysis:**
- ⚠️ **Potential data loss** - delta12_rad not stored
- `actuatorCommands` is `Vector3_t` (only 3 components)
- Need 4 components for 4-canard system
- **Action:** Extend `actuatorCommands` structure or create new field

---

## Algorithm Correctness Comparison

### Roll Control Algorithm
✅ **IDENTICAL** - Both implementations use same PI controller with anti-windup

### Pitch Control Algorithm  
✅ **FUNCTIONALLY IDENTICAL** - Same acceleration error calculation
- Integrated version adds safety check (improvement)

### Yaw Control Algorithm
✅ **FUNCTIONALLY IDENTICAL** - Same acceleration error calculation
- Integrated version adds safety check (improvement)

---

## Issues Summary

| # | Issue | Severity | Test Module | Integrated Module | Action |
|---|-------|----------|-------------|-------------------|--------|
| 1 | Parameter passing | LOW | By value | By pointer (better) | Update test to use pointer |
| 2 | Division-by-zero protection | **CRITICAL** | Missing | Has protection | Add to test module |
| 3 | Redundant flag check | MEDIUM | N/A | Redundant check | Remove from integrated |
| 4 | Unreachable else block | MEDIUM | N/A | Dead code | Remove from integrated |
| 5 | delta12_rad storage | MEDIUM | Returns all 4 | Only stores 3 | Extend structure |
| 6 | Type name consistency | LOW | DAPParameters_t | DapParams_t | Cosmetic only |

---

## Recommendations

### Priority 1 - Critical Fixes

1. ✅ **INTEGRATED MODULE IS MORE IMPORTANT** - Keep division-by-zero protection
2. 🔧 **Fix Test Module** - Add safety checks to match integrated version
3. 🔧 **Fix Integrated Module** - Remove redundant check and unreachable code

### Priority 2 - Important Improvements

4. 🔧 **Extend actuatorCommands** - Store all 4 canard deflections
5. 🔧 **Update Test Module** - Use pointer passing for parameters

### Priority 3 - Code Quality

6. 📝 **Standardize naming** - Use consistent type names across modules

---

## Detailed Fix Plan

### Fix 1: Add Division-by-Zero Protection to Test Module

**File:** `test/dap/dap.c`

**Add to `ComputeDeltaPitchCommand()`:**
```c
PYOutput_t ComputeDeltaPitchCommand(...)
{
    float currentPitchrate = pitchRate_radps;
    float qDot;
    float accErrorPitch;
    float deltaCommandPitch;
    PYOutput_t outP;

    /* ADD THIS SAFETY CHECK */
    float safe_timeStep = timeStep_s;
    if (safe_timeStep < 1e-6f) {  /* MATH_EPSILON equivalent */
        safe_timeStep = 1e-6f;
    }

    /* Calculate pitch rate derivative */
    qDot = (currentPitchrate - previousPitchrate) / safe_timeStep;  /* Use safe_timeStep */
    // ... rest unchanged
}
```

**Add to `ComputeDeltaYawCommand()`:**
```c
PYOutput_t ComputeDeltaYawCommand(...)
{
    float currentYawrate = yawRate_radps;
    float rDot;
    float accErrorYaw;
    float deltaCommandYaw;
    PYOutput_t outY;

    /* ADD THIS SAFETY CHECK */
    float safe_timeStep = timeStep_s;
    if (safe_timeStep < 1e-6f) {  /* MATH_EPSILON equivalent */
        safe_timeStep = 1e-6f;
    }

    /* Calculate yaw rate derivative */
    rDot = (currentYawrate - previousYawrate) / safe_timeStep;  /* Use safe_timeStep */
    // ... rest unchanged
}
```

---

### Fix 2: Remove Redundant Check in Integrated Module

**File:** `src/minor.c` lines 1374-1415

**Current (problematic):**
```c
static void execute_dap(float dt_s)
{
    /* Early exit if disabled */
    if (!systemState.flags.canardControlFlag) {
        systemState.actuatorCommands.x = 0.0f;
        systemState.actuatorCommands.y = 0.0f;
        systemState.actuatorCommands.z = 0.0f;
        return;
    }

    /* ... read inputs ... */

    DAPOutput_t dapOutput;
    // ...

    if (systemState.flags.canardControlFlag) {  // ← REDUNDANT!
        // ... compute commands ...
    }
    else {  // ← UNREACHABLE!
        dapOutput.delta3_rad = 0.0f;
        // ...
    }

    /* Store commands */
    systemState.actuatorCommands.x = dapOutput.delta3_rad;
    // ...
}
```

**Fixed (cleaner):**
```c
static void execute_dap(float dt_s)
{
    /* Early exit if disabled */
    if (!systemState.flags.canardControlFlag) {
        systemState.actuatorCommands.x = 0.0f;
        systemState.actuatorCommands.y = 0.0f;
        systemState.actuatorCommands.z = 0.0f;
        return;
    }

    /* ... read inputs ... */

    /* Execute DAP algorithm - canard control is enabled if we reach here */
    DAPOutput_t dapOutput;
    DAPOutput_t deltar = { 0.0f, 0.0f, 0.0f, 0.0f };
    PYOutput_t deltap = { 0.0f, 0.0f };
    PYOutput_t deltay = { 0.0f, 0.0f };

    if (rollRate != 0.0f) {
        /* Roll rate is non-zero - use roll control */
        deltar = compute_delta_roll_command(rollAngle, rollRate, dapParams, dt_s);
        dapOutput = deltar;
    }
    else {
        /* Roll rate is zero - use pitch and yaw control */
        deltap = compute_delta_pitch_command(accelerationY, pitchRate,
            accelerationYCommand, dapParams, dt_s);
        deltay = compute_delta_yaw_command(accelerationZ, yawRate,
            accelerationZCommand, dapParams, dt_s);

        dapOutput.delta3_rad = deltap.delta1_rad;
        dapOutput.delta6_rad = deltap.delta2_rad;
        dapOutput.delta9_rad = deltay.delta1_rad;
        dapOutput.delta12_rad = deltay.delta2_rad;
    }

    /* Store commands */
    systemState.actuatorCommands.x = dapOutput.delta3_rad;
    systemState.actuatorCommands.y = dapOutput.delta6_rad;
    systemState.actuatorCommands.z = dapOutput.delta9_rad;
    /* TODO: Store delta12_rad - need to extend actuatorCommands structure */
}
```

---

## Testing Checklist

- [ ] Test division-by-zero protection with `timeStep = 0.0`
- [ ] Test with very small timestep (`timeStep = 1e-10`)
- [ ] Verify roll control with non-zero roll rate
- [ ] Verify pitch/yaw control with zero roll rate
- [ ] Test canard control enable/disable flag
- [ ] Verify all 4 canard commands are generated
- [ ] Check that commands are stored correctly

---

**Status:** Analysis Complete - Fixes Ready to Apply

**Next Actions:**
1. Fix integrated module (remove redundant code)
2. Update test module (add safety checks)
3. Address delta12_rad storage issue
4. Update integration checklist

---

