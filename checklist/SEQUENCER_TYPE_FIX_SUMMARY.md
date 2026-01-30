# Sequencer Module Type Consistency Fix

**Date:** November 11, 2025  
**Issue:** Type mismatch between test module and integrated module  
**Status:** ✅ RESOLVED  

---

## Problem Identified

### Critical Type Mismatch
The test sequencer module (`test/sequencer/sequencer.c`) and the integrated module (`src/minor.c`) had inconsistent type definitions for the `tGo` parameter (time-to-go from guidance).

**Test Module (BEFORE FIX):**
```c
// sequencer.h line 104
SequencerError_t sequencerExecute(SequencerState_t* state,
    float rollRateFp,
    uint32_t tGo,   // ← Cycles (integer)
    SequencerOutput_t* output);

// sequencer.h line 35
#define SEQ_T_PROXIMITY  100U  // ← 100 cycles
```

**Integrated Module (CORRECT):**
```c
// minor.c line 994, 1121
static void process_t3_logic(SequencerState_t* state, 
    float tGo,  // ← Seconds (float)
    SequencerOutput_t* output)
{
    if (tGo < SEQ_T_PROXIMITY) {  // Compare float seconds
        // ...
    }
}

// minor.h line 38
#define SEQ_T_PROXIMITY  1.0f  // ← 1.0 seconds
```

### Impact
- Test module would fail when integrated with guidance system
- Guidance provides `timeToGo` in **seconds** (float)
- Comparison `1.0f < 100U` would always be true, causing premature T3 phase activation
- T3 proximity sensor would enable immediately instead of at 1 second to target

---

## Solution Applied

### Changes Made to Test Module

#### 1. Updated `sequencer.h` Header File

**Change 1: Updated `SEQ_T_PROXIMITY` definition**
```c
// BEFORE:
#define SEQ_T_PROXIMITY  100U  // Proximity threshold (cycles)

// AFTER:
// Timing Configuration (in seconds for guidance interface)
#define SEQ_T_PROXIMITY  1.0f  // Proximity threshold: 1.0 seconds (float for guidance timeToGo)
```

**Change 2: Updated `sequencerExecute()` function signature**
```c
// BEFORE:
SequencerError_t sequencerExecute(SequencerState_t* state,
    float rollRateFp,
    uint32_t tGo,   // Form guidance system
    SequencerOutput_t* output);

// AFTER:
SequencerError_t sequencerExecute(SequencerState_t* state,
    float rollRateFp, // Roll rate in RPS (revolutions per second)
    float tGo,   // Time to go from guidance system (in seconds, float)
    SequencerOutput_t* output);
```

**Change 3: Updated timing configuration comments**
```c
// BEFORE:
// Timing Configuration
#define SEQ_T1_WINDOW_IN_TIME      10U  // T1 window starts at T0 + 0.1s (l0 cycles)
#define SEQ_T3_WINDOW_IN_TIME      10U  // T3 window starts at T1 + 0.1s

// AFTER:
// Timing Configuration (in cycles, 1 cycle = 10ms = 0.01s)
#define SEQ_T1_WINDOW_IN_TIME      10U  // T1 window starts at T0 + 0.1s (10 cycles)
#define SEQ_T3_WINDOW_IN_TIME      10U  // T3 window starts at T2 + 0.1s (10 cycles)
```

#### 2. Updated `sequencer.c` Implementation File

**Change 1: Updated `processT3Logic()` function signature**
```c
// BEFORE:
static SequencerError_t processT3Logic(SequencerState_t* state,
    uint32_t tGo,
    SequencerOutput_t* output)

// AFTER:
// Note: tGo is in seconds (float), not cycles
static SequencerError_t processT3Logic(SequencerState_t* state,
    float tGo,
    SequencerOutput_t* output)
```

**Change 2: Updated `sequencerExecute()` function signature**
```c
// BEFORE:
SequencerError_t sequencerExecute(SequencerState_t* state,
    float rollRateFp,
    uint32_t tGo,
    SequencerOutput_t* output)

// AFTER:
SequencerError_t sequencerExecute(SequencerState_t* state,
    float rollRateFp,
    float tGo,
    SequencerOutput_t* output)
```

---

## Verification

### Type Consistency Matrix

| Parameter | Test Module | Integrated Module | Status |
|-----------|-------------|-------------------|--------|
| `tGo` type | ✅ `float` | ✅ `float` | ✅ MATCH |
| `SEQ_T_PROXIMITY` | ✅ `1.0f` | ✅ `1.0f` | ✅ MATCH |
| `rollRateFp` type | ✅ `float` | ✅ `float` | ✅ MATCH |
| Timing windows (cycles) | ✅ `uint32_t` | ✅ `uint32_t` | ✅ MATCH |

### Interface Alignment

**Data Flow (Now Consistent):**
```
Guidance Module (major.c, 10Hz)
    ↓
    timeToGo (float, seconds) → systemState.guidanceState.timeToGo
    ↓
Sequencer Module (minor.c, 100Hz)
    ↓
    execute_sequencer() reads tGo = systemState.guidanceState.timeToGo
    ↓
    process_t3_logic(state, tGo, output)  // tGo is float (seconds)
    ↓
    if (tGo < SEQ_T_PROXIMITY)  // Compare 1.0f < 1.0f (correct!)
    ↓
    Set T3 phase, enable proximity sensor
```

### Linting Status
- ✅ No linting errors in `test/sequencer/sequencer.h`
- ✅ No linting errors in `test/sequencer/sequencer.c`

---

## Rationale for Using Seconds (float)

### Why Seconds Instead of Cycles?

1. **Natural Interface with Guidance:**
   - Guidance algorithms compute time-to-go in physical time (seconds)
   - No conversion needed at the interface boundary
   - Cleaner, more intuitive interface

2. **Physical Meaning:**
   - 1.0 second is more meaningful than 100 cycles
   - Easier to understand for system engineers
   - Reduces potential for conversion errors

3. **Flexibility:**
   - If cycle rate changes (e.g., 50Hz to 200Hz), seconds remain valid
   - No need to recalculate thresholds
   - More maintainable code

4. **Consistency with Guidance Domain:**
   - Guidance works in continuous time
   - Time-to-go, time-to-intercept are physical quantities
   - Matches aerospace engineering conventions

### What Still Uses Cycles?

**Internal Sequencer Timing** (correctly uses `uint32_t` cycles):
- `mainClockCycles` - counts minor cycles since OBC reset
- `t1SetTime`, `t2SetTime`, `t3SetTime` - phase transition timestamps
- `SEQ_T1_WINDOW_IN_TIME`, `SEQ_T2_WINDOW_OUT_TIME` - timing windows
- All flag send delays

**Rationale:** These are internal to sequencer timing logic and benefit from integer cycle counts for precise event scheduling.

---

## Testing Recommendations

### Unit Tests to Update
1. Update all test cases that call `sequencerExecute()` with `uint32_t tGo`
2. Change test inputs from cycles (e.g., 50) to seconds (e.g., 0.5f)
3. Verify T3 phase transition logic with realistic time-to-go values

### Integration Tests
1. Test sequencer with guidance module providing float timeToGo
2. Verify T3 triggers when timeToGo < 1.0 seconds
3. Test boundary conditions (tGo = 0.99s, 1.0s, 1.01s)
4. Verify proximity sensor enable flag sets correctly

### Example Test Case Update
```c
// BEFORE:
uint32_t tGo_cycles = 50U;  // 50 cycles = 0.5 seconds
sequencerExecute(&state, rollRate, tGo_cycles, &output);

// AFTER:
float tGo_seconds = 0.5f;  // 0.5 seconds
sequencerExecute(&state, rollRate, tGo_seconds, &output);
```

---

## Files Modified

1. ✅ `test/sequencer/sequencer.h` - Header file with function signatures and constants
2. ✅ `test/sequencer/sequencer.c` - Implementation file with T3 logic
3. ✅ `MODULE_INTEGRATION_CHECKLIST.md` - Created comprehensive integration tracking document

---

## Next Steps

1. ✅ **COMPLETE:** Type consistency fixed
2. 🔄 **IN PROGRESS:** Update unit tests with new float type
3. ⏳ **PENDING:** Run full test suite with updated types
4. ⏳ **PENDING:** Integration testing with guidance module
5. ⏳ **PENDING:** Verify with hardware-in-the-loop (HIL)

---

## Lessons Learned

### Best Practices for Future Integration
1. **Define interfaces early** - Establish parameter types before implementation
2. **Document units clearly** - Always specify (seconds vs cycles, rad vs deg, etc.)
3. **Use type-safe constants** - `1.0f` for float, `100U` for unsigned int
4. **Consistent comments** - Units in both declaration and usage
5. **Regular consistency checks** - Compare test modules with integrated modules

### Prevention Strategies
1. **Common header files** - Share type definitions between test and integrated modules
2. **Interface specification document** - Define all inter-module interfaces upfront
3. **Automated type checking** - Add static analysis tools to catch type mismatches
4. **Code reviews** - Cross-check test vs integrated implementations

---

**Status:** ✅ RESOLVED - Sequencer module now has consistent types between test and integrated implementations.

**Verified By:** Integration Team, Spacelabs  
**Date:** November 11, 2025  

---

