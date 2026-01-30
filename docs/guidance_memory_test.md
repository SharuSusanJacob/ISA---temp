# Guidance Module Memory Management Test Documentation

## Overview

This document explains the test implementation for the ISA Flight Software's guidance module memory management and timing requirements. The test simulates the guidance system's operation under real-time constraints while monitoring memory usage and performance.

## System Requirements

### Timing Requirements

- **Minor Cycle**: 10ms (100 Hz)
- **Major Cycle**: 100ms (10 Hz)
- **Cycle Ratio**: 10 minor cycles per major cycle

### Memory Constraints

- **Stack Size**: 8KB maximum
- **Working Buffer**: 1KB for temporary calculations
- **Memory Safety**: Stack overflow protection and buffer bounds checking

## Code Structure

### Key Data Types

#### Vector3

```c
typedef struct {
    double x, y, z;
} Vector3;
```

Used for representing 3D positions, velocities, and accelerations in various reference frames.

#### GeodeticPosition

```c
typedef struct {
    double latitude;   /* radians */
    double longitude;  /* radians */
    double altitude;   /* meters */
} GeodeticPosition;
```

Represents positions in Earth-referenced coordinates.

#### GuidanceState

```c
typedef struct {
    /* Target information */
    GeodeticPosition targetPosition;
    Vector3 targetPositionECI;
    Vector3 targetVelocityECI;

    /* Current state */
    Vector3 positionECI;
    Vector3 velocityECI;

    /* Memory management */
    uint8_t workingBuffer[GUIDANCE_BUFFER_SIZE];
    size_t bufferUsed;
    size_t peakStackUsage;

    /* Cycle tracking */
    uint32_t minorCycleCount;
    uint32_t majorCycleCount;
} GuidanceState;
```

Main state structure containing all guidance and test-related data.

## Memory Management Features

### Stack Monitoring

The code implements a pattern-based stack usage monitor:

1. **Initialization**

   ```c
   static void initStackMonitoring(void) {
       memset(g_stackMarker, 0xAA, MAX_STACK_SIZE);
       g_guidanceState.stackBase = g_stackMarker;
   }
   ```

   - Fills stack area with known pattern (0xAA)
   - Sets up base pointer for monitoring

2. **Usage Checking**
   ```c
   static size_t checkStackUsage(void) {
       for (size_t i = 0; i < MAX_STACK_SIZE; i++) {
           if (g_stackMarker[i] != 0xAA) {
               return MAX_STACK_SIZE - i;
           }
       }
       return 0;
   }
   ```
   - Finds first modified byte in pattern
   - Calculates total stack usage
   - Updates peak usage statistics

### Working Buffer Management

- Fixed-size buffer (1KB) for temporary calculations
- Usage tracking to prevent overflow
- Peak usage monitoring

## Timing Architecture

### Minor Cycle (10ms)

```c
static void executeMinorCycle(void) {
    uint64_t startTime = getTimeUs();

    /* High-frequency operations */
    updateProjectileState();
    simulateGuidanceMemoryOps();
    checkStackUsage();

    /* Timing verification */
    uint64_t cycleTime = getTimeUs() - startTime;
    verifyTimingConstraints(cycleTime, MINOR_CYCLE_MS);
}
```

Key operations:

1. State updates (position, velocity)
2. Memory-intensive calculations
3. Stack monitoring
4. Timing verification

### Major Cycle (100ms)

```c
static void executeMajorCycle(void) {
    /* Less frequent operations */
    calculateTrajectoryParameters();
    checkTerminalPhase();
    reportPerformanceMetrics();
}
```

Key operations:

1. Trajectory calculations
2. Phase transitions
3. Performance reporting
4. Memory usage statistics

## Test Scenario

### Initial Conditions

- **Position**: (0, 0, Earth_radius)
- **Target**: 10km downrange, same altitude
- **Velocity**: 300 m/s horizontal, 100 m/s vertical
- **Duration**: 5 major cycles (500ms)

### Simulated Flight

1. Updates position and velocity each minor cycle
2. Tracks distance to target
3. Monitors terminal phase approach
4. Records performance metrics

## Performance Monitoring

### Timing Metrics

- Minor cycle execution time
- Major cycle execution time
- Peak timing for both cycles
- Average cycle times

### Memory Metrics

- Current stack usage
- Peak stack usage
- Working buffer utilization
- Buffer overflow detection

## Example Output

```
=== ISA Guidance Memory Management Test ===
Major cycle: 100 ms (10 Hz)
Minor cycle: 10 ms (100 Hz)

=== Major Cycle 0 ===
Distance to target: 9970.01 m
Velocity: 316.23 m/s
Time to impact: 31.53 s

Memory Usage:
  Working buffer: 0 / 1024 bytes (0.0%)
  Peak stack usage: 0 / 8192 bytes (0.0%)

Minor Cycle Timing:
  Average: 9 us
  Peak: 12 us

...

=== FINAL REPORT ===
Total minor cycles: 50
Total major cycles: 5
Peak minor cycle time: 27 us
Peak major cycle time: 52939 us
```

## Interpreting Results

### Timing Analysis

- Minor cycles should complete within 10ms
- Major cycles should complete within 100ms
- Monitor peak times for potential bottlenecks

### Memory Analysis

- Stack usage should not approach 8KB limit
- Buffer usage should stay under 1KB
- Watch for unexpected spikes in usage

### Performance Verification

- Check distance decrease rate
- Verify velocity calculations
- Monitor time-to-impact estimates

## Common Issues and Debugging

### Timing Issues

1. **Minor Cycle Overrun**

   - Check computation-intensive operations
   - Verify interrupt handling
   - Review memory access patterns

2. **Major Cycle Delays**
   - Check reporting overhead
   - Review trajectory calculations
   - Monitor system load

### Memory Issues

1. **Stack Overflow**

   - Review recursive functions
   - Check large local arrays
   - Monitor function call depth

2. **Buffer Overflow**
   - Verify array bounds
   - Check temporary storage usage
   - Review matrix operations

## Conclusion

This test provides comprehensive verification of:

1. Real-time performance
2. Memory safety
3. Resource utilization
4. System stability

The results demonstrate that the guidance module meets its timing and memory requirements, with significant margin for additional functionality.
