# ISA Projectile Sequencer Logic Flow

## Overview

The sequencer module manages mission phases and state transitions for precision-guided projectiles. It orchestrates the timing and sequence of critical events during flight through four key phases (T0, T1, T2, T3).

## Phase Transitions

### T0 Phase (Launch)

- **Trigger**: OBC reset (launch detection)
- **Actions**:
  - Reset main clock cycles to zero
  - Initialize phase T0
  - Reset confirmation counters

### T1 Phase (Canard Deployment)

- **Triggers**:
  - **Condition 1**: Roll rate ≤ 7 rps for 3 consecutive cycles after T1 window in (0.1s)
  - **Condition 2**: Timeout after T1 window out (5s)
- **Actions**:
  - Set T1 phase marker
  - Record T1 set time
  - Schedule canard deploy flag

### T2 Phase (Canard Control)

- **Triggers**:
  - **Condition 1**: Roll rate ≤ 2 rps for 3 consecutive cycles after T2 window in (T1 + 0.1s)
  - **Condition 2**: Timeout after T2 window out (T1 + 5s)
- **Actions**:
  - Set T2 phase marker
  - Record T2 set time
  - Schedule canard control flag

### T3 Phase (Guidance Active)

- **Triggers**:
  - **Condition 1**: T3 window out (T2 + 5s)
  - **Condition 2**: T3 window in (T2 + 0.1s) AND tGo < proximity time threshold
- **Actions**:
  - Set T3 phase marker
  - Record T3 set time
  - Enable proximity sensor

## Flag Sequence

### 1. Canard Deploy Flag

- **Scheduled**: When T1 is set
- **Timing**: After canardDeployFlagSendTime (T1 + delay, currently 0)
- **Purpose**: Deploy canards for aerodynamic control

### 2. Canard Control Flag

- **Scheduled**: When T2 is set
- **Timing**: After canardControlFlagSendTime (T2 + delay, currently 0)
- **Purpose**: Activate control surfaces
- **Effect**: Schedules FSA and guidance flags

### 3. FSA Activate Flag

- **Scheduled**: When canard control flag is sent
- **Timing**: After fsaActivateFlagSendTime (canardControl + delay, currently 0)
- **Purpose**: Activate flight stability systems

### 4. Guidance Start Flag

- **Scheduled**: When canard control flag is sent
- **Timing**: After guidStartFlagSendTime (canardControl + 200 cycles = 2s)
- **Purpose**: Activate guidance algorithms

## Detailed Logic Flow

### Main Execution Priority

The sequencer executes with the following priority hierarchy:

1. Check if T3 is set (if yes, exit - mission complete)
2. Check if T2 is set (if yes, process T3 logic)
3. Check if T1 is set (if yes, process T2 logic)
4. Check if T0 is set (if yes, process T1 logic)

### T1 Processing Logic

```
IF T1 window out time reached:
    Set T1
    Schedule canard deploy flag
ELSE IF T1 window in time reached:
    IF roll rate <= 7 rps:
        Increment confirmation counter
        IF confirmation counter >= 3:
            Set T1
            Schedule canard deploy flag
    ELSE:
        Reset confirmation counter
```

### T2 Processing Logic

```
IF canard deploy flag not sent yet:
    Check if it's time to send
    If yes, send flag and mark as sent

IF T2 window out time reached:
    Set T2
    Schedule canard control flag
ELSE IF T2 window in time reached:
    IF roll rate <= 2 rps:
        Increment confirmation counter
        IF confirmation counter >= 3:
            Set T2
            Schedule canard control flag
    ELSE:
        Reset confirmation counter
```

### T3 Processing Logic

```
IF guidance start flag already sent:
    IF FSA flag already sent:
        IF T3 window out time reached:
            Set T3
            Enable proximity sensor
        ELSE IF T3 window in time reached:
            IF tGo < proximity threshold:
                Set T3
                Enable proximity sensor
    ELSE:
        Send FSA flag if it's time
ELSE:
    IF canard control flag sent:
        IF FSA flag already sent:
            Send guidance start flag if it's time
        ELSE:
            Send FSA flag if it's time
            Check guidance flag timing regardless
    ELSE:
        Send canard control flag if it's time
        Schedule FSA and guidance flags when sent
        Check T2 window conditions
```

## Safety Mechanisms

1. **Timeouts**: Each phase has a window out time that triggers the next phase even if conditions aren't met
2. **Confirmation Counters**: Ensure stability of measurements before phase transitions (3 consecutive cycles)
3. **Flag Scheduling**: Flags are scheduled with appropriate delays to ensure proper sequencing
4. **Parameter Validation**: All inputs are validated before processing

## Critical Timing Parameters

- **SEQ_T1_WINDOW_IN_TIME**: 10 cycles (0.1s)
- **SEQ_T1_WINDOW_OUT_TIME**: 500 cycles (5s)
- **SEQ_T2_WINDOW_IN_TIME**: 10 cycles (0.1s after T1)
- **SEQ_T2_WINDOW_OUT_TIME**: 500 cycles (5s after T1)
- **SEQ_T3_WINDOW_IN_TIME**: 10 cycles (0.1s after T2)
- **SEQ_T3_WINDOW_OUT_TIME**: 500 cycles (5s after T2)
- **SEQ_CANARD_DEPLOY_FLAG_DELAY**: 0 cycles
- **SEQ_CANARD_CONTROL_ON_FLAG_DELAY**: 0 cycles
- **SEQ_FSA_FLAG_DELAY**: 0 cycles
- **SEQ_GUID_START_FLAG_DELAY**: 200 cycles (2s)

## Ananthu Dev, Project Engineer - Spacelabs
