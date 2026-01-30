# Software Requirements Specification (SRS)
## ISA Flight Software - Missile Sequencer Module

### Document Information
- **Project**: ISA Flight Software
- **Module**: Missile Flight Sequencer
- **Version**: 1.0
- **Date**: 2025
- **Author**: Ananthu Dev, Project Engineer, Spacelabs

---

## 1. Introduction

### 1.1 Purpose
This document specifies the software requirements for the missile flight sequencer module. The sequencer manages different flight phases (T0, T1, T2, T3) and controls the timing of various subsystem activations during missile flight.

### 1.2 Scope
The sequencer is a real-time, safety-critical software module that:
- Monitors flight conditions (roll rate, timing windows)
- Manages state transitions between flight phases
- Controls flag transmission to subsystems
- Ensures proper sequencing of flight events

### 1.3 Standards Compliance
- **MISRA C**: All code shall comply with MISRA C guidelines
- **Real-time**: Deterministic execution within 10ms minor cycles
- **Safety**: Fail-safe operation with comprehensive error handling

---

## 2. System Overview

### 2.1 Flight Phases
The missile flight sequence consists of four main phases:

| Phase | Description | Entry Condition |
|-------|-------------|-----------------|
| T0 | Launch Phase | G-switch activation + OBC reset |
| T1 | Stabilization Phase | Roll rate ≤ 7.0 rps for 3 cycles |
| T2 | Guidance Preparation | Roll rate ≤ 2.0 rps for 3 cycles |
| T3 | Terminal Guidance | Time-based or proximity conditions |

### 2.2 System Inputs
- **G-switch Status**: Launch detection signal
- **Roll Rate**: Current missile roll rate (rps)
- **System Clock**: Minor cycle counter (10ms cycles)
- **Guidance Data**: t_go and proximity flag from guidance system

### 2.3 System Outputs
- **Phase Flags**: T0, T1, T2, T3 phase indicators
- **Control Flags**: FSA, Canard, Control, Guidance Start
- **Proximity Sensor**: Enable/disable command

---

## 3. Functional Requirements

### 3.1 Timing Requirements

#### FR-001: Minor Cycle Execution
- **Requirement**: The sequencer shall execute once per minor cycle (10ms)
- **Priority**: High
- **Rationale**: Real-time system requirement

#### FR-002: Clock Management
- **Requirement**: Main clock shall start when G-switch activates and OBC resets
- **Priority**: High
- **Rationale**: Timing reference for all operations

### 3.2 Phase Transition Requirements

#### FR-003: T0 Phase Entry
- **Requirement**: T0 shall be set when G-switch is active AND OBC is reset
- **Priority**: High
- **Rationale**: Launch detection and system initialization

#### FR-004: T1 Phase Entry
- **Requirement**: T1 shall be set when:
  - T0 is active, AND
  - Current time > T1 window in time, AND
  - Roll rate ≤ 7.0 rps for 3 consecutive minor cycles
- **Alternative**: T1 shall be set when current time > T1 window out time
- **Priority**: High
- **Rationale**: Missile stabilization verification

#### FR-005: T2 Phase Entry
- **Requirement**: T2 shall be set when:
  - T1 is active, AND
  - Current time > T2 window in time, AND
  - Roll rate ≤ 2.0 rps for 3 consecutive minor cycles
- **Alternative**: T2 shall be set when current time > T2 window out time
- **Priority**: High
- **Rationale**: Guidance system preparation

#### FR-006: T3 Phase Entry
- **Requirement**: T3 shall be set when:
  - T2 is active, AND
  - Current time > T3 window out time
- **Alternative**: T3 shall be set when:
  - T2 is active, AND
  - Current time > T3 window in time, AND
  - t_go < proximity threshold
- **Priority**: High
- **Rationale**: Terminal guidance activation

### 3.3 Flag Control Requirements

#### FR-007: FSA Flag Control
- **Requirement**: FSA flag shall be sent when:
  - T1 is active, AND
  - FSA flag not previously sent, AND
  - Current time > FSA flag send time
- **Priority**: Medium
- **Rationale**: Flight Surface Actuator activation

#### FR-008: Canard Flag Control
- **Requirement**: Canard flag shall be sent when:
  - T1 is active, AND
  - FSA flag already sent, AND
  - Canard flag not previously sent, AND
  - Current time > Canard flag send time
- **Priority**: Medium
- **Rationale**: Canard control surface activation

#### FR-009: Control Flag Control
- **Requirement**: Control flag shall be sent when:
  - T2 is active, AND
  - Control flag not previously sent, AND
  - Current time > Control flag send time
- **Priority**: Medium
- **Rationale**: Flight control system activation

#### FR-010: Guidance Start Flag Control
- **Requirement**: Guidance Start flag shall be sent when:
  - T2 is active, AND
  - Control flag already sent, AND
  - Guidance Start flag not previously sent, AND
  - Current time > Guidance Start flag send time
- **Priority**: Medium
- **Rationale**: Guidance system activation

#### FR-011: Proximity Sensor Control
- **Requirement**: Proximity sensor shall be enabled when T3 is set
- **Priority**: Medium
- **Rationale**: Terminal guidance sensor activation

### 3.4 Roll Rate Confirmation Requirements

#### FR-012: Roll Rate Sampling
- **Requirement**: Roll rate shall be sampled every minor cycle
- **Priority**: High
- **Rationale**: Continuous monitoring for phase transitions

#### FR-013: Confirmation Logic
- **Requirement**: Roll rate conditions shall be met for exactly 3 consecutive minor cycles before phase transition
- **Priority**: High
- **Rationale**: Noise filtering and stability verification

#### FR-014: Counter Reset
- **Requirement**: Confirmation counters shall reset to 0 when roll rate condition is not met
- **Priority**: High
- **Rationale**: Restart confirmation process

---

## 4. Non-Functional Requirements

### 4.1 Performance Requirements

#### NFR-001: Execution Time
- **Requirement**: Sequencer execution shall complete within 1ms
- **Priority**: High
- **Rationale**: Real-time constraint within 10ms minor cycle

#### NFR-002: Memory Usage
- **Requirement**: Total RAM usage shall not exceed 1KB
- **Priority**: Medium
- **Rationale**: Embedded system memory constraints

### 4.2 Reliability Requirements

#### NFR-003: Error Handling
- **Requirement**: All functions shall return error codes for status reporting
- **Priority**: High
- **Rationale**: System health monitoring

#### NFR-004: Parameter Validation
- **Requirement**: All input parameters shall be validated before use
- **Priority**: High
- **Rationale**: Prevent system corruption

### 4.3 Safety Requirements

#### NFR-005: Fail-Safe Operation
- **Requirement**: System shall enter safe state on critical errors
- **Priority**: High
- **Rationale**: Mission safety

#### NFR-006: Deterministic Behavior
- **Requirement**: All execution paths shall have predictable timing
- **Priority**: High
- **Rationale**: Real-time system requirement

---

## 5. Interface Requirements

### 5.1 Input Interfaces

#### IFR-001: Roll Rate Interface
- **Type**: uint16_t (fixed-point, 0.1 rps resolution)
- **Range**: 0 to 100.0 rps
- **Update Rate**: Every minor cycle

#### IFR-002: Guidance Interface
- **t_go**: uint32_t (minor cycles)
- **proximity_flag**: bool
- **Update Rate**: Every minor cycle

#### IFR-003: System Interface
- **G-switch**: bool (active/inactive)
- **Update Rate**: Event-driven

### 5.2 Output Interfaces

#### IFR-004: Flag Outputs
- **Type**: bool for each flag
- **Flags**: FSA, Canard, Control, Guidance Start, Proximity Sensor
- **Update Rate**: Every minor cycle

#### IFR-005: Phase Outputs
- **Type**: bool for each phase
- **Phases**: T0, T1, T2, T3
- **Update Rate**: Every minor cycle

---

## 6. Configuration Parameters

### 6.1 Timing Configuration

| Parameter | Default Value | Unit | Description |
|-----------|---------------|------|-------------|
| T1_WINDOW_IN_TIME | 100 | cycles | T1 window start time |
| T1_WINDOW_OUT_TIME | 500 | cycles | T1 window end time |
| T2_WINDOW_IN_TIME | 200 | cycles | T2 window start time |
| T2_WINDOW_OUT_TIME | 800 | cycles | T2 window end time |
| T3_WINDOW_IN_TIME | 300 | cycles | T3 window start time |
| T3_WINDOW_OUT_TIME | 1000 | cycles | T3 window end time |
| FSA_FLAG_DELAY | 10 | cycles | FSA flag transmission delay |
| CANARD_FLAG_DELAY | 20 | cycles | Canard flag transmission delay |
| CONTROL_FLAG_DELAY | 30 | cycles | Control flag transmission delay |
| GUID_START_FLAG_DELAY | 40 | cycles | Guidance start flag delay |

### 6.2 Threshold Configuration

| Parameter | Default Value | Unit | Description |
|-----------|---------------|------|-------------|
| ROLL_RATE_T1_THRESHOLD | 7.0 | rps | T1 roll rate threshold |
| ROLL_RATE_T2_THRESHOLD | 2.0 | rps | T2 roll rate threshold |
| CONFIRMATION_CYCLES | 3 | cycles | Required confirmation cycles |

---

## 7. Test Requirements

### 7.1 Unit Test Requirements
- Test all function return values
- Test all error conditions
- Test boundary conditions
- Test timing sequences

### 7.2 Integration Test Requirements
- Test complete flight sequence
- Test roll rate confirmation logic
- Test flag timing coordination
- Test error recovery

### 7.3 Performance Test Requirements
- Measure execution time
- Verify memory usage
- Test under maximum load conditions

---

## 8. Acceptance Criteria

### 8.1 Functional Acceptance
- [ ] All functional requirements implemented
- [ ] All test cases pass
- [ ] MISRA C compliance verified
- [ ] Code review completed

### 8.2 Performance Acceptance
- [ ] Execution time < 1ms
- [ ] Memory usage < 1KB
- [ ] No memory leaks detected
- [ ] Deterministic execution verified

---

## 9. Traceability Matrix

| Requirement ID | Test Case ID | Implementation | Status |
|----------------|--------------|----------------|---------|
| FR-001 | TC-001 | sequencerExecute() | Pending |
| FR-002 | TC-002 | mainClockCycles | Pending |
| FR-003 | TC-003 | processT0Logic() | Pending |
| ... | ... | ... | ... |

---

## 10. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025 | Ananthu Dev | Initial version |