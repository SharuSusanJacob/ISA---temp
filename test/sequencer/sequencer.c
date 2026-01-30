/******************************************************************************
* ISA Flight Software
* @file sequencer.c
* @brief Projectile Flight Computer Sequencer Module
* @details Manages mission phases and state transitions for precision-guided projectile
* @author Ananthu Dev, Project Engineer, Spacelabs.
* @date 2025
* @version 2.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#include "sequencer.h"
#include <string.h> //for memset

SequencerError_t sequencerInit(SequencerState_t* state)
{
    // FLight software rule: Always validate input parameters first
    if (state == NULL) {
        return SEQ_ERROR_INVALID_PARAM;
    }

    //Intialize all state to zero (safest starting condtion)
    memset(state, 0, sizeof(SequencerState_t));

    // Explicitly set initial values eventhough memset made them 0
    //this make the code self-documenting
    state->isT0Set = false;
    state->isT1Set = false;
    state->isT2Set = false;
    state->isT3Set = false;

    // Intialize all flags to false
    state->isFsaActivateFlagSent = false;
    state->isCanardDeployFlagSent = false;
    state->isCanardControlFlagSent = false;
    state->isGuidStartFlagSent = false;

    // Initialize counters and timers to zero
    state->mainClockCycles = 0U;
    state->fsaActivateFlagSendTime = 0U;
    state->canardDeployFlagSendTime = 0U;
    state->canardControlFlagSendTime = 0U;
    state->guidStartFlagSendTime = 0U;
    state->t1RollRateCount = 0U;
    state->t2RollRateCount = 0U;

    // G-switch starts inactive
    state->isOBCReset = false;

    return SEQ_SUCCESS;
}

SequencerError_t sequencerSetOBCReset(SequencerState_t* state, bool isActive)
{
    // Parameter validation
    if (state == NULL) {
        return SEQ_ERROR_INVALID_PARAM;
    }

    // Set G-switch state
    state->isOBCReset = isActive;

    if (isActive) {
        //g switch activation triggers the start of timing
        // This is T=0 moment  - reset the OBC
        state->mainClockCycles = 0U;

        // Set T0 phase immediately when G -switch activates
        state->isT0Set = true;

        // Reset all confirmation counters
        state->t1RollRateCount = 0U;
        state->t2RollRateCount = 0U;
    }
    // it doesn't anything when G-switch goes inactive
    // Oncel launched, it stays launched    
    return SEQ_SUCCESS;
}

// Helper function to check the roll rate condition
static bool isRollRateOkForT1(float rollRateFp)
{
    // Roll rate threshold for T1 is 7.0 rps (70 in fixed point)
    return (rollRateFp <= SEQ_ROLL_RATE_T1_THRESHOLD);
}

static SequencerError_t processT1Logic(SequencerState_t* state,
    float rollRateFp,
    SequencerOutput_t* output)
{
    //First check if T1 window is out (T > T1WindowOut)
    if (state->mainClockCycles > SEQ_T1_WINDOW_OUT_TIME) {
        // Window out - Set T1 Immediately
        state->isT1Set = true;
        output->setT1 = true;
        state->t1SetTime = state->mainClockCycles; // Record T1 recordtime

        // Schedule the canard deploy flag if not already sent
        if (!state->isCanardDeployFlagSent) {
            state->canardDeployFlagSendTime = state->mainClockCycles + SEQ_CANARD_DEPLOY_FLAG_DELAY;
        }

        return SEQ_SUCCESS;
    }
    // Check for T1 window sensing
    if (state->mainClockCycles > SEQ_T1_WINDOW_IN_TIME) {
        // Check if the roll rate <= 7rps conditions are met
        if (isRollRateOkForT1(rollRateFp)) {
            //if the roll rate is good increment the confirmation counter
            state->t1RollRateCount++;

            // Check if the condition has been met for the required number of cycles
            if (state->t1RollRateCount >= SEQ_CONFIRMATION_CYCLES) {
                //  Set T1
                state->isT1Set = true;
                output->setT1 = true;
                state->t1SetTime = state->mainClockCycles;

                // Schedule the canard deploy flag if not already sent
                if (!state->isCanardDeployFlagSent) {
                    state->canardDeployFlagSendTime = state->mainClockCycles + SEQ_CANARD_DEPLOY_FLAG_DELAY;
                }

                return SEQ_SUCCESS;
            }
        }
        else {
            // Roll rate is not within the threshold - reset the counter
            state->t1RollRateCount = 0U;
        }
    }
    return SEQ_SUCCESS;
}

// T2 logic starts

// Helper function to check the roll rate condition for T2
static bool isRollRateOkForT2(float rollRateFp)
{
    return (rollRateFp <= SEQ_ROLL_RATE_T2_THRESHOLD);
}
static SequencerError_t processT2Logic(SequencerState_t* state,
    float rollRateFp,
    SequencerOutput_t* output)
{
    // Check if it's time to send the canard deploy flag
    if (!state->isCanardDeployFlagSent && (state->mainClockCycles > state->canardDeployFlagSendTime)) {
        output->canardDeployFlag = true;
        state->isCanardDeployFlagSent = true;
    }

    // handle the time out conditions
    if (state->mainClockCycles > (state->t1SetTime + SEQ_T2_WINDOW_OUT_TIME)) {
        // Window expired - set t2 immediately
        state->isT2Set = true;
        output->setT2 = true;
        state->t2SetTime = state->mainClockCycles; // Record T2 set time
        // Schedule the Canard control flag
        state->canardControlFlagSendTime = state->mainClockCycles + SEQ_CANARD_CONTROL_ON_FLAG_DELAY;
        return SEQ_SUCCESS;
    }
    //check the T2 detection window
    if (state->mainClockCycles > (state->t1SetTime + SEQ_T2_WINDOW_IN_TIME)) {
        // Check if the roll rate is below the T2 threshold(<= 2 rps).
        if (isRollRateOkForT2(rollRateFp)) {
            state->t2RollRateCount++;
            //check if the roll rate persists for 3 consecutive cycles.
            if (state->t2RollRateCount >= SEQ_CONFIRMATION_CYCLES) {
                // Set T2
                state->isT2Set = true;
                output->setT2 = true;
                state->t2SetTime = state->mainClockCycles; // Record T2 set time
                // Schedule the canard control flag,
                state->canardControlFlagSendTime = state->mainClockCycles + SEQ_CANARD_CONTROL_ON_FLAG_DELAY;
                return SEQ_SUCCESS;
            }
        }
        else {
            //Roll rate is not within the threshold - reset the counter .
            state->t2RollRateCount = 0U;
        }
    }
    return SEQ_SUCCESS;
}


//T2 logic ends


//T3 logic starts
// Implementation of T3 logic
// Note: tGo is in seconds (float), not cycles
static SequencerError_t processT3Logic(SequencerState_t* state,
    float tGo,
    SequencerOutput_t* output)
{
    // First check if guidance start flag is already sent
    if (state->isGuidStartFlagSent) {
        // Guidance start flag has already been sent
        // Check if FSA flag is already sent
        if (state->isFsaActivateFlagSent) {
            // Both guidance and FSA flags are sent
            // Check if T3 window is out
            if (state->mainClockCycles > (state->t2SetTime + SEQ_T3_WINDOW_OUT_TIME)) {
                // T3 window is out, set T3 if not already set
                if (!state->isT3Set) {
                    state->isT3Set = true;
                    output->setT3 = true;
                    state->t3SetTime = state->mainClockCycles;
                    output->enableProximitySensor = true;
                }
            }
            else {
                // T3 window is not out, check if T3 window in
                if (state->mainClockCycles > (state->t2SetTime + SEQ_T3_WINDOW_IN_TIME)) {
                    // T3 window is in, check tGo from guidance
                    if (tGo < SEQ_T_PROXIMITY) {
                        // tGo is less than time to enable proximity sensor
                        state->isT3Set = true;
                        output->setT3 = true;
                        state->t3SetTime = state->mainClockCycles;
                        output->enableProximitySensor = true;
                    }
                    // If tGo not less than threshold, exit
                }
                // If not T3 window in, exit
            }
        }
        else {
            // FSA flag not sent yet
            // Check if it's time to send the FSA flag
            if (state->mainClockCycles > state->fsaActivateFlagSendTime) {
                output->fsaActivateFlag = true;
                state->isFsaActivateFlagSent = true;
            }
        }
    }
    else {
        // Guidance start flag has not been sent
        // Check if canard control flag is already sent
        if (state->isCanardControlFlagSent) {
            // Control flag has been sent
            // First check if FSA flag is already sent
            if (state->isFsaActivateFlagSent) {
                // FSA flag already sent
                // Now check if it's time to send the Guidance Start Flag
                if (state->mainClockCycles > state->guidStartFlagSendTime) {
                    output->sendGuidStartFlag = true;
                    state->isGuidStartFlagSent = true;
                    return SEQ_SUCCESS;
                }
                // Not time for guidance flag yet, exit and wait for next cycle
                return SEQ_SUCCESS;
            }
            else {
                // FSA flag not sent yet
                // Check if it's time to send FSA flag
                if (state->mainClockCycles > state->fsaActivateFlagSendTime) {
                    output->fsaActivateFlag = true;
                    state->isFsaActivateFlagSent = true;
                }

                // Check if it's time to send the Guidance Start Flag regardless of FSA flag
                if (state->mainClockCycles > state->guidStartFlagSendTime) {
                    output->sendGuidStartFlag = true;
                    state->isGuidStartFlagSent = true;
                    return SEQ_SUCCESS;
                }
                return SEQ_SUCCESS;
            }
        }
        else {
            // Canard control flag not sent yet
            // Check if it's time to send the canard control flag
            if (state->mainClockCycles > state->canardControlFlagSendTime) {
                output->canardControlFlag = true;
                state->isCanardControlFlagSent = true;

                // Schedule the flags to be sent
                state->fsaActivateFlagSendTime = state->mainClockCycles + SEQ_FSA_FLAG_DELAY;
                state->guidStartFlagSendTime = state->mainClockCycles + SEQ_GUID_START_FLAG_DELAY;
            }
            // If not time, we just return and check again next cycle

            // Check if T2 window is out
            if (state->mainClockCycles > (state->t1SetTime + SEQ_T2_WINDOW_OUT_TIME)) {
                // T2 window out, set T2 immediately if not already set
                if (!state->isT2Set) {
                    state->isT2Set = true;
                    output->setT2 = true;
                    state->t2SetTime = state->mainClockCycles;
                }
            }
            else {
                // T2 window not out, check if T2 window in
                if (state->mainClockCycles > (state->t1SetTime + SEQ_T2_WINDOW_IN_TIME)) {
                    // T2 window in logic
                    // In processT3Logic we already know T2 conditions were met
                    // This is just for completeness
                }
            }
        }
    }

    return SEQ_SUCCESS;
}
//T3 logic ends

//Window priority
SequencerError_t sequencerExecute(SequencerState_t* state,
    float rollRateFp,
    float tGo,
    SequencerOutput_t* output)
{
    // Parameter validation first (critical for flight software)
    if ((state == NULL) || (output == NULL)) {
        return SEQ_ERROR_INVALID_PARAM;
        // Early exit on error
    }
    //clear all outputs first (safe starting state)
    memset(output, 0, sizeof(SequencerOutput_t)); // Safe starting state

    //Increment main clock if G-switch is active
    if (state->isOBCReset) {
        state->mainClockCycles++;
    }
    // Priority-based logic for ISA
    // check T3 first, then t2, then t1, then t0

    if (state->isT3Set) {
        //T3 is set - this is the end, sequencer exits
        return SEQ_SUCCESS;
    }
    if (state->isT2Set) {
        //the porjectile in T2 phase - check for t3 condtions
        return processT3Logic(state, tGo, output);
    }
    if (state->isT1Set) {
        // the projectile is in T1 phase - check for T2 conditions
        return processT2Logic(state, rollRateFp, output);
    }

    if (state->isT0Set) {
        // the projectile is in T0 phase, check for T1 conditions
        return processT1Logic(state, rollRateFp, output);
    }

    //If we get here, no phases are set (should not happen after G-switch)
    return SEQ_ERROR_INVALID_STATE;
}









