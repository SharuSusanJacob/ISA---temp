/******************************************************************************
 * ISA Flight Software
 *
 * File: dap.c
 * Description: Digital Autopilot module implementation
 *****************************************************************************/
#define _POSIX_C_SOURCE 199309L

#include <math.h>
#include <stdbool.h>
#include "config.h"
#include "dap.h"
#include "vector3.h"

/* Module-level variables for rate derivative calculations */
static double rollIntegrator = 0.0;
static double LPF_out_Pitch = 0.0;
static double lag_out_pitch = 0.0;
static double theta = 0.0;
static double pitchIntegrator = 0.0;
static double LPF_out_Yaw = 0.0;
static double lag_out_yaw = 0.0;
static double psi = 0.0;
static double yawIntegrator = 0.0;
static double integratorValue = 0.0;

DAPOutput_t DAP_Execute(
    double rollAngle_rad,
    double accelerationY_mps2,
    double accelerationZ_mps2,
    double rollRate_radps,
    double pitchRate_radps,
    double yawRate_radps,
    double accelerationYCommand_mps2,
    double accelerationZCommand_mps2,
    DAPParameters_t dapParams,
    double timeStep_s,
    bool canardControlFlag
)
{
    DAPOutput_t deltar = { 0.0, 0.0, 0.0, 0.0 };
    DAPOutput_t out = { 0.0, 0.0, 0.0, 0.0 };
    PYOutput_t deltap = { 0.0, 0.0 };
    PYOutput_t deltay = { 0.0, 0.0 };

    if (canardControlFlag != false)
    {
        if (rollRate_radps != 0.0)
        {   
            deltar = ComputeDeltaRollCommand(rollAngle_rad, rollRate_radps, dapParams, timeStep_s);
            out = deltar;
        }
        else
        {
            deltap = ComputeDeltaPitchCommand(accelerationY_mps2, pitchRate_radps, accelerationYCommand_mps2, dapParams, timeStep_s);
            deltay = ComputeDeltaYawCommand(accelerationZ_mps2, yawRate_radps, accelerationZCommand_mps2, dapParams, timeStep_s);
            out.delta3_rad = deltap.delta1_rad;
            out.delta9_rad = deltap.delta2_rad;
            out.delta6_rad = deltay.delta1_rad;
            out.delta12_rad = deltay.delta2_rad;
        }
        return out;
    }
    else
    {
        out.delta3_rad = -0.024432809773124; /*before CONTROL_ON (is it 0 or -1.4 (trim))*/ 
        out.delta6_rad = -0.024432809773124; /*Trim value (rad)*/ 
        out.delta9_rad = -0.024432809773124;
        out.delta12_rad = -0.024432809773124;
        return out;
    }
}

DAPOutput_t ComputeDeltaRollCommand(double rollAngle_rad, double rollRate_radps, DAPParameters_t dapParams, double timeStep_s)
{
    double phiCommand = 0.0; /* assuming roll command is zero */
    double phiErr = phiCommand - rollAngle_rad;
    double phiLimit;
    double out;
    double deltaCommandRoll;
    double integratorValue = rollIntegrator;
    DAPOutput_t Out;

    /* Apply roll angle limits */
    if (phiErr < dapParams.phi_min_rad)
    {
        phiLimit = dapParams.phi_min_rad;
    }
    else if (phiErr > dapParams.phi_max_rad)
    {
        phiLimit = dapParams.phi_max_rad;
    }
    else
    {
        phiLimit = phiErr;
    }

    /* Calculate proportional term */
    out = phiLimit - (dapParams.kr_roll * rollRate_radps);

    /* Calculate final control command */
    deltaCommandRoll = (dapParams.kp_roll * out) + rollIntegrator ;

    /* Update integrator with anti-windup */
    integratorValue = integratorValue + ((out * dapParams.ki_roll) * timeStep_s);
    if (integratorValue > dapParams.integratorR_max_rad)
    {
        integratorValue = dapParams.integratorR_max_rad;
    }
    else if (integratorValue < dapParams.integratorR_min_rad)
    {
        integratorValue = dapParams.integratorR_min_rad;
    }
    rollIntegrator = integratorValue;

    Out.delta3_rad = deltaCommandRoll;
    Out.delta6_rad = deltaCommandRoll;
    Out.delta9_rad = deltaCommandRoll;
    Out.delta12_rad = deltaCommandRoll;
    return Out;
}

PYOutput_t ComputeDeltaPitchCommand(double accelerationY_mps2, double pitchRate_radps, double accelerationYCommand_mps2, DAPParameters_t dapParams, double timeStep_s)
{
    double accErrorPitch;
    double deltaCommandPitch;
    PYOutput_t outP;
    double thetaLimit;

    /* Safety check for time step - prevent division by zero */
    double safe_timeStep = timeStep_s;
    if (safe_timeStep < 1e-6f) {
        safe_timeStep = 1e-6;
    }
    accErrorPitch=accelerationYCommand_mps2-accelerationY_mps2;
    double LPF_outdot = (accErrorPitch*dapParams.K_LPF_Pitch-LPF_out_Pitch)*dapParams.wC_pitch;
    double lag_outdot = LPF_outdot+(LPF_out_Pitch*dapParams.pitch_a)-(lag_out_pitch*dapParams.pitch_b);
    double thetaErr = lag_out_pitch-theta;

    if (thetaErr < dapParams.theta_min_rad)
    {
        thetaLimit = dapParams.theta_min_rad;
    }
    else if (thetaErr > dapParams.theta_max_rad)
    {
        thetaLimit = dapParams.theta_max_rad;
    }
    else
    {
        thetaLimit = thetaErr;
    }

    /* Calculate proportional term */
    double out_Pitch = thetaLimit - (dapParams.kr_pitch * pitchRate_radps);

    /* Calculate final control command */
    deltaCommandPitch = (dapParams.kp_pitch * out_Pitch) + pitchIntegrator;
    LPF_out_Pitch += LPF_outdot * safe_timeStep;
    lag_out_pitch += lag_outdot * safe_timeStep;
    theta += pitchRate_radps * safe_timeStep;
   /* Update integrator with anti-windup */
    integratorValue = pitchIntegrator + ((out_Pitch * dapParams.ki_pitch) * timeStep_s);
    if (integratorValue > dapParams.integratorP_max_rad)
    {
        integratorValue = dapParams.integratorP_max_rad;
    }
    else if (integratorValue < dapParams.integratorP_min_rad)
    {
        integratorValue = dapParams.integratorP_min_rad;
    }
    pitchIntegrator = integratorValue;
    outP.delta1_rad = deltaCommandPitch;
    outP.delta2_rad = deltaCommandPitch;
    return outP;
}

PYOutput_t ComputeDeltaYawCommand(double accelerationZ_mps2, double yawRate_radps, double accelerationZCommand_mps2, DAPParameters_t dapParams, double timeStep_s)
{
    double accErrorYaw;
    double deltaCommandYaw;
    PYOutput_t outY;
    double psiLimit;

    /* Safety check for time step - prevent division by zero */
    double safe_timeStep = timeStep_s;
    if (safe_timeStep < 1e-6) {
        safe_timeStep = 1e-6;
    }
    accErrorYaw=accelerationZCommand_mps2-accelerationZ_mps2;
    double LPF_outdot = (accErrorYaw*dapParams.K_LPF_Yaw-LPF_out_Yaw)*dapParams.wC_yaw;
    double lag_outdot = LPF_outdot+(LPF_out_Yaw*dapParams.yaw_a)-(lag_out_yaw*dapParams.yaw_b);
    double psiErr = lag_out_yaw-psi;

    if (psiErr < dapParams.psi_min_rad)
    {
        psiLimit = dapParams.psi_min_rad;
    }
    else if (psiErr > dapParams.psi_max_rad)
    {
        psiLimit = dapParams.psi_max_rad;
    }
    else
    {
        psiLimit = psiErr;
    }

    /* Calculate proportional term */
    double out_yaw = psiLimit - (dapParams.kr_yaw * yawRate_radps);


    /* Calculate final control command */
    deltaCommandYaw = (dapParams.kp_yaw * out_yaw) + yawIntegrator;
    LPF_out_Yaw += LPF_outdot * safe_timeStep;
    lag_out_yaw += lag_outdot * safe_timeStep;
    psi += yawRate_radps * safe_timeStep;

    /* Update integrator with anti-windup */
    double integratorValue = yawIntegrator + ((out_yaw * dapParams.ki_yaw) * timeStep_s);
    if (integratorValue > dapParams.integratorY_max_rad)
    {
        integratorValue = dapParams.integratorY_max_rad;
    }
    else if (integratorValue < dapParams.integratorY_min_rad)
    {
        integratorValue = dapParams.integratorY_min_rad;
    }
    yawIntegrator = integratorValue;
    outY.delta1_rad = deltaCommandYaw;
    outY.delta2_rad = deltaCommandYaw;
    return outY;
}




