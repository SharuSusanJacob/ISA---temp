/******************************************************************************
 * ISA Flight Software
 * @file pitch_limit.c
 * @brief Pitch Acceleration Limiter Implementation
 * @details Implements Mach-dependent pitch acceleration limits with interpolation
 * @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation
 *****************************************************************************/

#include "pitch_limit.h"

/* Lookup table data points */
#define NUM_MACH_POINTS 6

static const double mach_data[NUM_MACH_POINTS] = {
    0.4, 0.6, 0.8, 0.95, 1.05, 1.1};

static const double acc_max_data[NUM_MACH_POINTS] = {
    0.5, 2.0, 3.0, 10.0, 12.0, 15.0};

static const double acc_min_data[NUM_MACH_POINTS] = {
    -0.3, -1.0, -2.0, -8.0, -10.0, -13.0};

double pitch_limit(double mach, double input_acc)
{
    double acc_max;
    double acc_min;
    double output_acc;
    int idx_below;
    int idx_above;
    double mach_below;
    double mach_above;
    double t;
    int i;

    /* Handle cases outside the table range */
    if (mach <= mach_data[0])
    {
        /* Below minimum Mach in table */
        acc_max = acc_max_data[0];
        acc_min = acc_min_data[0];
    }
    else if (mach >= mach_data[NUM_MACH_POINTS - 1])
    {
        /* Above maximum Mach in table */
        acc_max = acc_max_data[NUM_MACH_POINTS - 1];
        acc_min = acc_min_data[NUM_MACH_POINTS - 1];
    }
    else
    {
        /* Find bracketing indices for interpolation */
        idx_below = 0;
        idx_above = NUM_MACH_POINTS - 1;

        for (i = 0; i < NUM_MACH_POINTS; i++)
        {
            if (mach_data[i] <= mach)
            {
                idx_below = i;
            }
            if (mach_data[i] >= mach && idx_above == NUM_MACH_POINTS - 1)
            {
                idx_above = i;
            }
        }

        /* Check if exact match */
        if (idx_below == idx_above)
        {
            acc_max = acc_max_data[idx_below];
            acc_min = acc_min_data[idx_below];
        }
        else
        {
            /* Linear interpolation */
            mach_below = mach_data[idx_below];
            mach_above = mach_data[idx_above];

            /* Interpolation parameter */
            t = (mach - mach_below) / (mach_above - mach_below);

            /* Interpolate max and min limits */
            acc_max = acc_max_data[idx_below] + t * (acc_max_data[idx_above] - acc_max_data[idx_below]);
            acc_min = acc_min_data[idx_below] + t * (acc_min_data[idx_above] - acc_min_data[idx_below]);
        }
    }

    /* Apply limits */
    if (input_acc > acc_max)
    {
        output_acc = acc_max;
    }
    else if (input_acc < acc_min)
    {
        output_acc = acc_min;
    }
    else
    {
        output_acc = input_acc;
    }

    return output_acc;
}
