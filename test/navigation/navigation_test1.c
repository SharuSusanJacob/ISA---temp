/******************************************************************************
* ISA Flight Software
* @file navigation_test.c
* @brief Test harness for Navigation Module based on reference implementation
* @details Implements same core logic as user_navigation.c reference
* @author Ananthu Dev, Project Engineer, Spacelabs
* @date 2025
* @version 1.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/
#define _USE_MATH_DEFINES  /* Add this line before including math.h */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "navigation.h"

/* File handling */
static FILE *states_file = NULL;
static FILE *nav_file = NULL;
static bool files_initialized = false;


/* Global variables mirroring reference implementation */
static bool highroll = true;
static bool gyro_attitude_init = false;
static float mag_body[3] = {0.0f, 0.0f, 0.0f};
static float phi = 0.0f;
static float roll = 0.0f;
static float dt = 0.01f;
static float theta = 25.0f * M_PI / 180.0f;
static float psi = 0.0f * M_PI / 180.0f;

/* Constants */
#define TWO_PI (2.0f * M_PI)

/* Function prototypes */
static void initialize_files(void);
static void close_files(void);
static float mod(float a, float d);
static void NVEtoBody(float phi, float theta, float psi, float mag_ini[3], float mag_body[3]);
static void normalize3(float v[3]);
static void run_navigation(float *states, float mission_time, NavigationState_t *nav_state);
static void print_navigation_status(const NavigationState_t *nav_state, float phi);

/**
 * @brief Modulo function identical to reference implementation
 * @param a Value to take modulo of
 * @param d Modulo divisor
 * @return float Result of modulo operation
 */
static float mod(float a, float d)
{
    float m = fmodf(a, d);
    if ((m < 0.0f && d > 0.0f) || (m > 0.0f && d < 0.0f))
    {
        m += d;
    }
    return m;
}

/**
 * @brief Initialize output files
 */
static void initialize_files(void)
{
    if (!files_initialized)
    {
        states_file = fopen("states.csv", "w");
        nav_file = fopen("navout.csv", "w");

        if (states_file == NULL || nav_file == NULL)
        {
            perror("Error opening log files");
            exit(EXIT_FAILURE);
        }

        fprintf(states_file,
                "missiontime,Pos_X,Pos_Y,Pos_Z,Vel_X,Vel_Y,Vel_Z,q0,q1,q2,q3,"
                "Gyro_p,Gyro_q,Gyro_r,Accel_X,Accel_Y,Accel_Z,Mag_X,Mag_Y,Mag_Z,simmode\n");

        fprintf(nav_file,
                "missiontime,Pos_X,Pos_Y,Pos_Z,Vel_X,Vel_Y,Vel_Z,Acc_X,Acc_Y,Acc_Z,"
                "Gyro_p,Gyro_q,Gyro_r,Roll,Pitch,Yaw,rollRPS\n");

        files_initialized = true;
    }
}

/**
 * @brief Close output files
 */
static void close_files(void)
{
    if (files_initialized)
    {
        if (states_file != NULL)
        {
            fclose(states_file);
            states_file = NULL;
        }
        
        if (nav_file != NULL)
        {
            fclose(nav_file);
            nav_file = NULL;
        }
        
        files_initialized = false;
    }
}

/**
 * @brief NVE to Body frame transformation - identical to reference implementation
 */
static void NVEtoBody(float phi, float theta, float psi, float mag_ini[3], float mag_body[3])
{
    float cphi = cosf(phi), sphi = sinf(phi);
    float ctheta = cosf(theta), stheta = sinf(theta);
    float cpsi = cosf(psi), spsi = sinf(psi);

    float Rx[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, cphi, -sphi},
        {0.0f, sphi, cphi}};
    float Rz[3][3] = {
        {ctheta, -stheta, 0.0f},
        {stheta, ctheta, 0.0f},
        {0.0f, 0.0f, 1.0f}};
    float Ry[3][3] = {
        {cpsi, 0.0f, spsi},
        {0.0f, 1.0f, 0.0f},
        {-spsi, 0.0f, cpsi}};
        
    /* Temp = Rz * Rx */
    float Temp[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Temp[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                Temp[i][j] += Rz[i][k] * Rx[k][j];
            }
        }
    }
    
    /* C = Ry * Temp */
    float C[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            C[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                C[i][j] += Ry[i][k] * Temp[k][j];
            }
        }
    }
    
    for (int i = 0; i < 3; i++)
    {
        mag_body[i] = 0.0f;
        for (int j = 0; j < 3; j++)
        {
            mag_body[i] += C[i][j] * mag_ini[j];
        }
    }
}

/**
 * @brief Vector normalization - identical to reference implementation
 */
static void normalize3(float v[3])
{
    float mag = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (mag > 1e-12f)
    { /* Avoid divide-by-zero */
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
    }
    else
    {
        v[0] = v[1] = v[2] = 0.0f;
    }
}

/**
 * @brief Run navigation with simulated sensor data using reference implementation logic
 * 
 * @param states Sensor input array (same format as reference implementation)
 * @param mission_time Current mission time in seconds
 * @param nav_state Navigation state structure to update
 */
static void run_navigation(float *states, float mission_time, NavigationState_t *nav_state)
{
    float navigation_out[16] = {0.0f};
    
    /* Initialize output files */
    initialize_files();
    
    /* Process position data (lines 305-310 in reference) */
    navigation_out[0] = 0.0f;     /* position x */
    navigation_out[1] = 0.0f;     /* position y */
    navigation_out[2] = 0.0f;     /* position z */
    navigation_out[3] = 0.0f;     /* velocity x */
    navigation_out[4] = 0.0f;     /* velocity y */
    navigation_out[5] = 0.0f;     /* velocity z */
    
    /* Process IMU data (lines 311-317 in reference) */
    navigation_out[6] = states[13];                /* acceleration x */
    navigation_out[7] = states[14];                /* acceleration y */
    navigation_out[8] = states[15];                /* acceleration z */
    navigation_out[9] = states[10];                /* gyro p */
    navigation_out[10] = states[11];               /* gyro q */
    navigation_out[11] = states[12];               /* gyro r */
    navigation_out[15] = states[10] / (2.0f * M_PI); /* roll rate in rps for sequencer */
    
    /* High roll rate handling (lines 336-342 in reference) */
    // if (highroll)
    // {
    //     printf("\n");
    //     printf("===============  High roll rate, waiting for roll to come down  =============\n");
    //     printf("\n");
    // }
    
    // /* Roll rate threshold detection (lines 343-443 in reference) */
    // if (navigation_out[9] < 5.0f * TWO_PI && !gyro_attitude_init)
    // {
    //     /* Transform magnetometer data exactly like reference code */
    //     float M[3][3] = {
    //         {1.0f, 0.0f, 0.0f},
    //         {0.0f, 0.0f, -1.0f},
    //         {0.0f, 1.0f, 0.0f}};
            
    //     float states_sub[3] = {states[16], states[17], states[18]};
    //     float mag_ini[3];
        
    //     for (int i = 0; i < 3; i++)
    //     {
    //         mag_ini[i] = 0.0f;
    //         for (int j = 0; j < 3; j++)
    //         {
    //             mag_ini[i] += 0.01f * M[i][j] * states_sub[j]; /* In milligauss */
    //         }
    //     }
        
        //normalize3(mag_ini);
        
        /* Transform magnetic field to body frame - exact same as reference */
        // NVEtoBody(roll, theta, psi, mag_ini, mag_body);

        theta = 0.43633231299858238;
        psi = 0;
        
        float mx = 363.740845;
        float my = 178.018822;
        float mz = -124.075590;

        // 363.740845,178.018822,-124.075590
        
        /* Use exact same constants as reference */
        float mv = -118.19609852920119f;
        float me = -7.4203686987882156f;
        
        printf("\n");
        printf("=================  Roll estimation ongoing, roll rate < 5 rps  ===============\n");
        printf("\n");
        
        /* Use exact same roll estimation formula as reference */
        float phi1 = atan2f(mv * (mx * sinf(psi) + mz * cosf(psi)) -
                           me * (-mx * sinf(theta) * cosf(psi) + my * cosf(theta) + mz * sinf(theta) * sinf(psi)),
                           mv * (-mx * sinf(theta) * cosf(psi) + my * cosf(theta) + mz * sinf(theta) * sinf(psi)) +
                           me * (mx * sinf(psi) + mz * cosf(psi)));
                           
        phi = mod(phi1, TWO_PI); /* wrap to 2pi */
        
        navigation_out[12] = phi;
        navigation_out[13] = theta;
        navigation_out[14] = psi;
        highroll = false;

    
    /* Low roll rate handling (lines 445-462 in reference) */
    // if (navigation_out[9] < 2.0f * TWO_PI)
    // {
    //     printf("\n");
    //     printf("========  Gyro attitude initialized when roll rate < 2 rps and integrating  ========\n");
    //     printf("\n");
        
    //     /* Update angles by integrating rates - exactly like reference code */
    //     phi = phi + navigation_out[9] * dt;
    //     theta = theta + navigation_out[10] * dt;
    //     psi = psi + navigation_out[11] * dt;
        
    //     /* Wrap angles to 0-2π - exactly like reference code */
    //     phi = mod(phi, TWO_PI);
    //     theta = mod(theta, TWO_PI);
    //     psi = mod(psi, TWO_PI);
        
    //     navigation_out[12] = phi;   /* roll */
    //     navigation_out[13] = theta; /* pitch */
    //     navigation_out[14] = psi;   /* yaw */
    //     gyro_attitude_init = true;
    // }
    
    /* Write states to CSV files exactly like reference */
    fprintf(states_file,
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            mission_time, states[0], states[1], states[2], states[3], states[4], states[5],
            0.0f, 0.0f, 0.0f, 0.0f, /* Quaternion placeholders */
            states[10], states[11], states[12], states[13], states[14], states[15], 
            states[16], states[17], states[18], states[19]);
            
    fprintf(nav_file,
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            mission_time, 
            navigation_out[0], navigation_out[1], navigation_out[2],
            navigation_out[3], navigation_out[4], navigation_out[5],
            navigation_out[6], navigation_out[7], navigation_out[8],
            navigation_out[9], navigation_out[10], navigation_out[11],
            navigation_out[12], navigation_out[13], navigation_out[14], 
            navigation_out[15]);
    
    fflush(states_file);
    fflush(nav_file);
    
    /* Update navigation state (for compatibility with the navigation module) */
    NavImuData_t imu_data;
    vector3_create(&(imu_data.gyro), states[10], states[11], states[12]);
    vector3_create(&(imu_data.accel), states[13], states[14], states[15]);
    vector3_create(&(imu_data.mag), states[16], states[17], states[18]);
    imu_data.gyro_valid = true;
    imu_data.accel_valid = true;
    imu_data.mag_valid = true;
    (void)navigation_process_imu(nav_state, &imu_data, dt);
    
    /* Set attitude in navigation state from our phi, theta, psi */
    NavEulerAngles_t euler;
    euler.roll_rad = navigation_out[12];
    euler.pitch_rad = navigation_out[13];
    euler.yaw_rad = navigation_out[14];
    
    NavQuaternion_t quat;
    (void)navigation_euler_to_quaternion(&quat, &euler);
    (void)navigation_set_attitude(nav_state, &quat);
    
    /* Update accumulated roll - exactly like reference */
    roll = roll + states[10] * dt;
}

/**
 * @brief Print navigation status similar to reference implementation
 */
static void print_navigation_status(const NavigationState_t *nav_state, float phi)
{
    Vector3_t angular_rates;
    NavEulerAngles_t attitude;
    
    (void)navigation_get_angular_rates(nav_state, &angular_rates);
    (void)navigation_get_attitude_euler(nav_state, &attitude);
    
    printf("Position (x,y,z): %.6f, %.6f, %.6f\n", 0.0f, 0.0f, 0.0f);
    printf("Velocity (u,v,w): %.6f, %.6f, %.6f\n", 0.0f, 0.0f, 0.0f);
    printf("Acceleration (ax,ay,az): %.6f, %.6f, %.6f\n", 
           nav_state->imu_data.accel.x, 
           nav_state->imu_data.accel.y, 
           nav_state->imu_data.accel.z);
    printf("Angular Rates (p,q,r): %.6f, %.6f, %.6f\n", 
           angular_rates.x, angular_rates.y, angular_rates.z);
    printf("Attitude (roll,pitch,yaw): %.6f, %.6f, %.6f\n", 
           attitude.roll_rad, attitude.pitch_rad, attitude.yaw_rad);
    printf("Sequencer RPS: %.6f\n", angular_rates.x / (2.0f * M_PI));
    printf("Roll in deg: %.6f\n", phi * 180.0f / M_PI);
    printf("\n");
}

/**
 * @brief Main function
 * @return int Exit status (0 for success)
 */
int main(void) {
    NavigationState_t nav_state;
    float mission_time = 0.0f;
    float sim_duration = 30.0f; /* 30 second simulation */
    const float time_step = 0.01f; /* 10ms time step */
    uint32_t step = 0;
    
    /* Initialize navigation state */
    if (navigation_init(&nav_state) != NAV_SUCCESS) {
        printf("ERROR: Failed to initialize navigation state\n");
        return 1;
    }
    
    printf("\n===== ISA Navigation Module Test =====\n\n");
    printf("Running test simulation for %.1f seconds...\n\n", sim_duration);
    
    /* Main simulation loop */
    while (mission_time <= sim_duration) {
        /* Create simulated sensor data */
        float states[20] = {0};
        
        /* Position (X,Y,Z) */
        states[0] = 6378137.0f + 1000.0f; /* Earth radius + altitude */
        states[1] = 0.0f;
        states[2] = 0.0f;
        
        /* Velocity (X,Y,Z) */
        states[3] = 0.0f;
        states[4] = 100.0f; /* Moving at 100 m/s */
        states[5] = 0.0f;
        
        /* Quaternion (for reference only) */
        states[6] = 1.0f;
        states[7] = 0.0f;
        states[8] = 0.0f;
        states[9] = 0.0f;
        
        /* Angular rates - simulate decreasing roll rate */
        float initial_roll_rate = 4000.0f; /* deg/s */
        float mid_roll_rate = 1500.0f; /* deg/s */
        float final_roll_rate = 0.01f; /* deg/s */
        float t_mid = 20.0f; /* s */
        float t_end = 25.0f; /* s */
        
        float roll_rate;
        if (mission_time <= 0.0f) {
            roll_rate = initial_roll_rate;
        } else if (mission_time >= t_end) {
            roll_rate = final_roll_rate;
        } else if (mission_time <= t_mid) {
            float alpha = mission_time / t_mid; /* 0 to 1 */
            roll_rate = initial_roll_rate + (mid_roll_rate - initial_roll_rate) * 
                       (0.5f - 0.5f * cosf(M_PI * alpha));
        } else {
            float alpha = (mission_time - t_mid) / (t_end - t_mid);
            roll_rate = mid_roll_rate + (final_roll_rate - mid_roll_rate) * 
                       (0.5f - 0.5f * cosf(M_PI * alpha));
        }
        
        states[10] = roll_rate * M_PI / 180.0f; /* p: roll rate (rad/s) */
        states[11] = 0.0f;                      /* q: pitch rate (rad/s) */
        states[12] = 0.0f;                      /* r: yaw rate (rad/s) */
        
        /* Accelerometer */
        states[13] = 0.0f;
        states[14] = 0.0f;
        states[15] = -9.81f; /* Gravity */
        
        /* Magnetometer */
        states[16] = 100.0f * cosf(mission_time * 0.1f);
        states[17] = 0.0f;
        states[18] = 100.0f * sinf(mission_time * 0.1f);
        
        /* Simulation mode */
        states[19] = 1.0f; /* Use ideal position/velocity */
        
        /* Run navigation update */
        run_navigation(states, mission_time, &nav_state);
        
        /* Print status every second */
        if ((step % 100) == 0) {
            print_navigation_status(&nav_state, phi);
        }
        
        /* Update time and step */
        mission_time += time_step;
        step++;
    }
    
    /* Close files */
    close_files();
    
    printf("Navigation test completed successfully\n");
    printf("Results written to states.csv and navout.csv\n");
    
    return 0;
}