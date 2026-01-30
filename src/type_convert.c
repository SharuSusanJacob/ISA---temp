/******************************************************************************
* ISA Flight Software
* @file type_convert.c
* @brief Type Conversion from LSB to Double Implementation
* @details Converts raw LSB values from IMU/GNSS hardware to double values
* @author Integration Team, Spacelabs
* @date 2025
* @version 1.0
*****************************************************************************/

#include "type_convert.h"
#include <stdint.h>
#include <stddef.h> /* For NULL */

/* ===== Accelerometer Conversion ===== */

void convert_accel_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                                 uint16_t offset_x, uint16_t offset_y, uint16_t offset_z)
{
    if (result == NULL || buffer == NULL)
    {
        return;
    }

    /* Convert LSB to int16_t (little-endian) then to double (ft/s²) */
    int16_t lsb_x = TO_INT16_LE(&buffer[offset_x]);
    int16_t lsb_y = TO_INT16_LE(&buffer[offset_y]);
    int16_t lsb_z = TO_INT16_LE(&buffer[offset_z]);

    result->x = (double)lsb_x * CONV_ACCEL_LSB_TO_FT_S2;
    result->y = (double)lsb_y * CONV_ACCEL_LSB_TO_FT_S2;
    result->z = (double)lsb_z * CONV_ACCEL_LSB_TO_FT_S2;
}

/* ===== Gyroscope Conversion ===== */

void convert_gyro_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                                uint16_t offset_x, uint16_t offset_y, uint16_t offset_z)
{
    if (result == NULL || buffer == NULL)
    {
        return;
    }

    /* Convert LSB to int16_t (little-endian) then to double (rad/s) */
    int16_t lsb_x = TO_INT16_LE(&buffer[offset_x]);
    int16_t lsb_y = TO_INT16_LE(&buffer[offset_y]);
    int16_t lsb_z = TO_INT16_LE(&buffer[offset_z]);

    result->x = (double)lsb_x * CONV_GYRO_LSB_TO_RAD_S;
    result->y = (double)lsb_y * CONV_GYRO_LSB_TO_RAD_S;
    result->z = (double)lsb_z * CONV_GYRO_LSB_TO_RAD_S;
}

/* ===== Magnetometer Conversion ===== */

void convert_mag_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                               uint16_t offset_x, uint16_t offset_y, uint16_t offset_z)
{
    if (result == NULL || buffer == NULL)
    {
        return;
    }

    /* Convert LSB to int16_t (little-endian) then to double (mG) */
    int16_t lsb_x = TO_INT16_LE(&buffer[offset_x]);
    int16_t lsb_y = TO_INT16_LE(&buffer[offset_y]);
    int16_t lsb_z = TO_INT16_LE(&buffer[offset_z]);

    result->x = (double)lsb_x * CONV_MAG_LSB_TO_MG;
    result->y = (double)lsb_y * CONV_MAG_LSB_TO_MG;
    result->z = (double)lsb_z * CONV_MAG_LSB_TO_MG;
}

/* ===== ECEF Position Conversion ===== */

void convert_ecef_position_lsb_to_double(double *result_x, double *result_y, double *result_z,
                                         const uint8_t *buffer,
                                         uint16_t offset_x, uint16_t offset_y, uint16_t offset_z)
{
    if (result_x == NULL || result_y == NULL || result_z == NULL || buffer == NULL)
    {
        return;
    }

    /* Convert LSB to int32_t (big-endian) then to double (meters) */
    int32_t lsb_x = TO_INT32_BE(&buffer[offset_x]);
    int32_t lsb_y = TO_INT32_BE(&buffer[offset_y]);
    int32_t lsb_z = TO_INT32_BE(&buffer[offset_z]);

    *result_x = (double)lsb_x * CONV_ECEF_POS_LSB_TO_M;
    *result_y = (double)lsb_y * CONV_ECEF_POS_LSB_TO_M;
    *result_z = (double)lsb_z * CONV_ECEF_POS_LSB_TO_M;
}

/* ===== ECEF Velocity Conversion ===== */

void convert_ecef_velocity_lsb_to_double(double *result_vx, double *result_vy, double *result_vz,
                                         const uint8_t *buffer,
                                         uint16_t offset_vx, uint16_t offset_vy, uint16_t offset_vz)
{
    if (result_vx == NULL || result_vy == NULL || result_vz == NULL || buffer == NULL)
    {
        return;
    }

    /* Convert LSB to int32_t (big-endian) then to double (m/s) */
    int32_t lsb_vx = TO_INT32_BE(&buffer[offset_vx]);
    int32_t lsb_vy = TO_INT32_BE(&buffer[offset_vy]);
    int32_t lsb_vz = TO_INT32_BE(&buffer[offset_vz]);

    *result_vx = (double)lsb_vx * CONV_ECEF_VEL_LSB_TO_M_S;
    *result_vy = (double)lsb_vy * CONV_ECEF_VEL_LSB_TO_M_S;
    *result_vz = (double)lsb_vz * CONV_ECEF_VEL_LSB_TO_M_S;
}

/* ===== Incremental Velocity Conversion ===== */

void convert_inc_vel_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                                   uint16_t offset_x, uint16_t offset_y, uint16_t offset_z)
{
    if (result == NULL || buffer == NULL)
    {
        return;
    }

    /* Convert LSB to int32_t (little-endian) then to double (ft/s) */
    int32_t lsb_x = TO_INT32_LE(&buffer[offset_x]);
    int32_t lsb_y = TO_INT32_LE(&buffer[offset_y]);
    int32_t lsb_z = TO_INT32_LE(&buffer[offset_z]);

    result->x = (double)lsb_x * CONV_INC_VEL_LSB_TO_FT_S;
    result->y = (double)lsb_y * CONV_INC_VEL_LSB_TO_FT_S;
    result->z = (double)lsb_z * CONV_INC_VEL_LSB_TO_FT_S;
}

/* ===== Incremental Angle Conversion ===== */

void convert_inc_angle_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                                     uint16_t offset_x, uint16_t offset_y, uint16_t offset_z)
{
    if (result == NULL || buffer == NULL)
    {
        return;
    }

    /* Convert LSB to int32_t (little-endian) then to double (radians) */
    int32_t lsb_x = TO_INT32_LE(&buffer[offset_x]);
    int32_t lsb_y = TO_INT32_LE(&buffer[offset_y]);
    int32_t lsb_z = TO_INT32_LE(&buffer[offset_z]);

    result->x = (double)lsb_x * CONV_INC_ANGLE_LSB_TO_RAD;
    result->y = (double)lsb_y * CONV_INC_ANGLE_LSB_TO_RAD;
    result->z = (double)lsb_z * CONV_INC_ANGLE_LSB_TO_RAD;
}
