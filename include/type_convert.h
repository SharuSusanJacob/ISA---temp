/******************************************************************************
* ISA Flight Software
* @file type_convert.h
* @brief Type Conversion from LSB to Float
* @details Converts raw LSB values from IMU/GNSS hardware to float values
* @author Integration Team, Spacelabs
* @date 2025
* @version 1.0
*****************************************************************************/

#ifndef TYPE_CONVERT_H
#define TYPE_CONVERT_H

#include <stdint.h>
#include <stdbool.h>
#include "math_utils.h"

/* ===== Byte Order Conversion Macros ===== */

// Little-endian conversion
#define TO_INT16_LE(ptr) ((int16_t)((uint16_t)(ptr)[0] | ((uint16_t)(ptr)[1] << 8)))
#define TO_INT32_LE(ptr) ((int32_t)((uint32_t)(ptr)[0] | ((uint32_t)(ptr)[1] << 8) | ((uint32_t)(ptr)[2] << 16) | ((uint32_t)(ptr)[3] << 24)))
#define TO_UINT16_LE(ptr) ((uint16_t)((uint16_t)(ptr)[0] | ((uint16_t)(ptr)[1] << 8)))
#define TO_UINT32_LE(ptr) ((uint32_t)((uint32_t)(ptr)[0] | ((uint32_t)(ptr)[1] << 8) | ((uint32_t)(ptr)[2] << 16) | ((uint32_t)(ptr)[3] << 24)))

// Big-endian conversion
#define TO_INT16_BE(ptr) ((int16_t)(((uint16_t)(ptr)[0] << 8) | (uint16_t)(ptr)[1]))
#define TO_INT32_BE(ptr) ((int32_t)(((uint32_t)(ptr)[0] << 24) | ((uint32_t)(ptr)[1] << 16) | ((uint32_t)(ptr)[2] << 8) | (uint32_t)(ptr)[3]))
#define TO_UINT16_BE(ptr) ((uint16_t)(((uint16_t)(ptr)[0] << 8) | (uint16_t)(ptr)[1]))
#define TO_UINT32_BE(ptr) ((uint32_t)(((uint32_t)(ptr)[0] << 24) | ((uint32_t)(ptr)[1] << 16) | ((uint32_t)(ptr)[2] << 8) | (uint32_t)(ptr)[3]))

/* ===== Conversion Factor Constants ===== */

// ECEF Position (Signed int32, big-endian) -> meters
#define CONV_ECEF_POS_LSB_TO_M 1e-2 /* 0.01 */

// ECEF Velocity (Signed int32, big-endian) -> m/s
#define CONV_ECEF_VEL_LSB_TO_M_S 1e-2 /* 0.01 */

// Accelerometer (Signed int16, little-endian) -> ft/s²
#define CONV_ACCEL_LSB_TO_FT_S2 0.1

// Gyroscope (Signed int16, little-endian) -> rad/s
#define CONV_GYRO_LSB_TO_RAD_S 2.23705532e-3

// Magnetometer (Signed int16, little-endian) -> mG
// #define CONV_MAG_LSB_TO_MG 0.438404

#define CONV_MAG_LSB_TO_MG 1

// Incremental Velocity (Signed int32, little-endian) -> ft/s
#define CONV_INC_VEL_LSB_TO_FT_S 1.52587891e-8

// Incremental Angle (Signed int32, little-endian) -> rad
#define CONV_INC_ANGLE_LSB_TO_RAD 3.41347552e-10

/* ===== Conversion Function Declarations ===== */

/**
 * @brief Convert accelerometer LSB values to double (ft/s²)
 *
 * @param result Output Vector3_t with converted values (ft/s²)
 * @param buffer Pointer to uint8_t buffer containing LSB values (little-endian)
 * @param offset_x Byte offset for X-axis LSB (2 bytes)
 * @param offset_y Byte offset for Y-axis LSB (2 bytes)
 * @param offset_z Byte offset for Z-axis LSB (2 bytes)
 */
void convert_accel_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                                 uint16_t offset_x, uint16_t offset_y, uint16_t offset_z);

/**
 * @brief Convert gyroscope LSB values to double (rad/s)
 *
 * @param result Output Vector3_t with converted values (rad/s)
 * @param buffer Pointer to uint8_t buffer containing LSB values (little-endian)
 * @param offset_x Byte offset for X-axis LSB (2 bytes)
 * @param offset_y Byte offset for Y-axis LSB (2 bytes)
 * @param offset_z Byte offset for Z-axis LSB (2 bytes)
 */
void convert_gyro_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                                uint16_t offset_x, uint16_t offset_y, uint16_t offset_z);

/**
 * @brief Convert magnetometer LSB values to double (mG)
 *
 * @param result Output Vector3_t with converted values (mG)
 * @param buffer Pointer to uint8_t buffer containing LSB values (little-endian)
 * @param offset_x Byte offset for X-axis LSB (2 bytes)
 * @param offset_y Byte offset for Y-axis LSB (2 bytes)
 * @param offset_z Byte offset for Z-axis LSB (2 bytes)
 */
void convert_mag_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                               uint16_t offset_x, uint16_t offset_y, uint16_t offset_z);

/**
 * @brief Convert ECEF position LSB values to double (meters)
 *
 * @param result_x Output X position in meters
 * @param result_y Output Y position in meters
 * @param result_z Output Z position in meters
 * @param buffer Pointer to uint8_t buffer containing LSB values (big-endian)
 * @param offset_x Byte offset for X position LSB (4 bytes)
 * @param offset_y Byte offset for Y position LSB (4 bytes)
 * @param offset_z Byte offset for Z position LSB (4 bytes)
 */
void convert_ecef_position_lsb_to_double(double *result_x, double *result_y, double *result_z,
                                         const uint8_t *buffer,
                                         uint16_t offset_x, uint16_t offset_y, uint16_t offset_z);

/**
 * @brief Convert ECEF velocity LSB values to double (m/s)
 *
 * @param result_vx Output X velocity in m/s
 * @param result_vy Output Y velocity in m/s
 * @param result_vz Output Z velocity in m/s
 * @param buffer Pointer to uint8_t buffer containing LSB values (big-endian)
 * @param offset_vx Byte offset for X velocity LSB (4 bytes)
 * @param offset_vy Byte offset for Y velocity LSB (4 bytes)
 * @param offset_vz Byte offset for Z velocity LSB (4 bytes)
 */
void convert_ecef_velocity_lsb_to_double(double *result_vx, double *result_vy, double *result_vz,
                                         const uint8_t *buffer,
                                         uint16_t offset_vx, uint16_t offset_vy, uint16_t offset_vz);

/**
 * @brief Convert incremental velocity LSB values to double (ft/s)
 *
 * @param result Output Vector3_t with converted values (ft/s)
 * @param buffer Pointer to uint8_t buffer containing LSB values (little-endian)
 * @param offset_x Byte offset for X-axis LSB (4 bytes)
 * @param offset_y Byte offset for Y-axis LSB (4 bytes)
 * @param offset_z Byte offset for Z-axis LSB (4 bytes)
 */
void convert_inc_vel_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                                   uint16_t offset_x, uint16_t offset_y, uint16_t offset_z);

/**
 * @brief Convert incremental angle LSB values to double (radians)
 *
 * @param result Output Vector3_t with converted values (radians)
 * @param buffer Pointer to uint8_t buffer containing LSB values (little-endian)
 * @param offset_x Byte offset for X-axis LSB (4 bytes)
 * @param offset_y Byte offset for Y-axis LSB (4 bytes)
 * @param offset_z Byte offset for Z-axis LSB (4 bytes)
 */
void convert_inc_angle_lsb_to_double(Vector3_t *result, const uint8_t *buffer,
                                     uint16_t offset_x, uint16_t offset_y, uint16_t offset_z);

#endif /* TYPE_CONVERT_H */