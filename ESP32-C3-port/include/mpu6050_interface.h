#ifndef MPU6050_INTERFACE_H
#define MPU6050_INTERFACE_H

#include "config.h"
#include "pins.h"
#include "types.h"

// ============================================================================
// MPU6050 IMU INTERFACE MODULE
// ============================================================================
// Handles I2C communication with MPU6050 accelerometer & gyroscope
// Implements complementary filter for attitude estimation
// Fuses gyro (fast, drifts) + accel (slow, accurate) for stable angles
// ============================================================================

/**
 * @brief Initialize MPU6050 sensor via I2C
 *        - Configure I2C bus
 *        - Wake up MPU6050 (disable sleep mode)
 *        - Set accelerometer range (±2g)
 *        - Set gyroscope range (±250°/s)
 *        - Configure low-pass filters
 * @return 1 if initialized successfully, 0 if failed
 */
uint8_t mpu6050_init();

/**
 * @brief Read raw sensor data from MPU6050
 *        - Read 14 bytes: 6 accel + 2 temp + 6 gyro
 *        - Convert to g and degrees/second
 *        - Apply axis mapping and orientation
 * @param imu Pointer to IMU_Data struct to fill
 * @return 1 if read successful, 0 if I2C error
 */
uint8_t mpu6050_read_raw(IMU_Data *imu);

/**
 * @brief Compute attitude (roll, pitch, yaw) using complementary filter
 *        - Fuses accelerometer (slow, stable) with gyroscope (fast, drifts)
 *        - Gyro provides fast response, accel corrects drift
 *        - Formula: angle_new = GYR_WEIGHT * (angle_old + gyro*dt) + (1-GYR_WEIGHT) * accel_angle
 *        - Output: angles in 0.1° units (180 = 18°)
 * @param imu Pointer to IMU_Data with current sensor readings
 * @param att Pointer to Attitude struct to fill with computed angles
 * @param dt Delta time since last call (seconds, typically ~0.0028s)
 */
void mpu6050_compute_attitude(const IMU_Data *imu, Attitude *att, float dt);

/**
 * @brief Get roll angle from last computation
 * @return Roll angle in 0.1° units (-1800 to 1800 = -180° to 180°)
 */
int16_t mpu6050_get_roll();

/**
 * @brief Get pitch angle from last computation
 * @return Pitch angle in 0.1° units (-1800 to 1800 = -180° to 180°)
 */
int16_t mpu6050_get_pitch();

/**
 * @brief Get yaw heading from last computation
 * @return Yaw heading in degrees (0-360)
 */
uint16_t mpu6050_get_yaw();

/**
 * @brief Calibrate gyroscope offset (run on startup with drone level)
 *        - Averages 100 gyro readings to find bias
 *        - Removes offset from future readings
 *        - Takes ~100-200ms
 */
void mpu6050_calibrate_gyro();

/**
 * @brief Calibrate accelerometer for 1G reference
 *        - Used to correct sensitivity drift
 *        - Run with drone level
 */
void mpu6050_calibrate_accel();

/**
 * @brief Reset attitude estimation (set angles to 0)
 */
void mpu6050_reset_attitude();

/**
 * @brief Get current IMU health status
 * @return 1 if healthy, 0 if errors detected
 */
uint8_t mpu6050_is_healthy();

#endif // MPU6050_INTERFACE_H
