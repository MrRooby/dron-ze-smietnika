#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"
#include "pins.h"
#include "types.h"

// ============================================================================
// MOTOR CONTROL MODULE
// ============================================================================
// Handles PWM output to 4 ESCs via ESP32-C3 LEDC peripheral
// Implements QuadX mixing algorithm for motor speed calculation
// ============================================================================

/**
 * @brief Initialize motor PWM channels on ESP32-C3
 *        - Sets up LEDC timer at 50Hz
 *        - Configures 4 PWM channels for motors
 *        - Sets initial throttle to minimum (1000 µs = stop)
 */
void motor_init();

/**
 * @brief Set individual motor speed
 * @param motor   Motor index [0-3]: 0=FL, 1=FR, 2=BL, 3=BR
 * @param pwm     PWM value [1000-2000] µs (1000=stop, 2000=full)
 */
void motor_set_speed(uint8_t motor, uint16_t pwm);

/**
 * @brief Get current motor speed
 * @param motor   Motor index [0-3]
 * @return        Current PWM value [1000-2000]
 */
uint16_t motor_get_speed(uint8_t motor);

/**
 * @brief Apply QuadX mixing algorithm
 *        - Converts throttle + PID outputs to individual motor commands
 *        - Format: motor[i] = throttle + k[i][0]*roll + k[i][1]*pitch + k[i][2]*yaw
 *        - Enforces min/max throttle limits
 * @param throttle  Throttle command [1000-2000]
 * @param roll      Roll PID output [-500 to +500]
 * @param pitch     Pitch PID output [-500 to +500]
 * @param yaw       Yaw PID output [-500 to +500]
 * @param motor_out Motor_Output struct to fill with results
 */
void motor_mix_quadx(uint16_t throttle, int16_t roll, int16_t pitch, int16_t yaw, Motor_Output *motor_out);

/**
 * @brief Write all motor speeds to hardware
 * @param motor_out Motor_Output struct with 4 motor speeds
 */
void motor_write_all(const Motor_Output *motor_out);

/**
 * @brief Stop all motors immediately (emergency)
 */
void motor_stop_all();

/**
 * @brief Arm motors (allow them to spin)
 */
void motor_arm();

/**
 * @brief Disarm motors (prevent spinning)
 */
void motor_disarm();

/**
 * @brief Check if motors are armed
 * @return 1 if armed, 0 if disarmed
 */
uint8_t motor_is_armed();

#endif // MOTOR_CONTROL_H
