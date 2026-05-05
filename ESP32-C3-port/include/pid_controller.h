#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "config.h"
#include "types.h"

// ============================================================================
// PID CONTROLLER MODULE
// ============================================================================
// Implements proportional-integral-derivative control for drone stability
// Runs separate PID loops for ROLL, PITCH, and YAW axes
// ============================================================================

/**
 * @brief Initialize PID controllers with default coefficients from config.h
 *        - Sets up ROLL, PITCH, YAW PID structs
 *        - Resets accumulators and error history
 */
void pid_init();

/**
 * @brief Update one PID controller
 *        - Calculates: output = P*error + I*integral(error) + D*(error_rate)
 *        - Clamps integral to prevent windup
 *        - Returns output in [-500, +500] range for mixing
 * @param pid      Pointer to PID_Controller struct
 * @param setpoint Target attitude angle (0.1° units or other)
 * @param feedback Current attitude angle (same units as setpoint)
 * @param dt       Delta time since last call (seconds)
 * @param max_i    Maximum integral accumulator to prevent windup
 * @return PID output [-500 to +500]
 */
int16_t pid_update(PID_Controller *pid, int16_t setpoint, int16_t feedback, float dt, int16_t max_i);

/**
 * @brief Reset all PID controllers (clear errors and integrals)
 *        - Called when disarming or switching flight modes
 */
void pid_reset_all();

/**
 * @brief Set PID coefficients for a specific axis
 * @param axis Index [0=ROLL, 1=PITCH, 2=YAW]
 * @param p    Proportional gain
 * @param i    Integral gain
 * @param d    Derivative gain
 */
void pid_set_gains(uint8_t axis, float p, float i, float d);

/**
 * @brief Get current PID output for an axis
 * @param axis Index [0=ROLL, 1=PITCH, 2=YAW]
 * @return Latest PID output
 */
int16_t pid_get_output(uint8_t axis);

/**
 * @brief Enable/disable integral windup protection
 * @param enable 1 to enable, 0 to disable
 */
void pid_set_integral_protection(uint8_t enable);

#endif // PID_CONTROLLER_H
