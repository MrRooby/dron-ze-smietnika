#ifndef RC_PROCESSOR_H
#define RC_PROCESSOR_H

#include "config.h"
#include "types.h"

// ============================================================================
// RC PROCESSOR MODULE
// ============================================================================
// Processes raw RC commands and generates normalized control inputs
// Handles arming/disarming, flight mode switching, failsafe
// ============================================================================

/**
 * @brief Initialize RC processor state
 */
void rc_init();

/**
 * @brief Process raw RC data into command values
 *        - Applies expo curves for smoother control feel
 *        - Normalizes throttle to [MINTHROTTLE-MAXTHROTTLE]
 *        - Normalizes roll/pitch/yaw to [-500, +500]
 * @param rc_state Pointer to RC_State struct with raw rcData[] filled
 */
void rc_process(RC_State *rc_state);

/**
 * @brief Check for arm stick position (yaw full right, throttle near zero)
 * @param rc_state Pointer to RC_State struct
 * @return 1 if arm stick detected, 0 otherwise
 */
uint8_t rc_check_arm(RC_State *rc_state);

/**
 * @brief Check for disarm stick position (yaw full left, throttle near zero)
 * @param rc_state Pointer to RC_State struct
 * @return 1 if disarm stick detected, 0 otherwise
 */
uint8_t rc_check_disarm(RC_State *rc_state);

/**
 * @brief Apply failsafe when signal is lost
 *        - Reduces throttle to safe landing speed
 *        - Zeros out roll/pitch/yaw commands
 * @param rc_state Pointer to RC_State struct
 */
void rc_failsafe_apply(RC_State *rc_state);

/**
 * @brief Get current flight mode from AUX channel
 * @param rc_state Pointer to RC_State struct
 * @return FlightMode enum value
 */
uint8_t rc_get_flight_mode(RC_State *rc_state);

/**
 * @brief Expo curve for smoother stick response
 *        - Makes small stick inputs have less effect (more precise)
 *        - Makes large stick inputs proportional to requested rate
 * @param input Raw stick input [-500 to +500]
 * @param expo Expo factor (0.0 = linear, 1.0 = cubic, default ~0.7)
 * @return Transformed output [-500 to +500]
 */
int16_t rc_expo_curve(int16_t input, float expo);

#endif // RC_PROCESSOR_H
