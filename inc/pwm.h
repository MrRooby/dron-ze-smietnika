#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

/**
 * Initialize PWM timers (TIM1 and TIM2)
 * - TIM1: Compare3 (Front rotor - PC3), Compare4 (Back rotor - PC4)
 * - TIM2: Compare2 (Left rotor - PD3), Compare3 (Right rotor - PD4)
 */
void pwm_init(void);

/**
 * Set rotor PWM values
 * @param front: Front rotor PWM (0-255)
 * @param back: Back rotor PWM (0-255)
 * @param left: Left rotor PWM (0-255)
 * @param right: Right rotor PWM (0-255)
 */
void pwm_set_rotors(uint8_t front, uint8_t back, uint8_t left, uint8_t right);

/**
 * Ramp rotors from 0 to target speed
 * @param front: Target front PWM
 * @param back: Target back PWM
 * @param left: Target left PWM
 * @param right: Target right PWM
 * @param delay_ms: Milliseconds between steps
 */
void pwm_ramp_rotors(uint16_t front, uint16_t back, uint16_t left, uint16_t right, uint8_t delay_ms);

#endif
