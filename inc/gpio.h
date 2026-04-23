#ifndef __GPIO_H
#define __GPIO_H

#include "stm8s_gpio.h"

// Drone Rotor Pins
#define ROTOR_L_PORT GPIOD
#define ROTOR_L_PIN GPIO_PIN_3
#define ROTOR_R_PORT GPIOD
#define ROTOR_R_PIN GPIO_PIN_0
#define ROTOR_F_PORT GPIOC
#define ROTOR_F_PIN GPIO_PIN_3
#define ROTOR_B_PORT GPIOC
#define ROTOR_B_PIN GPIO_PIN_4

// LED Pins
#define LED_STATUS_PORT GPIOF
#define LED_STATUS_PIN GPIO_PIN_4
#define LED_WINGS_PORT GPIOC
#define LED_WINGS_PIN GPIO_PIN_1

/**
 * Initialize GPIO pins for LEDs and status indicators
 */
void gpio_init_leds(void);

/**
 * Turn on LEDs on wings
*/
void led_wings_on(void);

/**
 * Turn off LEDs on wings
 */
void led_wings_off(void);

/**
 * Turn on status LED
 */
void led_status_on(void);

/**
 * Turn off status LED
 */
void led_status_off(void);

/**
 * Toggle status LED
 */
void led_status_toggle(void);

#endif
