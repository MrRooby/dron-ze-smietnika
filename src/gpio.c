#include "gpio.h"

void gpio_init_leds(void) {
  GPIO_Init(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
}

void led_blue_on(void) {
  GPIO_WriteHigh(LED_BLUE_PORT, LED_BLUE_PIN);
}

void led_blue_off(void) {
  GPIO_WriteLow(LED_BLUE_PORT, LED_BLUE_PIN);
}

void led_status_on(void) {
  GPIO_WriteHigh(LED_STATUS_PORT, LED_STATUS_PIN);
}

void led_status_off(void) {
  GPIO_WriteLow(LED_STATUS_PORT, LED_STATUS_PIN);
}

void led_status_toggle(void) {
  GPIO_WriteReverse(LED_STATUS_PORT, LED_STATUS_PIN);
}
