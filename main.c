#include "stm8s_clk.h"
// #include "stm8s_iwdg.h"
// #include "serial.h"
// #include "pwm.h"
// #include "mpu.h"
#include "gpio.h"
// #include "utils.h"
// #include <stdio.h>

/**
 * STM8S005 Drone Controller
 * 
 * UART RX: Rotor PWM commands from ESP32 (115200 baud)
 * PWM OUT: Motor ESC control signals
 * I2C: MPU6050 sensor communication
 * GPIO: Status LEDs
 */

int main(void) {
  // Initialize system clock to 16MHz
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
  
  // Initialize peripherals
  gpio_init_leds();
  
  // Turn on blue LED
  led_blue_on();
  led_blue_off();

  // Main control loop: receive commands and apply rotor control
  while (1) {
    // Watchdog - temporarily disabled for testing
    // IWDG_ReloadCounter();
    
    // Just keep LED on
  }
}
