#include "stm8s_clk.h"
// #include "serial.h"
// #include "pwm.h"
// #include "mpu.h"
#include "gpio.h"
#include "utils.h"
#include <stdio.h>

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
  // Serial_begin(115200);
  // pwm_init();
  // mpu_init();
  
  // Signal startup complete
  led_status_on();
  delay_ms_dumb(3000);
  led_status_off();

  // Turn on blue LED
  led_blue_on();

  // Main control loop: receive commands and apply rotor control
  while (1) {
    // led_blue_on();
    // printf("test");
    // Get latest rotor command from UART (non-blocking)
    // RotorPWM_t* pwm_data = Serial_getRotorPWM();
    
    // // Apply new rotor speeds if valid packet received
    // if (pwm_data->valid) {
    //   pwm_set_rotors(pwm_data->O1, pwm_data->O2, pwm_data->B1, pwm_data->B2);
    //   pwm_data->valid = 0;  // Clear flag after processing
    // }
    
    // // Brief delay to prevent busy-waiting (10ms = 100Hz control rate)
    // delay_ms_dumb(10);
  }
}
