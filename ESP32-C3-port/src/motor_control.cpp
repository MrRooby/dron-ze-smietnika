#include "motor_control.h"
#include <Arduino.h>

static Motor_Output current_motors = {
  .value = {1000, 1000, 1000, 1000},  // All stopped
  .armed = 0
};

static const uint8_t motor_pins[4] = {
  MOTOR_FL_PIN,     // Motor 0: Front-Left
  MOTOR_FR_PIN,     // Motor 1: Front-Right
  MOTOR_BL_PIN,     // Motor 2: Back-Left
  MOTOR_BR_PIN      // Motor 3: Back-Right
};

static const uint8_t ledc_channels[4] = {
  LEDC_CHANNEL_FL,  // Motor 0: Front-Left
  LEDC_CHANNEL_FR,  // Motor 1: Front-Right
  LEDC_CHANNEL_BL,  // Motor 2: Back-Left
  LEDC_CHANNEL_BR   // Motor 3: Back-Right
};

static uint16_t pwm_to_duty(uint16_t pwm) {
  // Clamp PWM to valid range
  if (pwm < MINTHROTTLE) pwm = MINTHROTTLE;
  if (pwm > MAXTHROTTLE) pwm = MAXTHROTTLE;
  
  uint32_t duty = ((uint32_t)pwm * 16384 + 10000) / 20000;
  return (uint16_t)duty;
}


void motor_init() {
  // Initialize all 4 motor channels
  for (uint8_t i = 0; i < 4; i++) {
    // Attach pin to LEDC channel
    ledcSetup(i, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(motor_pins[i], ledc_channels[i]);
    
    // Set initial duty cycle (motors stopped at 1000 µs)
    uint16_t duty = pwm_to_duty(MINTHROTTLE);
    ledcWrite(ledc_channels[i], duty);
  }
  
  // All motors disarmed by default
  current_motors.armed = 0;
  
  if (DEBUG_ENABLED) {
    Serial.println("[MOTOR] Initialized 4 PWM channels at 50Hz, all motors stopped");
    Serial.printf("[MOTOR] FL=%d, FR=%d, BL=%d, BR=%d\n", 
      motor_pins[0], motor_pins[1], motor_pins[2], motor_pins[3]);
  }
}


void motor_set_speed(uint8_t motor, uint16_t pwm) {
  if (motor >= 4) return;  // Invalid motor index
  
  // Clamp PWM value
  if (pwm < MINTHROTTLE) pwm = MINTHROTTLE;
  if (pwm > MAXTHROTTLE) pwm = MAXTHROTTLE;
  
  // Convert PWM to duty cycle and write to hardware
  uint16_t duty = pwm_to_duty(pwm);
  ledcWrite(ledc_channels[motor], duty);
  
  // Update state
  current_motors.value[motor] = pwm;
}

uint16_t motor_get_speed(uint8_t motor) {
  if (motor >= 4) return 0;
  return current_motors.value[motor];
}


void motor_mix_quadx(uint16_t throttle, int16_t roll, int16_t pitch, int16_t yaw, Motor_Output *motor_out) {
  // QuadX mixing matrix coefficients
  // motor[i] = throttle + coef[i][0]*roll + coef[i][1]*pitch + coef[i][2]*yaw
  //
  // QuadX configuration:
  //     Front
  //    (0)  (1)
  //    FL   FR
  //     |   |
  //     |   |
  //    BL   BR
  //    (2)  (3)
  //     Back
  //
  // Mixing:
  // FL (0): throttle + roll - pitch - yaw  = (+1, -1, -1)
  // FR (1): throttle - roll - pitch + yaw  = (-1, -1, +1)
  // BL (2): throttle + roll + pitch + yaw  = (+1, +1, +1)
  // BR (3): throttle - roll + pitch - yaw  = (-1, +1, -1)
  
  static const int8_t mix_matrix[4][3] = {
    {+1, -1, -1},  // Motor 0: Front-Left
    {-1, -1, +1},  // Motor 1: Front-Right
    {+1, +1, +1},  // Motor 2: Back-Left
    {-1, +1, -1}   // Motor 3: Back-Right
  };
  
  // Apply mixing for each motor
  for (uint8_t i = 0; i < 4; i++) {
    int32_t motor_cmd = throttle;
    motor_cmd += (int32_t)roll * mix_matrix[i][0] / 4;    // Scale down to prevent overflow
    motor_cmd += (int32_t)pitch * mix_matrix[i][1] / 4;
    motor_cmd += (int32_t)yaw * mix_matrix[i][2] / 4;
    
    // Clamp to valid PWM range
    if (motor_cmd < MINTHROTTLE) motor_cmd = MINTHROTTLE;
    if (motor_cmd > MAXTHROTTLE) motor_cmd = MAXTHROTTLE;
    
    motor_out->value[i] = (uint16_t)motor_cmd;
  }
  
  motor_out->armed = current_motors.armed;
}


void motor_write_all(const Motor_Output *motor_out) {
  if (!motor_out) return;
  
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t pwm = motor_out->value[i];
    
    // If disarmed, force throttle to minimum
    if (!current_motors.armed) {
      pwm = MINTHROTTLE;
    }
    
    motor_set_speed(i, pwm);
  }
  
  if (DEBUG_MOTORS && DEBUG_ENABLED) {
    Serial.printf("[MOTOR] FL=%d FR=%d BL=%d BR=%d (armed=%d)\n",
      current_motors.value[0], current_motors.value[1],
      current_motors.value[2], current_motors.value[3],
      current_motors.armed);
  }
}


void motor_arm() {
  current_motors.armed = 1;
  if (DEBUG_ENABLED) {
    Serial.println("[MOTOR] *** ARMED ***");
  }
}

void motor_disarm() {
  current_motors.armed = 0;
  motor_stop_all();
  if (DEBUG_ENABLED) {
    Serial.println("[MOTOR] *** DISARMED ***");
  }
}

uint8_t motor_is_armed() {
  return current_motors.armed;
}

void motor_stop_all() {
  for (uint8_t i = 0; i < 4; i++) {
    motor_set_speed(i, MINTHROTTLE);
  }
}
