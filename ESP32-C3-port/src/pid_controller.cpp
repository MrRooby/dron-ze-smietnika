#include "pid_controller.h"
#include <Arduino.h>
#include <string.h>

// ============================================================================
// PID STATE
// ============================================================================

static PID_Controller pid_controllers[3];  // [0]=ROLL, [1]=PITCH, [2]=YAW

static struct {
  uint8_t integral_protection_enabled;
} pid_state = {1};  // Anti-windup enabled by default

// ============================================================================
// INITIALIZATION
// ============================================================================

void pid_init() {
  // Initialize ROLL PID
  pid_controllers[ROLL].coef.P = PID_ROLL_P;
  pid_controllers[ROLL].coef.I = PID_ROLL_I;
  pid_controllers[ROLL].coef.D = PID_ROLL_D;
  pid_controllers[ROLL].error_prev = 0;
  pid_controllers[ROLL].integral = 0;
  pid_controllers[ROLL].output = 0;
  
  // Initialize PITCH PID
  pid_controllers[PITCH].coef.P = PID_PITCH_P;
  pid_controllers[PITCH].coef.I = PID_PITCH_I;
  pid_controllers[PITCH].coef.D = PID_PITCH_D;
  pid_controllers[PITCH].error_prev = 0;
  pid_controllers[PITCH].integral = 0;
  pid_controllers[PITCH].output = 0;
  
  // Initialize YAW PID
  pid_controllers[YAW].coef.P = PID_YAW_P;
  pid_controllers[YAW].coef.I = PID_YAW_I;
  pid_controllers[YAW].coef.D = PID_YAW_D;
  pid_controllers[YAW].error_prev = 0;
  pid_controllers[YAW].integral = 0;
  pid_controllers[YAW].output = 0;
  
  if (DEBUG_ENABLED) {
    Serial.println("[PID] Controllers initialized (ROLL/PITCH/YAW)");
    Serial.printf("[PID] ROLL: P=%.2f I=%.2f D=%.2f\n", PID_ROLL_P, PID_ROLL_I, PID_ROLL_D);
    Serial.printf("[PID] PITCH: P=%.2f I=%.2f D=%.2f\n", PID_PITCH_P, PID_PITCH_I, PID_PITCH_D);
    Serial.printf("[PID] YAW: P=%.2f I=%.2f D=%.2f\n", PID_YAW_P, PID_YAW_I, PID_YAW_D);
  }
}

// ============================================================================
// PID UPDATE (MAIN ALGORITHM)
// ============================================================================

int16_t pid_update(PID_Controller *pid, int16_t setpoint, int16_t feedback, float dt, int16_t max_i) {
  if (!pid || dt <= 0) return 0;
  
  // Calculate error
  int32_t error = setpoint - feedback;
  
  // Proportional term
  float p_term = pid->coef.P * error;
  
  // Integral term (with anti-windup)
  pid->integral += error * dt;
  
  // Anti-windup: clamp integral accumulator
  if (pid_state.integral_protection_enabled) {
    if (pid->integral > max_i) pid->integral = max_i;
    if (pid->integral < -max_i) pid->integral = -max_i;
  }
  
  float i_term = pid->coef.I * pid->integral;
  
  // Derivative term (filtered)
  float error_rate = (error - pid->error_prev) / dt;
  float d_term = pid->coef.D * error_rate;
  pid->error_prev = error;
  
  // Sum all terms
  float output = p_term + i_term + d_term;
  
  // Clamp output to [-500, +500] for motor mixing
  if (output > 500) output = 500;
  if (output < -500) output = -500;
  
  pid->output = (int16_t)output;
  
  return pid->output;
}

// ============================================================================
// RESET & CONFIGURATION
// ============================================================================

void pid_reset_all() {
  for (uint8_t i = 0; i < 3; i++) {
    pid_controllers[i].error_prev = 0;
    pid_controllers[i].integral = 0;
    pid_controllers[i].output = 0;
  }
  
  if (DEBUG_ENABLED) {
    Serial.println("[PID] All controllers reset");
  }
}

void pid_set_gains(uint8_t axis, float p, float i, float d) {
  if (axis >= 3) return;
  
  pid_controllers[axis].coef.P = p;
  pid_controllers[axis].coef.I = i;
  pid_controllers[axis].coef.D = d;
  
  if (DEBUG_ENABLED) {
    const char *axis_name[] = {"ROLL", "PITCH", "YAW"};
    Serial.printf("[PID] %s gains: P=%.2f I=%.2f D=%.2f\n", axis_name[axis], p, i, d);
  }
}

int16_t pid_get_output(uint8_t axis) {
  if (axis >= 3) return 0;
  return pid_controllers[axis].output;
}

void pid_set_integral_protection(uint8_t enable) {
  pid_state.integral_protection_enabled = enable ? 1 : 0;
}
