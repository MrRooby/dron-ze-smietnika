#include "rc_processor.h"
#include <Arduino.h>
#include <math.h>

// ============================================================================
// RC STATE
// ============================================================================

static struct {
  uint32_t arm_stick_start_ms;
  uint32_t disarm_stick_start_ms;
} rc_state_local = {0, 0};

// ============================================================================
// INITIALIZATION
// ============================================================================

void rc_init() {
  rc_state_local.arm_stick_start_ms = 0;
  rc_state_local.disarm_stick_start_ms = 0;
  
  if (DEBUG_ENABLED) {
    Serial.println("[RC] RC Processor initialized");
  }
}

// ============================================================================
// RC PROCESSING
// ============================================================================

void rc_process(RC_State *rc_state) {
  if (!rc_state) return;
  
  // Extract raw RC values (1000-2000 µs range)
  uint16_t throttle = rc_state->rcData[THROTTLE];
  uint16_t roll = rc_state->rcData[ROLL];
  uint16_t pitch = rc_state->rcData[PITCH];
  uint16_t yaw = rc_state->rcData[YAW];
  
  // ========== THROTTLE PROCESSING ==========
  // Map 1000-2000 to direct throttle command [1000-2000]
  // No changes needed, already in correct range
  rc_state->rcCommand[THROTTLE] = constrain(throttle, MINTHROTTLE, MAXTHROTTLE);
  
  // ========== ROLL/PITCH/YAW PROCESSING ==========
  // Map 1000-2000 to -500...+500 command range
  // Center at 1500 µs
  
  int16_t roll_raw = (int16_t)roll - MIDTHROTTLE;      // -500 to +500
  int16_t pitch_raw = (int16_t)pitch - MIDTHROTTLE;    // -500 to +500
  int16_t yaw_raw = (int16_t)yaw - MIDTHROTTLE;        // -500 to +500
  
  // Apply expo curve for smoother control (optional, set expo to 0 to disable)
  //float expo_factor = 0.7f;  // Tune this for feel
  float expo_factor = 0.0f;   // Disabled for now (linear response)
  
  rc_state->rcCommand[ROLL] = rc_expo_curve(roll_raw, expo_factor);
  rc_state->rcCommand[PITCH] = rc_expo_curve(pitch_raw, expo_factor);
  rc_state->rcCommand[YAW] = rc_expo_curve(yaw_raw, expo_factor);
  
  // Clamp to safe limits
  rc_state->rcCommand[ROLL] = constrain(rc_state->rcCommand[ROLL], -500, 500);
  rc_state->rcCommand[PITCH] = constrain(rc_state->rcCommand[PITCH], -500, 500);
  rc_state->rcCommand[YAW] = constrain(rc_state->rcCommand[YAW], -500, 500);
  
  // Determine flight mode from AUX channel
  rc_state->flight_mode = rc_get_flight_mode(rc_state);
}

// ============================================================================
// ARM/DISARM DETECTION
// ============================================================================

uint8_t rc_check_arm(RC_State *rc_state) {
  if (!rc_state) return 0;
  
  uint16_t throttle = rc_state->rcData[THROTTLE];
  uint16_t yaw = rc_state->rcData[YAW];
  
  // Check conditions: throttle low, yaw high (right stick full right)
  if (throttle < ARM_THROTTLE_MAX && yaw > ARM_YAW_MIN && yaw < ARM_YAW_MAX) {
    // Stick in correct position
    if (rc_state_local.arm_stick_start_ms == 0) {
      rc_state_local.arm_stick_start_ms = millis();
    }
    
    // Check if held long enough
    if ((millis() - rc_state_local.arm_stick_start_ms) > STICK_ARM_DELAY) {
      // ARM detected!
      rc_state_local.arm_stick_start_ms = 0;  // Reset
      return 1;
    }
  } else {
    // Stick moved away from arm position
    rc_state_local.arm_stick_start_ms = 0;
  }
  
  return 0;
}

uint8_t rc_check_disarm(RC_State *rc_state) {
  if (!rc_state) return 0;
  
  uint16_t throttle = rc_state->rcData[THROTTLE];
  uint16_t yaw = rc_state->rcData[YAW];
  
  // Check conditions: throttle low, yaw low (left stick full left)
  if (throttle < DISARM_THROTTLE_MAX && yaw > DISARM_YAW_MIN && yaw < DISARM_YAW_MAX) {
    // Stick in correct position
    if (rc_state_local.disarm_stick_start_ms == 0) {
      rc_state_local.disarm_stick_start_ms = millis();
    }
    
    // Check if held long enough
    if ((millis() - rc_state_local.disarm_stick_start_ms) > STICK_DISARM_DELAY) {
      // DISARM detected!
      rc_state_local.disarm_stick_start_ms = 0;  // Reset
      return 1;
    }
  } else {
    // Stick moved away from disarm position
    rc_state_local.disarm_stick_start_ms = 0;
  }
  
  return 0;
}

// ============================================================================
// FAILSAFE
// ============================================================================

void rc_failsafe_apply(RC_State *rc_state) {
  if (!rc_state) return;
  
  // Cut throttle to safe landing level
  rc_state->rcCommand[THROTTLE] = 1300;  // ~30% throttle for descent
  
  // Zero out pitch/roll commands (neutral)
  rc_state->rcCommand[ROLL] = 0;
  rc_state->rcCommand[PITCH] = 0;
  rc_state->rcCommand[YAW] = 0;  // Stop spinning
}

// ============================================================================
// FLIGHT MODE SELECTION
// ============================================================================

uint8_t rc_get_flight_mode(RC_State *rc_state) {
  if (!rc_state) return MODE_RATE;
  
  // Use AUX1 channel to determine flight mode
  uint16_t aux1 = rc_state->rcData[AUX1];
  
  if (aux1 < 1300) {
    return MODE_RATE;        // Low: Rate mode (manual)
  } else if (aux1 < 1700) {
    return MODE_ACRO;        // Mid: Acro mode (high rate)
  } else {
    return MODE_STABILIZE;   // High: Stabilize mode (assisted leveling)
  }
}

// ============================================================================
// EXPO CURVE
// ============================================================================

int16_t rc_expo_curve(int16_t input, float expo) {
  if (expo <= 0.0f) {
    // Linear response
    return input;
  }
  
  // Cubic expo curve: output = x^3 * expo + x * (1 - expo)
  // This makes small inputs proportionally smaller, large inputs less affected
  float normalized = input / 500.0f;  // Convert to [-1, +1]
  
  float cubic = normalized * normalized * normalized;
  float output = cubic * expo + normalized * (1.0f - expo);
  
  // Convert back to [-500, +500]
  int16_t result = (int16_t)(output * 500.0f);
  
  return constrain(result, -500, 500);
}
