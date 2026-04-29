#include "pid.h"

AxisPID axisPID = {{0,0,0}};

void runPID(int16_t setpoint[3], int16_t currentAngle[3], uint16_t throttle) {
  if (throttle < 1050) {
    for (int i = 0; i < 3; i++){
      errorI[i] = 0; 
      lastError[i] = 0;
      axisPID.axis[i] = 0;
    }
  
  }

  for (uint8_t axis = 0; axis < 3; axis++) {
    // 1. Calculate Error
    int16_t error = setpoint[axis] - currentAngle[axis];

    // 2. P-Term (Proportional)
    int16_t pTerm = (int32_t)error * kp[axis];

    // 3. I-Term (Integral) with Anti-Windup
    errorI[axis] += error;
    // Simple Clamp Anti-Windup
    if (errorI[axis] > I_LIMIT) errorI[axis] = I_LIMIT;
    else if (errorI[axis] < -I_LIMIT) errorI[axis] = -I_LIMIT;

    int16_t iTerm = errorI[axis] * (axis == ROLL ? PID_ROLL_I : (axis == PITCH ? PID_PITCH_I : PID_YAW_I));

    // 4. D-Term (Derivative) 
    // Note: MultiWii usually applies D-term to the change in measurement to avoid "D-kick"
    delta = error - lastError[axis];
    lastError[axis] = error;

    dTerm = (int32_t)delta * (axis == ROLL ? PID_ROLL_D : (axis == PITCH ? PID_PITCH_D : PID_YAW_D));

    // 5. Combine and Scale Output
    // We divide by a power of 2 (e.g., >> 8) to bring the high-precision sum 
    // back into a range usable by the motor mixer.
    axisPID[axis] = (int16_t)((pTerm + iTerm + dTerm) >> 8);
  }
}
