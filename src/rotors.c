#include "rotors.h"

void Rotors_Init(void){
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);

  TIM1_TimeBaseInit(15, TIM1_COUNTERMODE_UP, 999, 0);
  TIM2_TimeBaseInit(TIM2_PRESCALER_16, 999);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
      TIM1_OUTPUTSTATE_ENABLE,
      TIM1_OUTPUTNSTATE_DISABLE,
      0,
      TIM1_OCPOLARITY_HIGH,
      TIM1_OCNPOLARITY_HIGH,
      TIM1_OCIDLESTATE_SET,
      TIM1_OCNIDLESTATE_RESET);
  TIM1_OC4Init( 
      TIM1_OCMODE_PWM1,
      TIM1_OUTPUTSTATE_ENABLE,
      0,
      TIM1_OCPOLARITY_HIGH,
      TIM1_OCIDLESTATE_SET);
  TIM2_OC2Init(
      TIM2_OCMODE_PWM1,
      TIM2_OUTPUTSTATE_ENABLE,
      0,
      TIM2_OCPOLARITY_HIGH);
  TIM2_OC3Init(
      TIM2_OCMODE_PWM1,
      TIM2_OUTPUTSTATE_ENABLE,
      0,
      TIM2_OCPOLARITY_HIGH);

  TIM1_CtrlPWMOutputs(ENABLE);
  TIM2_ARRPreloadConfig(ENABLE) ;

  TIM1_Cmd(ENABLE);
  TIM2_Cmd(ENABLE);
}

void setRotorPWM(enum Rotor rotor, const uint16_t pwm){
  switch (rotor) {
    case O1:
      TIM1_SetCompare3(pwm);
      break;
    case O2:
      TIM1_SetCompare4(pwm);
      break;
    case B1:
      TIM2_SetCompare2(pwm);
      break;
    case B2:
      TIM2_SetCompare3(pwm);
      break;
    default:
      return;
  }
}

void setAllRotorPWM(const uint16_t pwm){
  TIM1_SetCompare3(pwm);
  TIM1_SetCompare4(pwm);
  TIM2_SetCompare2(pwm);
  TIM2_SetCompare3(pwm);
}

void mixTable(const int16_t throttle, const int16_t roll, const int16_t pitch, const int16_t yaw) {
    int16_t motor[4];

    motor[O1] = throttle + pitch + yaw; // Rear
    motor[O2] = throttle - roll  - yaw; // Right
    motor[B1] = throttle - pitch + yaw; // Left
    motor[B2] = throttle + roll  - yaw; // Front

    for (uint8_t i = 0; i < 4; i++) {
        // Constrain to 1000-2000 range
        if (motor[i] < 1000) motor[i] = 1000;
        if (motor[i] > 2000) motor[i] = 2000;
        
        // Map 1000-2000 to your Timer ARR (0-999)
        // If 1000 is OFF and 2000 is MAX, we subtract 1000.
        setRotorPWM((enum Rotor)i, (uint16_t)(motor[i] - 1000));
    }
}

void runPID(int16_t setpoint[3], int16_t currentAngle[3]) {
    int16_t error;
    int16_t delta;
    int32_t pTerm, iTerm, dTerm;

    for (uint8_t axis = 0; axis < 3; axis++) {
        // 1. Calculate Error
        error = setpoint[axis] - currentAngle[axis];

        // 2. P-Term (Proportional)
        pTerm = (int32_t)error * (axis == ROLL ? PID_ROLL_P : (axis == PITCH ? PID_PITCH_P : PID_YAW_P));

        // 3. I-Term (Integral) with Anti-Windup
        errorI[axis] += error;
        // Simple Clamp Anti-Windup
        if (errorI[axis] > I_LIMIT) errorI[axis] = I_LIMIT;
        else if (errorI[axis] < -I_LIMIT) errorI[axis] = -I_LIMIT;
        
        iTerm = errorI[axis] * (axis == ROLL ? PID_ROLL_I : (axis == PITCH ? PID_PITCH_I : PID_YAW_I));

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
