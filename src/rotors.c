#include "rotors.h"
#include "pid.h"

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

void mixTable(const int16_t throttle, AxisPID *p_pid) {
    int16_t motor[4];

    motor[O1] = throttle + p_pid->pitch + p_pid->yaw; // Rear
    motor[O2] = throttle - p_pid->roll  - p_pid->yaw; // Right
    motor[B1] = throttle - p_pid->pitch + p_pid->yaw; // Left
    motor[B2] = throttle + p_pid->roll  - p_pid->yaw; // Front

    for (uint8_t i = 0; i < 4; i++) {
        // Constrain to 1000-2000 range
        if (motor[i] < 1000) motor[i] = 1000;
        if (motor[i] > 2000) motor[i] = 2000;
        
        // Map 1000-2000 to your Timer ARR (0-999)
        // If 1000 is OFF and 2000 is MAX, we subtract 1000.
        setRotorPWM((enum Rotor)i, (uint16_t)(motor[i] - 1000));
    }
}

