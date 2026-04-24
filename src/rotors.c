#include "rotors.h"

void initPWM(void){
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

void rotors(const uint16_t O1, const uint16_t O2, const uint16_t B1, const uint16_t B2, const uint8_t delay){
  for (int pwm = 0; pwm < 200; pwm++) {
    if (pwm < O1) TIM1_SetCompare3(pwm);
    if (pwm < O2) TIM1_SetCompare4(pwm);
    if (pwm < B1) TIM2_SetCompare2(pwm);
    if (pwm < B2) TIM2_SetCompare3(pwm);
    DelayDumb(delay);
  }
}
