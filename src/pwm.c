#include "pwm.h"
#include "stm8s_tim1.h"
#include "stm8s_tim2.h"
#include "stm8s_clk.h"
#include "utils.h"

void pwm_init(void) {
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);

  // TIM1: 16MHz / 16 = 1MHz (1us resolution), 1000us period = 1kHz PWM
  TIM1_TimeBaseInit(15, TIM1_COUNTERMODE_UP, 999, 0);
  
  // TIM2: 16MHz / 16 = 1MHz, 1000us period = 1kHz PWM
  TIM2_TimeBaseInit(TIM2_PRESCALER_16, 999);

  // TIM1 Compare 3 (Front rotor - PC3)
  TIM1_OC3Init(TIM1_OCMODE_PWM1,
      TIM1_OUTPUTSTATE_ENABLE,
      TIM1_OUTPUTNSTATE_DISABLE,
      0,
      TIM1_OCPOLARITY_HIGH,
      TIM1_OCNPOLARITY_HIGH,
      TIM1_OCIDLESTATE_SET,
      TIM1_OCNIDLESTATE_RESET);
  
  // TIM1 Compare 4 (Back rotor - PC4)
  TIM1_OC4Init( 
      TIM1_OCMODE_PWM1,
      TIM1_OUTPUTSTATE_ENABLE,
      0,
      TIM1_OCPOLARITY_HIGH,
      TIM1_OCIDLESTATE_SET);
  
  // TIM2 Compare 2 (Left rotor - PD3)
  TIM2_OC2Init(
      TIM2_OCMODE_PWM1,
      TIM2_OUTPUTSTATE_ENABLE,
      0,
      TIM2_OCPOLARITY_HIGH);
  
  // TIM2 Compare 3 (Right rotor - PD4)
  TIM2_OC3Init(
      TIM2_OCMODE_PWM1,
      TIM2_OUTPUTSTATE_ENABLE,
      0,
      TIM2_OCPOLARITY_HIGH);

  TIM1_CtrlPWMOutputs(ENABLE);
  TIM2_ARRPreloadConfig(ENABLE);

  TIM1_Cmd(ENABLE);
  TIM2_Cmd(ENABLE);
}

void pwm_set_rotors(uint8_t front, uint8_t back, uint8_t left, uint8_t right) {
  TIM1_SetCompare3(front);   // Front rotor
  TIM1_SetCompare4(back);    // Back rotor
  TIM2_SetCompare2(left);    // Left rotor
  TIM2_SetCompare3(right);   // Right rotor
}

void pwm_ramp_rotors(uint16_t front, uint16_t back, uint16_t left, uint16_t right, uint8_t delay_ms) {
  uint16_t max_val = front;
  if (back > max_val) max_val = back;
  if (left > max_val) max_val = left;
  if (right > max_val) max_val = right;

  for (uint16_t pwm = 0; pwm <= max_val; pwm++) {
    uint8_t f = (pwm < front) ? pwm : front;
    uint8_t b = (pwm < back) ? pwm : back;
    uint8_t l = (pwm < left) ? pwm : left;
    uint8_t r = (pwm < right) ? pwm : right;
    
    pwm_set_rotors(f, b, l, r);
    delayDumb(delay_ms);
  }
}
