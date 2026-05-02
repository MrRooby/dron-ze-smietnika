#include "timing.h"
#include "stm8s_clk.h"
#include "stm8s_gpio.h"
#include "stm8s_tim3.h"
#include "stm8s_itc.h"
#include "stm8s_tim4.h"


volatile uint32_t ms_tics = 0;


void Delay(const uint32_t ms){
  uint32_t start_time = millis();   
  while(millis() - start_time < ms){
    __asm__("nop");
  };
}

void Timer_Init(void){
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 125-1);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  TIM4_Cmd(ENABLE);
  enableInterrupts();
}

uint32_t millis(void){
  uint32_t ms;
  disableInterrupts();
  ms = ms_tics;
  enableInterrupts();
  return ms;
}

uint32_t micros(void){
  uint32_t ms;
  uint8_t tim_cnt;

  disableInterrupts();
  ms = ms_tics;
  tim_cnt = TIM4_GetCounter();

  // Check if TIM4 overflowed but the ISR hasn't run yet
  if ((TIM4->SR1 & TIM4_SR1_UIF) && (tim_cnt < 124)) {
    ms++;
  }
  enableInterrupts();

  // Each unit of tim_cnt is 8 microseconds
  return (ms * 1000) + (uint32_t)(tim_cnt * 8);
}

// TIM3 Update/Overflow Interrupt Vector
// void TIM4_ISR(void) __interrupt(25) {
//   ms_tics++;
//   TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
// }

// Using the explicit SDCC syntax for the TIM4 vector

