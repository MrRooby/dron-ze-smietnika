#include "timing.h"


volatile uint32_t ms_tics = 0;


void Delay(const uint32_t ms){
  uint32_t start_time = millis();   
  while(millis() - start_time < ms){
    __asm__("nop");
  };
}

void Timer_Init(void){
  TIM3_TimeBaseInit(TIM3_PRESCALER_16, 1000);
  TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);
  TIM3_Cmd(ENABLE);
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
  uint16_t tim_cnt;

  disableInterrupts();
  ms = ms_tics;
  tim_cnt = TIM3_GetCounter();

  // Checking if timer overflowed in this brief span of time
  if ((TIM3->SR1 & TIM3_SR1_UIF) && (tim_cnt < 1000)) {
    ms++;
  }
  enableInterrupts();

  // miliseconds plus current timer value
  return (ms * 1000) + tim_cnt;
}

// TIM4 Update/Overflow Interrupt Vector
void TIM3_ISR(void) __interrupt(16) {
    ms_tics++;
    TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
}

