#ifndef TIMING_H
#define TIMING_H
#include "stm8s.h"
#include "stm8s_tim3.h"

extern volatile uint32_t ms_tics;

void Delay(const uint32_t ms);
void Timer_Init(void);
uint32_t millis(void);
uint32_t micros(void);

#endif // !TIMING_H
