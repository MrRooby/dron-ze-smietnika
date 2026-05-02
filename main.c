#include "stm8s_tim4.h"
#define DEBUG
#include "stm8s.h"
// #include "stm8s_clk.h"
// #include "stm8s_gpio.h"
#include <stdbool.h>
#include <stdio.h>

// #include "rotors.h"
// #include "pid.h"
// #include "radio.h"
#include "timing.h"
// #include "imu.h"
// #include "pid.h"

#ifdef DEBUG
  #include "serial.h"
#endif /* ifdef DEBUG */

/*
    front
   O1   O2
      X
   B1   B2

   B - O1
   F - B1
   R - O2
   L - B2
*/

// LEDs
#define B_LED_PORT GPIOF
#define B_LED_PIN GPIO_PIN_4
#define LEDS_PORT GPIOC
#define LEDS_PIN GPIO_PIN_1

#define LOOP_T 1000

uint16_t pwm = 0;
bool down = false;

int main(void){
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // Set CPU to 16MHz
                                                 //
  GPIO_Init(B_LED_PORT, B_LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(LEDS_PORT, LEDS_PIN, GPIO_MODE_OUT_PP_LOW_FAST);


  #ifdef DEBUG
    Serial_begin(115200);
  #endif /* ifdef DEBUG */

  // Rotors_Init();
  // initMPU();
  // GPIO_WriteHigh(B_LED_PORT, B_LED_PIN);
  // Delay(3000);

  // long lastLoopTime = micros();
  printf("booting\n");
  Timer_Init();
  int32_t cnt = 0;

  while(true){
    // long currTime = micros();
   //  if((currTime - lastLoopTime) >= LOOP_T){
   //    lastLoopTime = currTime;
   //
   //    computeIMU();
   //    runPID(rcCommand, att.angle);
   //    mixTable(rcThrottle, axisPID);
   // }

    #ifdef DEBUG
      printf("%lu\n", millis());

    #endif /* ifdef DEBUG */

    // setAllRotorPWM(80);
    // Delay(5000);
    // setAllRotorPWM(0);
    // Delay(10000);
  }
}

void TIM4_UPD_OVF_IRQHandler(void) __interrupt(23) {
    ms_tics++;
    TIM4->SR1 &= (uint8_t)(~0x01); // Direct clear to avoid library overhead during crash
}
