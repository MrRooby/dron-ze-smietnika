#include "stm8s_gpio.h"
#include "stm8s_tim4.h"
#define DEBUG
#include "stm8s.h"
// #include "stm8s_clk.h"
// #include "stm8s_gpio.h"
#include <stdbool.h>
#include <stdio.h>

// #include "rotors.h"
// #include "pid.h"
#include "radio.h"
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
int32_t cnt = 0;
long lastLoopTime = 0;

int main(void){
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // Set CPU to 16MHz
                                                 //
  GPIO_Init(B_LED_PORT, B_LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(LEDS_PORT, LEDS_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  NRF24_Init();
  // GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);
  // GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);


  #ifdef DEBUG
    Serial_begin(115200);
    printf("booting\n");
  #endif /* ifdef DEBUG */

  // Rotors_Init();
  // IMU_Init();

  Timer_Init();

  while(true){
    long currTime = millis();
    if((currTime - lastLoopTime) >= 50){
      GPIO_WriteReverse(B_LED_PORT, B_LED_PIN);
      // GPIO_WriteReverse(GPIOB, GPIO_PIN_5);
      // GPIO_WriteReverse(GPIOB, GPIO_PIN_4);
      lastLoopTime = currTime;
      NRF24_Write_Reg(NRF_CONFIG, 0x03); 
      printf("Ping: %lu\n", millis()/1000);
      printf("Status %d\n", NRF24_Get_Status());
      NRF24_Test_Connection();
    }

    // setAllRotorPWM(80);
    // Delay(5000);
    // setAllRotorPWM(0);
    // Delay(10000);
  }
}





/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// INTERRUPT VECTORS /////////////////////////////////////////

void TIM4_UPD_OVF_IRQHandler(void) __interrupt(23) {
  ms_tics++;
  TIM4_ClearITPendingBit(TIM4_FLAG_UPDATE);
}
