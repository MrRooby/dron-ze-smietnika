#define DEBUG
#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_gpio.h"
#include <stdbool.h>
#include <stdio.h>

#include "utils.h"
#include "rotors.h"
#include "imu.h"
#include "radio.h"

#ifdef DEBUG
  #include "serial.h"
#endif /* ifdef DEBUG */

/*
   O O
   B B

   B - te polutowane O1
   f - B1
   r - O2
   l - B2
   */

// LEDs
#define B_LED_PORT GPIOF
#define B_LED_PIN GPIO_PIN_4
#define LEDS_PORT GPIOC
#define LEDS_PIN GPIO_PIN_1

uint16_t pwm = 0;
bool down = false;

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

int main(void){
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // Set CPU to 16MHz
  GPIO_Init(B_LED_PORT, B_LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(LEDS_PORT, LEDS_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

#ifdef DEBUG
  Serial_begin(115200);
#endif /* ifdef DEBUG */

  initPWM();
  initMPU();
  GPIO_WriteHigh(LEDS_PORT, LEDS_PIN);
  DelayDumb(3000);


  while(true){

#ifdef DEBUG
    printf("ping\n");
#endif /* ifdef DEBUG */

    TIM1_SetCompare3(80);
    TIM1_SetCompare4(80);
    TIM2_SetCompare2(80);
    TIM2_SetCompare3(80);
    DelayDumb(5000);
    TIM1_SetCompare3(0);
    TIM1_SetCompare4(0);
    TIM2_SetCompare2(0);
    TIM2_SetCompare3(0);
    DelayDumb(10000);
    // readTemp();
  }
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////



