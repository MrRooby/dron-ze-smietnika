#include "serial.h"
#include "stm8s_gpio.h"
#include <stdbool.h>
#include <stdio.h>

#define L_PORT GPIOD
#define L_PIN GPIO_PIN_3

#define R_PORT GPIOD
#define R_PIN GPIO_PIN_2

#define F_PORT GPIOC
#define F_PIN GPIO_PIN_3

#define B_PORT GPIOC
#define B_PIN GPIO_PIN_4

int main(void){
  Serial_begin(115200);
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);

  while(true){
    printf("zyje\n");

    GPIO_WriteHigh(GPIOC, GPIO_PIN_6);
    for (volatile uint32_t i = 0; i < 50000; i++);
    GPIO_WriteLow(GPIOC, GPIO_PIN_6);
    for (volatile uint32_t i = 0; i < 50000; i++);
  }
}
