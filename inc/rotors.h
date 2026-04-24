#ifndef SERIAL_H
#define SERIAL_H

#include "stm8s_tim1.h"
#include "stm8s_tim2.h"
#include "stm8s_gpio.h"
#include "utils.h"

#define L_PORT GPIOD
#define L_PIN GPIO_PIN_3
#define R_PORT GPIOD
#define R_PIN GPIO_PIN_0
#define F_PORT GPIOC
#define F_PIN GPIO_PIN_3
#define B_PORT GPIOC
#define B_PIN GPIO_PIN_4

void initPWM();
void rotors(
    const uint16_t O1,
    const uint16_t O2,
    const uint16_t B1,
    const uint16_t B2,
    const uint8_t delay);

#endif // !SERIAL_H
