#ifndef ROTORS_H
#define ROTORS_H

#include "stm8s_tim1.h"
#include "stm8s_tim2.h"
#include "stm8s_clk.h"

#define L_PORT GPIOD
#define L_PIN GPIO_PIN_3
#define R_PORT GPIOD
#define R_PIN GPIO_PIN_0
#define F_PORT GPIOC
#define F_PIN GPIO_PIN_3
#define B_PORT GPIOC
#define B_PIN GPIO_PIN_4

enum Rotor {
  O1,  
  O2,  
  B1,  
  B2,  
};

void initPWM();

void setRotorPWM(enum Rotor rotor, const uint8_t pwm);

void setAllRotorPWM(const uint8_t pwm);

#endif // !SERIAL_H
