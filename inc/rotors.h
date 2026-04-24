#ifndef ROTORS_H
#define ROTORS_H

#include "stm8s_tim1.h"
#include "stm8s_tim2.h"
#include "stm8s_clk.h"
#include "imu.h"

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

void Rotors_Init(void);
void setRotorPWM(enum Rotor rotor, const uint16_t pwm);
void setAllRotorPWM(const uint16_t pwm);
void mixTable(
    const int16_t throttle,
    const int16_t roll,
    const int16_t pitch,
    const int16_t yaw
);

#endif // !ROTORS_H
