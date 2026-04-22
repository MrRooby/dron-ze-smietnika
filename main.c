#include "serial.h"
#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_tim1.h"
#include "stm8s_gpio.h"
#include "stm8s_i2c.h"
#include "stm8s_tim2.h"
#include <stdbool.h>
#include <stdio.h>

/*
   O O
   B B

   B - te polutowane O1
   f - B1
   r - O2
   l - B2
   */

// Drone Rotors
#define L_PORT GPIOD
#define L_PIN GPIO_PIN_3
#define R_PORT GPIOD
#define R_PIN GPIO_PIN_0
#define F_PORT GPIOC
#define F_PIN GPIO_PIN_3
#define B_PORT GPIOC
#define B_PIN GPIO_PIN_4

// MPU6050
#define MPU_SCL_PORT GPIOB
#define MPU_SCL_PIN GPIO_PIN_4
#define MPU_SCK_PORT GPIOB
#define MPU_SCK_PIN GPIO_PIN_5
#define MPU6050_ADDRESS (0x68 << 1)

// LEDs
#define B_LED_PORT GPIOF
#define B_LED_PIN GPIO_PIN_4
#define LEDS_PORT GPIOC
#define LEDS_PIN GPIO_PIN_1


inline void DelayDumb(const uint32_t ms);
void initMPU();
void MPU_Write(uint8_t reg, uint8_t data);
uint8_t MPU_ReadReg(uint8_t reg);
void readTemp();
void initPWM();
void rotors(const uint16_t O1, const uint16_t O2, const uint16_t B1, const uint16_t B2, const uint8_t delay);

uint16_t pwm = 0;
bool down = false;

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

int main(void){
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // Set CPU to 16MHz
  GPIO_Init(B_LED_PORT, B_LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(LEDS_PORT, LEDS_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

  Serial_begin(115200);

  initPWM();
  initMPU();
  GPIO_WriteHigh(LEDS_PORT, LEDS_PIN);
  DelayDumb(3000);


  while(true){
    printf("ping\n");
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

inline void DelayDumb(const uint32_t ms){
  for (volatile uint32_t i = 0; i < (ms * 800); i++);
}

void initMPU(){
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
  GPIO_Init(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST);

  I2C_DeInit();                              
  I2C_Init(
      100000,           // 100kHz I2C Clock
      0x01,             // Own address
      I2C_DUTYCYCLE_2,               
      I2C_ACK_CURR,                   
      I2C_ADDMODE_7BIT,
      16                            
      );

  I2C_Cmd(ENABLE);
}

void MPU_Write(uint8_t reg, uint8_t data) {
  I2C_GenerateSTART(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)) {GPIO_WriteHigh(B_LED_PORT, B_LED_PIN);}
  GPIO_WriteLow(B_LED_PORT, B_LED_PIN);

  I2C_Send7bitAddress(MPU6050_ADDRESS, I2C_DIRECTION_TX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(reg);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_SendData(data);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTOP(ENABLE);
}

uint8_t MPU_ReadReg(uint8_t reg) {
  uint8_t data;

  // 1. Wait for bus to be idle
  while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));

  // 2. Start + Address (Write Mode)
  I2C_GenerateSTART(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(MPU6050_ADDRESS, I2C_DIRECTION_TX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  // 3. Point to the register
  I2C_SendData(reg);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  // 4. Restart + Address (Read Mode)
  I2C_GenerateSTART(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(MPU6050_ADDRESS, I2C_DIRECTION_RX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  // 5. Read the byte and NACK (since it's only 1 byte)
  I2C_AcknowledgeConfig(I2C_ACK_NONE); // Don't ack the last byte
  I2C_GenerateSTOP(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
  data = I2C_ReceiveData();

  // Re-enable ACK for future multi-byte reads
  I2C_AcknowledgeConfig(I2C_ACK_CURR);

  return data;
}

void readTemp() {
  uint8_t msb = MPU_ReadReg(0x41);
  uint8_t lsb = MPU_ReadReg(0x42);

  // Combine into signed 16-bit
  int16_t raw_temp = (int16_t)((msb << 8) | lsb);

  // Convert to float (or milli-Celsius to avoid floats on STM8)
  int32_t temp_mc = ((int32_t)raw_temp * 100 / 34) + 36530; 

  printf("Temp: %ld.%ld C\n", temp_mc / 1000, (temp_mc % 1000) / 100);
}

void initPWM(void){
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);

  TIM1_TimeBaseInit(15, TIM1_COUNTERMODE_UP, 999, 0);
  TIM2_TimeBaseInit(TIM2_PRESCALER_16, 999);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
      TIM1_OUTPUTSTATE_ENABLE,
      TIM1_OUTPUTNSTATE_DISABLE,
      0,
      TIM1_OCPOLARITY_HIGH,
      TIM1_OCNPOLARITY_HIGH,
      TIM1_OCIDLESTATE_SET,
      TIM1_OCNIDLESTATE_RESET);
  TIM1_OC4Init( 
      TIM1_OCMODE_PWM1,
      TIM1_OUTPUTSTATE_ENABLE,
      0,
      TIM1_OCPOLARITY_HIGH,
      TIM1_OCIDLESTATE_SET);
  TIM2_OC2Init(
      TIM2_OCMODE_PWM1,
      TIM2_OUTPUTSTATE_ENABLE,
      0,
      TIM2_OCPOLARITY_HIGH);
  TIM2_OC3Init(
      TIM2_OCMODE_PWM1,
      TIM2_OUTPUTSTATE_ENABLE,
      0,
      TIM2_OCPOLARITY_HIGH);

  TIM1_CtrlPWMOutputs(ENABLE);
  TIM2_ARRPreloadConfig(ENABLE) ;

  TIM1_Cmd(ENABLE);
  TIM2_Cmd(ENABLE);
}

void rotors(const uint16_t O1, const uint16_t O2, const uint16_t B1, const uint16_t B2, const uint8_t delay){
  for (int pwm = 0; pwm < 200; pwm++) {
    if (pwm < O1) TIM1_SetCompare3(pwm);
    if (pwm < O2) TIM1_SetCompare4(pwm);
    if (pwm < B1) TIM2_SetCompare2(pwm);
    if (pwm < B2) TIM2_SetCompare3(pwm);
    DelayDumb(delay);
  }
}
