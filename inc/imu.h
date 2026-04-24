#ifndef IMU_H
#define IMU_H

#include "stm8s.h"
#include "stm8s_i2c.h"

#define MPU_SCL_PORT GPIOB
#define MPU_SCL_PIN GPIO_PIN_4
#define MPU_SCK_PORT GPIOB
#define MPU_SCK_PIN GPIO_PIN_5
#define MPU6050_ADDRESS (0x68 << 1)


void initMPU(void);
void MPU_Write(uint8_t reg, uint8_t data);
uint8_t MPU_ReadReg(uint8_t reg);
void readTemp(void);

#endif // !IMU_H
