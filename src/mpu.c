#include "mpu.h"
#include "stm8s_i2c.h"
#include "stm8s_clk.h"
#include "stm8s_gpio.h"
#include <stdio.h>

void mpu_init(void) {
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

void mpu_write(uint8_t reg, uint8_t data) {
  I2C_GenerateSTART(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(MPU6050_ADDRESS, I2C_DIRECTION_TX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(reg);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_SendData(data);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTOP(ENABLE);
}

uint8_t mpu_read_reg(uint8_t reg) {
  uint8_t data;

  // Wait for bus to be idle
  while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));

  // Start + Address (Write Mode)
  I2C_GenerateSTART(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(MPU6050_ADDRESS, I2C_DIRECTION_TX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  // Point to the register
  I2C_SendData(reg);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  // Restart + Address (Read Mode)
  I2C_GenerateSTART(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(MPU6050_ADDRESS, I2C_DIRECTION_RX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  // Read the byte and NACK (since it's only 1 byte)
  I2C_AcknowledgeConfig(I2C_ACK_NONE);
  I2C_GenerateSTOP(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
  data = I2C_ReceiveData();

  // Re-enable ACK for future multi-byte reads
  I2C_AcknowledgeConfig(I2C_ACK_CURR);

  return data;
}

void mpu_read_temp(void) {
  uint8_t msb = mpu_read_reg(0x41);
  uint8_t lsb = mpu_read_reg(0x42);

  // Combine into signed 16-bit
  int16_t raw_temp = (int16_t)((msb << 8) | lsb);

  // Convert to milli-Celsius
  int32_t temp_mc = ((int32_t)raw_temp * 100 / 34) + 36530;

  printf("Temp: %ld.%ld C\n", temp_mc / 1000, (temp_mc % 1000) / 100);
}
