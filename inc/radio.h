#ifndef RADIO_H
#define RADIO_H
#include "stm8s.h"
#include "stm8s_spi.h"
#include "stm8s_gpio.h"
#include <stdio.h>

#define CSN_PORT GPIOD
#define CSN_PIN  GPIO_PIN_0
#define CE_PORT GPIOA
#define CE_PIN  GPIO_PIN_1

#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07

void NRF24_Init(void);
uint8_t SPI_Exchange(uint8_t data);
void NRF24_Write_Reg(uint8_t reg, uint8_t value);
uint8_t NRF24_Read_Reg(uint8_t reg);
uint8_t NRF24_Get_Status(void);
void NRF24_Test_Connection(void);
void NRF24_WhoAmI(void);


#endif /* ifndef RADIO_H */
