#ifndef __MPU_H
#define __MPU_H

#include <stdint.h>

#define MPU6050_ADDRESS (0x68 << 1)

/**
 * Initialize MPU6050 I2C communication
 * - I2C clock: 100kHz
 * - Standard 7-bit addressing
 */
void mpu_init(void);

/**
 * Write a byte to MPU6050 register
 * @param reg: Register address
 * @param data: Data byte to write
 */
void mpu_write(uint8_t reg, uint8_t data);

/**
 * Read a byte from MPU6050 register
 * @param reg: Register address
 * @return: Data byte read from register
 */
uint8_t mpu_read_reg(uint8_t reg);

/**
 * Read temperature from MPU6050
 * Prints: "Temp: XX.XX C\n" to serial
 */
void mpu_read_temp(void);

#endif
