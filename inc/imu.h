#ifndef IMU_H
#define IMU_H

#include "stm8s.h"
#include "stm8s_i2c.h"
#include "timing.h"

#define MPU_SCL_PORT GPIOB
#define MPU_SCL_PIN GPIO_PIN_4
#define MPU_SCK_PORT GPIOB
#define MPU_SCK_PIN GPIO_PIN_5

#define MPU6050_ADDRESS ((uint8_t)0x68 << 1)
#define PWR_MGMT_1      0x6B
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define SMPLRT_DIV      0x19
#define ACCEL_XOUT_H    0x3B

#define ROLL 0
#define PITCH 1
#define ACC_LPF_FACTOR 4
#define GYR_CMPF_FACTOR 10

#define RAD_TO_DEG 572.95

typedef struct {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} IMUData;

typedef struct {
  int16_t angle[2]; // [0] = ROLL; [1] = PITCH
} Attitude ;

extern Attitude att;

extern int16_t gyro_bias[3];


void IMU_Init(void);
void IMU_ReadBurst(IMUData *data);
void computeIMU(void);
void MPU_Write(uint8_t reg, uint8_t data);
uint8_t MPU_ReadReg(uint8_t reg);
void readTemp(void);
int16_t _atan2(int32_t y, int32_t x);
void IMU_Calibrate(const uint16_t samples);

#endif // !IMU_H
