#include "imu.h"
#include "stm8s_gpio.h"
#include "stm8s_i2c.h"
#include "timing.h"
#include <stdint.h>
#include <stdio.h>

int16_t gyro_bias[3] = {0,0,0};
Attitude att = {{0,0}};

// void IMU_Init(void){
//   CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
//   // GPIO_Init(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_);
//
//   I2C_DeInit();                              
//   I2C_Init(
//       100000,           // 100kHz I2C Clock
//       0x00,             // Own address
//       I2C_DUTYCYCLE_2,               
//       I2C_ACK_CURR,                   
//       I2C_ADDMODE_7BIT,
//       16                            
//   );
//   I2C_Cmd(ENABLE);
//
//   I2C_ClearFlag(I2C_FLAG_BUSBUSY);
//   while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY)){
//     printf("I2C busy\n");
//   }
//
//   // Wake up the IMU
//   MPU_Write(PWR_MGMT_1, 0x01); 
//   // Set DLPF to 42Hz
//   MPU_Write(CONFIG, 0x03);
//   // Sample rate of 1kHz
//   MPU_Write(SMPLRT_DIV, 0x00);
//   // Gyroscope +-500 deg/s
//   MPU_Write(GYRO_CONFIG, 0x08);
//   // Accel +-8g
//   MPU_Write(ACCEL_CONFIG, 0x10);
// }

void IMU_Init(void) {
    // 1. Reset standard GPIOs
    GPIO_Init(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST);
    
    // Set both high. If analyzer is LOW here, pull-ups are bad or chip is dead.
    GPIO_WriteHigh(GPIOB, GPIO_PIN_4); 
    GPIO_WriteHigh(GPIOB, GPIO_PIN_5); 
    Delay(1000); // 1 second delay - check logic analyzer! It MUST be high here.

    // 2. Hardware I2C Clock ON
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);

    // 3. Reset the I2C block
    I2C->CR2 |= I2C_CR2_SWRST; 
    Delay(100); 
    I2C->CR2 &= ~I2C_CR2_SWRST;
    Delay(100);

    // 4. Configure settings (Peripheral is still disabled here)
    I2C_Init(100000, 0x00, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);

    Delay(1000); // 1 second delay - check logic analyzer! It MUST still be high here.

    // 5. This is the danger zone. Enable the peripheral to take over the pins.
    I2C_Cmd(ENABLE);

    // // Wait and check if the peripheral crashes and yanks the lines low
    // while(1) {
    //     printf("Alive! Is logic analyzer High or Low?\n");
    //     Delay(500);
    // }
    MPU_Write(PWR_MGMT_1, 0x01); 
}

void MPU_Write(uint8_t reg, uint8_t data) {
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

void readTemp(void) {
  uint8_t msb = MPU_ReadReg(0x41);
  uint8_t lsb = MPU_ReadReg(0x42);

  // Combine into signed 16-bit
  int16_t raw_temp = (int16_t)((msb << 8) | lsb);

  // Convert to float (or milli-Celsius to avoid floats on STM8)
  int32_t temp_mc = ((int32_t)raw_temp * 100 / 34) + 36530; 

#ifdef DEBUG
  printf("Temp: %ld.%ld C\n", temp_mc / 1000, (temp_mc % 1000) / 100);
#endif /* ifdef DEBUG */
}

void IMU_ReadBurst(IMUData *data) {
  uint8_t buffer[14];

  // 1. Send start and register address
  I2C_GenerateSTART(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(MPU6050_ADDRESS, I2C_DIRECTION_TX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(ACCEL_XOUT_H);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  // 2. Restart for Read
  I2C_GenerateSTART(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(MPU6050_ADDRESS, I2C_DIRECTION_RX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  // 3. Sequential read
  for(int i = 0; i < 14; i++) {
    if(i == 13) {
      I2C_AcknowledgeConfig(I2C_ACK_NONE); // NACK last byte
      I2C_GenerateSTOP(ENABLE);
    }
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
    buffer[i] = I2C_ReceiveData();
  }

  // 4. Parse buffer (skipping temp bytes 6 and 7)
  data->ax = (int16_t)((buffer[0] << 8) | buffer[1]);
  data->ay = (int16_t)((buffer[2] << 8) | buffer[3]);
  data->az = (int16_t)((buffer[4] << 8) | buffer[5]);

  data->gx = (int16_t)((buffer[8] << 8) | buffer[9]);
  data->gy = (int16_t)((buffer[10] << 8) | buffer[11]);
  data->gz = (int16_t)((buffer[12] << 8) | buffer[13]);

  I2C_AcknowledgeConfig(I2C_ACK_CURR); // Restore for next time
}

void computeIMU(void) {
  IMUData data1, data2;
  int16_t gyro[3];
  static int32_t accSmooth[3] = {0, 0, 0};
  int16_t accAngle[2];
  uint32_t timeInterleave;

  IMU_ReadBurst(&data1);
  timeInterleave = micros();
  while((int16_t)(micros()-timeInterleave)<650);
  IMU_ReadBurst(&data2);

  // Averaged Gyro data with calibration offset
  gyro[ROLL]  = ((data1.gx + data2.gx) >> 1) - gyro_bias[ROLL];
  gyro[PITCH] = ((data1.gy + data2.gy) >> 1) - gyro_bias[PITCH];

  // Low pass filter 
  accSmooth[0] = ((accSmooth[0] * 3) + data1.ax) >> 2;
  accSmooth[1] = ((accSmooth[1] * 3) + data1.ay) >> 2;
  accSmooth[2] = ((accSmooth[2] * 3) + data1.az) >> 2;

  // Calculate Accelerometer Angles (Simplified atan2 for small angles)
  // MultiWii uses a lookup table or atan2. For STM8, degrees*10 is standard.
  accAngle[ROLL]  = _atan2(accSmooth[1], accSmooth[2]);
  accAngle[PITCH] = _atan2(accSmooth[0], accSmooth[2]);

  int16_t gyroDelta = (int16_t)(((int32_t)gyro[ROLL] * 10) / 6553);
  att.angle[ROLL] = ((GYR_CMPF_FACTOR * (att.angle[ROLL] + gyroDelta)) + accAngle[ROLL]) / (GYR_CMPF_FACTOR + 1);
  gyroDelta = (int16_t)(((int32_t)gyro[PITCH] * 10) / 6553);
  att.angle[PITCH] = ((GYR_CMPF_FACTOR * (att.angle[PITCH] + gyroDelta)) + accAngle[PITCH]) / (GYR_CMPF_FACTOR + 1);
}


int16_t _atan2(int32_t y, int32_t x){
  int32_t abs_y = y < 0 ? -y : y;
  int32_t abs_x = x < 0 ? -x : x;
  int32_t angle = 0;

  if (abs_y == 0 && abs_x == 0) return 0;

  if(abs_x >= abs_y){
    angle = (450 * abs_y) / abs_x;
  } else {
    angle = 900 - (450 * abs_x) / abs_y;
  }

  if (x < 0) angle = 1800 - angle;
  if (y < 0) angle = -angle;

  return (int16_t)angle;
}

void IMU_Calibrate(const uint16_t samples){
  int32_t gyro_sum[3] = {0, 0, 0};
    IMUData temp_data;

    for (uint16_t i = 0; i < samples; i++) {
      IMU_ReadBurst(&temp_data);

      gyro_sum[0] += temp_data.gx;
      gyro_sum[1] += temp_data.gy;
      gyro_sum[2] += temp_data.gz;

      // Short delay to allow the sensor to refresh
      Delay(2); 
    }

    // Average the results
    gyro_bias[0] = (int16_t)(gyro_sum[0] / samples);
    gyro_bias[1] = (int16_t)(gyro_sum[1] / samples);
    gyro_bias[2] = (int16_t)(gyro_sum[2] / samples);
}
