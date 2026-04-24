#include "imu.h"

void IMU_Init(void){
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
  GPIO_Init(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST);

  I2C_DeInit();                              
  I2C_Init(
      400000,           // 400kHz I2C Clock
      0x01,             // Own address
      I2C_DUTYCYCLE_2,               
      I2C_ACK_CURR,                   
      I2C_ADDMODE_7BIT,
      16                            
  );
  I2C_Cmd(ENABLE);

  // Wake up the IMU
  MPU_Write(PWR_MGMT_1, 0x01); 
  // Set DLPF to 42Hz
  MPU_Write(CONFIG, 0x03);
  // Sample rate of 1kHz
  MPU_Write(SMPLRT_DIV, 0x00);
  // Gyroscope +-500 deg/s
  MPU_Write(GYRO_CONFIG, 0x08);
  // Accel +-8g
  MPU_Write(ACCEL_CONFIG, 0x10);
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

void readTemp() {
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

void computeIMU () {
  IMUData data_first = {0,0,0,0,0,0}; 
  IMUData data_second = {0,0,0,0,0,0}; 
  IMUData data_final = {0,0,0,0,0,0}; 

  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  static int16_t gyroADCinter[3];

  uint16_t timeInterleave = 0;
  IMU_ReadBurst(&data_first);
  
  timeInterleave = micros();
  uint8_t t = 0;
  while((int16_t)(micros()-timeInterleave)<650) t=1; //empirical, interleaving delay between 2 consecutive reads
  IMU_ReadBurst(&data_second);

  for (axis = 0; axis < 3; axis++) {

    gyroADCinter[axis] =  imu.gyroADC[axis]+gyroADCinter[axis];
    // empirical, we take a weighted value of the current and the previous values
    imu.gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
    gyroADCprevious[axis] = gyroADCinter[axis]>>1;
    if (!ACC) imu.accADC[axis]=0;
  }
}

