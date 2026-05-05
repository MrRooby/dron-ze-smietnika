#include "mpu6050_interface.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// ============================================================================
// MPU6050 REGISTER DEFINITIONS
// ============================================================================
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

// ============================================================================
// SENSOR STATE
// ============================================================================

static struct {
  float gyro_offset[3];    // Gyro calibration offset
  float attitude[3];       // Current attitude: roll, pitch, yaw (in 0.1° units)
  float attitude_rad[3];   // Current attitude in radians
  uint32_t last_read_ms;
  uint8_t healthy;
  uint8_t calibrated;
} sensor_state = {
  .gyro_offset = {0, 0, 0},
  .attitude = {0, 0, 0},
  .attitude_rad = {0, 0, 0},
  .last_read_ms = 0,
  .healthy = 0,
  .calibrated = 0
};

// ============================================================================
// I2C UTILITY FUNCTIONS
// ============================================================================

static void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
  delay(1);
}

static uint8_t i2c_read_byte(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)addr, (size_t)1, true);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

static void i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)addr, (size_t)len, true);
  for (uint8_t i = 0; i < len; i++) {
    if (Wire.available()) {
      buf[i] = Wire.read();
    }
  }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

uint8_t mpu6050_init() {
  if (DEBUG_ENABLED) {
    Serial.println("[MPU6050] Initializing sensor...");
  }
  
  // Initialize I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_FREQ);
  
  delay(100);  // Wait for sensor to stabilize
  
  // Test communication: read WHO_AM_I register (0x75, should return 0x68)
  uint8_t who_am_i = i2c_read_byte(MPU6050_ADDR, 0x75);
  if (who_am_i != 0x98) {
    if (DEBUG_ENABLED) {
      Serial.printf("[MPU6050] ERROR: WHO_AM_I = 0x%02X (expected 0x68)\n", who_am_i);
    }
    sensor_state.healthy = 0;
    return 0;
  }
  
  // Wake up MPU6050 (disable sleep mode)
  i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
  delay(50);
  
  // Set sampling rate divider (default 0 = 8kHz / (1 + 0) = 8kHz)
  i2c_write_byte(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 0);
  
  // Set low-pass filter configuration (register 0x1A)
  // Bits [2:0] = filter bandwidth
  // 0 = 260Hz, 1 = 184Hz, 2 = 94Hz, 3 = 44Hz, 4 = 21Hz, 5 = 10Hz, 6 = 5Hz
  i2c_write_byte(MPU6050_ADDR, MPU6050_CONFIG, 1);  // 184Hz LPF
  
  // Set gyroscope range to ±250°/s (register 0x1B, bits [4:3])
  // 0 = ±250, 1 = ±500, 2 = ±1000, 3 = ±2000
  i2c_write_byte(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);
  
  // Set accelerometer range to ±2g (register 0x1C, bits [4:3])
  // 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g
  i2c_write_byte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);
  
  delay(100);
  
  sensor_state.healthy = 1;
  sensor_state.last_read_ms = millis();
  
  if (DEBUG_ENABLED) {
    Serial.println("[MPU6050] ✓ Sensor initialized (WHO_AM_I = 0x68)");
    Serial.println("[MPU6050] Range: ±2g accel, ±250°/s gyro");
    Serial.println("[MPU6050] Running gyro calibration...");
  }
  
  // Calibrate gyro
  mpu6050_calibrate_gyro();
  sensor_state.calibrated = 1;
  
  if (DEBUG_ENABLED) {
    Serial.printf("[MPU6050] Gyro offset: X=%.2f Y=%.2f Z=%.2f\n",
      sensor_state.gyro_offset[0], sensor_state.gyro_offset[1], sensor_state.gyro_offset[2]);
  }
  
  return 1;
}

// ============================================================================
// SENSOR READING
// ============================================================================

uint8_t mpu6050_read_raw(IMU_Data *imu) {
  if (!imu || !sensor_state.healthy) {
    return 0;
  }
  
  // Read 14 bytes: 6 accel (0x3B-0x40) + 2 temp (0x41-0x42) + 6 gyro (0x43-0x48)
  uint8_t raw_data[14];
  i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, raw_data, 14);
  
  // Parse accelerometer (16-bit big-endian)
  imu->accel.x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  imu->accel.y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  imu->accel.z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
  
  // Parse temperature
  imu->temp = (int16_t)((raw_data[6] << 8) | raw_data[7]);
  
  // Parse gyroscope (16-bit big-endian)
  imu->gyro.x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
  imu->gyro.y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
  imu->gyro.z = (int16_t)((raw_data[12] << 8) | raw_data[13]);
  
  // Convert to physical units
  // Accelerometer: LSB/g = 512 for ±2g → divide by 512 to get g
  imu->accel_g[0] = imu->accel.x / (float)ACC_1G;
  imu->accel_g[1] = imu->accel.y / (float)ACC_1G;
  imu->accel_g[2] = imu->accel.z / (float)ACC_1G;
  
  // Gyroscope: LSB/°/s = 16.4 for ±250°/s → divide by 16.4 to get °/s
  imu->gyro_dps[0] = (imu->gyro.x / GYRO_SENSITIVITY) - sensor_state.gyro_offset[0];
  imu->gyro_dps[1] = (imu->gyro.y / GYRO_SENSITIVITY) - sensor_state.gyro_offset[1];
  imu->gyro_dps[2] = (imu->gyro.z / GYRO_SENSITIVITY) - sensor_state.gyro_offset[2];
  
  sensor_state.last_read_ms = millis();
  
  if (DEBUG_IMU && DEBUG_ENABLED) {
    // Serial.printf("[MPU6050] ACC=%.2fg,%.2fg,%.2fg GYR=%.1f,%.1f,%.1f dps\n",
      // imu->accel_g[0], imu->accel_g[1], imu->accel_g[2],
      // imu->gyro_dps[0], imu->gyro_dps[1], imu->gyro_dps[2]);
  }
  
  return 1;
}

// ============================================================================
// ATTITUDE ESTIMATION (Complementary Filter)
// ============================================================================

void mpu6050_compute_attitude(const IMU_Data *imu, Attitude *att, float dt) {
  if (!imu || !att) return;
  
  // Extract sensor data
  float ax = imu->accel_g[0];
  float ay = imu->accel_g[1];
  float az = imu->accel_g[2];
  
  float gx = imu->gyro_dps[0] * M_PI / 180.0f;  // Convert to rad/s
  float gy = imu->gyro_dps[1] * M_PI / 180.0f;
  float gz = imu->gyro_dps[2] * M_PI / 180.0f;
  
  // Get current attitude from state (using array indices: 0=roll, 1=pitch, 2=yaw)
  float roll = sensor_state.attitude_rad[0];
  float pitch = sensor_state.attitude_rad[1];
  float yaw = sensor_state.attitude_rad[2];
  
  // ========== GYROSCOPE INTEGRATION (predicts new attitude) ==========
  // Small angle approximation for roll and pitch
  float sin_roll = sinf(roll);
  float cos_roll = cosf(roll);
  float tan_pitch = tanf(pitch);
  
  // Gyro-based attitude rate
  float roll_rate = gx + sin_roll * tan_pitch * gy + cos_roll * tan_pitch * gz;
  float pitch_rate = cos_roll * gy - sin_roll * gz;
  
  // Integrate to get new attitude
  float roll_gyro = roll + roll_rate * dt;
  float pitch_gyro = pitch + pitch_rate * dt;
  float yaw_gyro = yaw + gz * dt;
  
  // ========== ACCELEROMETER CORRECTION (stable, slow) ==========
  // Calculate roll & pitch from accelerometer
  float accel_roll = atan2f(ay, az);
  float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
  
  // ========== COMPLEMENTARY FILTER FUSION ==========
  // High-pass filter (gyro): passes fast changes
  // Low-pass filter (accel): passes slow, stable corrections
  // GYR_CMPF_FACTOR = 10 means: gyro_weight = 1024, accel_weight = 256
  // Ratio = 1024:256 = 4:1 (80% gyro, 20% accel)
  
  uint16_t gyro_weight = (1 << GYR_CMPF_FACTOR);  // 1024
  uint16_t accel_weight = 256;
  uint16_t total_weight = gyro_weight + accel_weight;
  
  // Blend with error wrapping for angle discontinuities
  float roll_error = accel_roll - roll_gyro;
  if (roll_error > M_PI) roll_error -= 2*M_PI;
  if (roll_error < -M_PI) roll_error += 2*M_PI;
  
  float pitch_error = accel_pitch - pitch_gyro;
  if (pitch_error > M_PI) pitch_error -= 2*M_PI;
  if (pitch_error < -M_PI) pitch_error += 2*M_PI;
  
  // Apply weighted correction
  sensor_state.attitude_rad[0] = roll_gyro + (roll_error * accel_weight) / total_weight;
  sensor_state.attitude_rad[1] = pitch_gyro + (pitch_error * accel_weight) / total_weight;
  sensor_state.attitude_rad[2] = yaw_gyro;  // Gyro only for yaw (no accel data)
  
  // Store in final attitude struct (indices match RC_Channel for compatibility)
  att->angle_rad[ROLL] = sensor_state.attitude_rad[0];
  att->angle_rad[PITCH] = sensor_state.attitude_rad[1];
  att->angle_rad[YAW] = sensor_state.attitude_rad[2];
  
  // Convert to 0.1° units (multiply by 180/π * 10)
  att->angle[ROLL] = (int16_t)(sensor_state.attitude_rad[0] * 573.0f);   // 180/π * 10 ≈ 573
  att->angle[PITCH] = (int16_t)(sensor_state.attitude_rad[1] * 573.0f);
  att->heading = (uint16_t)((sensor_state.attitude_rad[2] * 180.0f / M_PI) + 360) % 360;
}

// ============================================================================
// ATTITUDE ACCESSORS
// ============================================================================

int16_t mpu6050_get_roll() {
  return (int16_t)(sensor_state.attitude_rad[0] * 573.0f);
}

int16_t mpu6050_get_pitch() {
  return (int16_t)(sensor_state.attitude_rad[1] * 573.0f);
}

uint16_t mpu6050_get_yaw() {
  return (uint16_t)((sensor_state.attitude_rad[2] * 180.0f / M_PI) + 360) % 360;
}

// ============================================================================
// CALIBRATION
// ============================================================================

void mpu6050_calibrate_gyro() {
  if (DEBUG_ENABLED) {
    Serial.println("[MPU6050] Gyro calibration - keeping drone LEVEL...");
  }
  
  float gyro_sum[3] = {0, 0, 0};
  uint16_t samples = 100;
  
  for (uint16_t i = 0; i < samples; i++) {
    IMU_Data imu = {0};
    mpu6050_read_raw(&imu);
    
    gyro_sum[0] += imu.gyro_dps[0];
    gyro_sum[1] += imu.gyro_dps[1];
    gyro_sum[2] += imu.gyro_dps[2];
    
    delay(10);
  }
  
  sensor_state.gyro_offset[0] = gyro_sum[0] / samples;
  sensor_state.gyro_offset[1] = gyro_sum[1] / samples;
  sensor_state.gyro_offset[2] = gyro_sum[2] / samples;
  
  if (DEBUG_ENABLED) {
    Serial.println("[MPU6050] ✓ Gyro calibration complete");
  }
}

void mpu6050_calibrate_accel() {
  // Placeholder: More complex calibration not implemented yet
  if (DEBUG_ENABLED) {
    Serial.println("[MPU6050] Accel calibration skipped (not implemented)");
  }
}

void mpu6050_reset_attitude() {
  sensor_state.attitude[0] = 0;
  sensor_state.attitude[1] = 0;
  sensor_state.attitude[2] = 0;
  sensor_state.attitude_rad[0] = 0;
  sensor_state.attitude_rad[1] = 0;
  sensor_state.attitude_rad[2] = 0;
}

// ============================================================================
// STATUS
// ============================================================================

uint8_t mpu6050_is_healthy() {
  return sensor_state.healthy;
}
