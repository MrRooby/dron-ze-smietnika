#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

// ============================================================================
// ENUMERATIONS
// ============================================================================

enum Axis {
  AXIS_ROLL = 0,
  AXIS_PITCH = 1,
  AXIS_YAW = 2
};

enum RC_Channel {
  THROTTLE = 0,
  ROLL = 1,
  PITCH = 2,
  YAW = 3,
  AUX1 = 4,
  AUX2 = 5
};

enum FlightMode {
  MODE_RATE = 0,      // Manual rate control
  MODE_ACRO = 1,      // Acrobatic (no angle limit)
  MODE_STABILIZE = 2  // Assisted (angle limit + leveling)
};

// ============================================================================
// IMU DATA STRUCTURES
// ============================================================================

typedef struct {
  int16_t x, y, z;      // Raw 16-bit values from MPU6050
} Sensor_Raw;

typedef struct {
  Sensor_Raw accel;      // Accelerometer raw (±2g range, 512 LSB/g)
  Sensor_Raw gyro;       // Gyroscope raw (±250°/s range, 16.4 LSB/°/s)
  int16_t temp;          // Temperature raw
  float accel_g[3];      // Converted to g (x, y, z)
  float gyro_dps[3];     // Converted to degrees/second (x, y, z)
} IMU_Data;

typedef struct {
  float angle[3];        // Roll, Pitch, Yaw in 0.1° units (indices: ROLL, PITCH, YAW from RC_Channel)
  float angle_rad[3];    // Roll, Pitch, Yaw in radians
  float heading;         // Absolute heading (0-360°)
} Attitude;

// ============================================================================
// RC (REMOTE CONTROL) STRUCTURES
// ============================================================================

// Raw NRF24 packet received from transmitter (6 bytes)
typedef struct {
  uint8_t throttle;      // [0-255]
  uint8_t yaw;           // [0-255]
  uint8_t pitch;         // [0-255]
  uint8_t roll;          // [0-255]
  uint8_t AUX1;          // [0-255]
  uint8_t AUX2;          // [0-255]
} RF24_Packet;


// Processed RC command values (PWM microseconds range)
typedef struct {
  uint16_t rcData[6];        // RAW: [1000-2000] µs for each channel
  uint16_t rcCommand[4];     // PROCESSED: [1000-2000] for throttle, [-500...+500] for axes
  uint8_t flight_mode;       // Current flight mode
  bool armed;                // Arm state
  uint32_t last_signal_ms;   // Timestamp of last valid RF24 packet
  bool signal_lost;          // True if signal lost for > RF24_FAILSAFE_TIME
} RC_State;

// ============================================================================
// MOTOR STRUCTURES
// ============================================================================

typedef struct {
  uint16_t value[4];     // PWM values for motors [0-3]: FL, FR, BL, BR [1000-2000]
  uint8_t armed;         // True if drone is armed and can spin motors
} Motor_Output;

// ============================================================================
// PID CONTROLLER STRUCTURES
// ============================================================================

typedef struct {
  float P;               // Proportional gain
  float I;               // Integral gain
  float D;               // Derivative gain
} PID_Coefficients;

typedef struct {
  PID_Coefficients coef;
  float error_prev;      // Previous error for derivative
  float integral;        // Accumulated integral error
  float output;          // Latest PID output
} PID_Controller;

// ============================================================================
// FLIGHT STATE STRUCTURE (combines all state)
// ============================================================================

typedef struct {
  // Sensors
  IMU_Data imu;
  Attitude att;
  
  // Remote Control
  RC_State rc;
  
  // Motor Output
  Motor_Output motors;
  
  // PID Controllers
  PID_Controller pid[3];  // [0]=ROLL, [1]=PITCH, [2]=YAW
  float axisPID[3];       // PID outputs for mixing
  
  // Status
  uint32_t loop_count;
  uint32_t loop_time_us;  // Actual loop execution time (microseconds)
  bool healthy;           // Overall system health
  
} Flight_State;

// ============================================================================
// TELEMETRY STRUCTURES (for ACK payload to transmitter)
// ============================================================================

typedef struct {
  int16_t roll;          // Roll angle in 0.1° units
  int16_t pitch;         // Pitch angle in 0.1° units
  uint16_t yaw;          // Yaw heading in degrees
  uint8_t armed : 1;     // Arm status
  uint8_t healthy : 1;   // System health
  uint8_t rssi : 6;      // Signal strength (0-63)
} Telemetry_Packet;

#endif // TYPES_H
