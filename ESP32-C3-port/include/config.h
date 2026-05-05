#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
#define LOOP_TIME 2800  // Main loop period in microseconds (357 Hz)

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================
#define MINTHROTTLE 1000   // ESC minimum signal (1ms pulse = stop)
#define MAXTHROTTLE 2000   // ESC maximum signal (2ms pulse = full throttle)
#define MIDTHROTTLE 1500   // Center throttle position

// Motor PWM frequency (50Hz for ESC compatibility)
#define MOTOR_PWM_FREQ 50
// Motor PWM resolution (14-bit)
#define MOTOR_PWM_RESOLUTION 14
// Motor PWM duty max (for 16-bit: 65535)
#define MOTOR_PWM_MAX ((1 << MOTOR_PWM_RESOLUTION) - 1)

// ============================================================================
// RADIO (NRF24) CONFIGURATION
// ============================================================================
#define RF24_CHANNEL 76        // 2.4 GHz channel (76 = 2476 MHz)
#define RF24_DATARATE RF24_250KBPS  // Lowest rate for maximum range
#define RF24_FAILSAFE_TIME 1000  // ms until failsafe triggers (signal loss)

// ============================================================================
// IMU (MPU6050) CONFIGURATION
// ============================================================================
#define MPU6050_ADDR 0x68       // I2C address
#define ACC_1G 512              // Accelerometer sensitivity (512 LSB/g for ±2g)
#define GYRO_SENSITIVITY 16.4f  // Gyroscope sensitivity (16.4 LSB/°/s for ±250°/s)

// Low-pass filter factor for gyro (higher = more filtering)
#define ACC_LPF_FACTOR 4
#define GYR_LPF_FACTOR 4

// Complementary filter weighting (Gyro weight :: Accel weight = 1024 :: 256)
#define GYR_CMPF_FACTOR 10      // log2(1024/256) = log2(4) = 2, so 2^10 = 1024

// ============================================================================
// PID CONTROLLER DEFAULTS
// ============================================================================
// ROLL PID
#define PID_ROLL_P 1.30f
#define PID_ROLL_I 0.10f
#define PID_ROLL_D 23.00f

// PITCH PID
#define PID_PITCH_P 1.30f
#define PID_PITCH_I 0.10f
#define PID_PITCH_D 23.00f

// YAW PID
#define PID_YAW_P 3.00f
#define PID_YAW_I 0.10f
#define PID_YAW_D 0.00f

// ============================================================================
// ARMING/DISARMING CONFIGURATION
// ============================================================================
#define STICK_ARM_DELAY 500     // ms to hold sticks in arm position
#define STICK_DISARM_DELAY 1000 // ms to hold sticks in disarm position

// Stick position thresholds for arm/disarm detection
#define ARM_THROTTLE_MAX 1050   // Throttle must be < 1050
#define ARM_YAW_MIN 1850        // Yaw must be > 1850 to arm
#define ARM_YAW_MAX 2100        // Yaw range for arming

#define DISARM_THROTTLE_MAX 1050 // Throttle must be < 1050
#define DISARM_YAW_MIN 1900      // Yaw minimum for disarm detection
#define DISARM_YAW_MAX 2100      // Yaw maximum for disarm detection

// ============================================================================
// DEBUG & SERIAL OUTPUT
// ============================================================================
#define SERIAL_BAUD 115200
#define DEBUG_ENABLED 1         // Set to 1 to enable debug output
#define DEBUG_IMU 1             // Print IMU data
#define DEBUG_RC 1              // Print RC commands
#define DEBUG_MOTORS 0          // Print motor values (verbose)
#define DEBUG_TELEMETRY 0       // Print telemetry packets

// ============================================================================
// FRAME CONFIGURATION
// ============================================================================
// Quadcopter frame type (X-configuration)
// Motor layout:
//       Front
//      (0,1)
//     /    \
//   (0)    (1)
//   FL     FR
//    |     |
//   BL     BR
//   (2)    (3)
//     \    /
//      (2,3)
//       Back

#define QUADX 1

// ============================================================================
// MISCELLANEOUS
// ============================================================================
#define YAW_DIRECTION 1  // 1 for normal, -1 to reverse yaw direction

#endif // CONFIG_H
