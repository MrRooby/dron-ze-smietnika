#ifndef PINS_H
#define PINS_H

// ============================================================================
// ESP32-C3 PINOUT & HARDWARE CONFIGURATION
// ============================================================================

// Note: ESP32-C3 has 22 GPIO pins (GPIO0-GPIO22)
// Pin names refer to board silkscreen (A0=GPIO0, etc.)

// ============================================================================
// I2C - MPU6050 CONNECTION (Wire/I2C0)
// ============================================================================
#define I2C_SDA_PIN 8       // GPIO8 = D2 on board
#define I2C_SCL_PIN 9       // GPIO9 = D1 on board
#define I2C_FREQ 400000     // 400 kHz I2C bus speed

// ============================================================================
// SPI - NRF24L01 CONNECTION (HSPI)
// ============================================================================
// Note: SPI pins can be configured on ESP32-C3
#define SPI_SCK_PIN 4       // GPIO4  = SCK
#define SPI_MOSI_PIN 6      // GPIO6  = MOSI
#define SPI_MISO_PIN 5      // GPIO5  = MISO

// NRF24 Control lines
#define NRF24_CSN_PIN 7     // GPIO7  = Slave Select (active LOW)
#define NRF24_CE_PIN 10     // GPIO10 = Chip Enable (active HIGH)

// Optional: NRF24 IRQ not used (pull-up on module)
// #define NRF24_IRQ_PIN -1

// ============================================================================
// PWM MOTOR OUTPUT PINS (LEDC)
// ============================================================================
// ESP32-C3 PWM Motors via GPIO pins
// Format: GPIO pin for each motor (FL, FR, BL, BR)

#define MOTOR_FL_PIN 0      // A0 = GPIO0  - Front-Left
#define MOTOR_FR_PIN 1      // A1 = GPIO1  - Front-Right
#define MOTOR_BL_PIN 2      // A2 = GPIO2  - Back-Left
#define MOTOR_BR_PIN 3      // A3 = GPIO3  - Back-Right

// Motor array indices
#define MOTOR_FRONT_LEFT 0
#define MOTOR_FRONT_RIGHT 1
#define MOTOR_BACK_LEFT 2
#define MOTOR_BACK_RIGHT 3

// ============================================================================
// SERIAL DEBUG PORT
// ============================================================================
#define SERIAL_TX_PIN 21    // GPIO21 (default USB CDC)
#define SERIAL_RX_PIN 20    // GPIO20 (default USB CDC)
// Note: USB CDC is handled by ARDUINO_USB_CDC_ON_BOOT=1
// Use Serial.begin(115200) for debug output

// ============================================================================
// LEDC PWM CHANNEL ALLOCATION
// ============================================================================
// ESP32-C3 has 6 independent LEDC channels (0-5)
// We use channels 0-3 for the 4 motors

#define LEDC_CHANNEL_FL 0   // Front-Left motor
#define LEDC_CHANNEL_FR 1   // Front-Right motor
#define LEDC_CHANNEL_BL 2   // Back-Left motor
#define LEDC_CHANNEL_BR 3   // Back-Right motor

// LEDC Timer allocation (LOW speed timer)
#define LEDC_TIMER_IDX 0    // Use LEDC_LOW_SPEED_TIMER == 0

// ============================================================================
// UNUSED/AVAILABLE PINS
// ============================================================================
// GPIO 4, 11-19, 21-22 are available for future use
// (Note: GPIO8-9 used for I2C, GPIO5-7 for SPI, GPIO0-3 for motors, GPIO10,20 for NRF24)

// ============================================================================
// HARDWARE INFORMATION
// ============================================================================
#define BOARD_NAME "ESP32-C3 Super Mini"
#define CPU_FREQ 160         // MHz (can be 80 or 160)
#define FLASH_SIZE 4         // MB

#endif // PINS_H
