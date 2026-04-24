# STM8 MultiWii Port - Implementation Checklist

## Phase 1: Core Flight Control (MVP)

### I2C & MPU6050 Driver
- [✓] Configure STM8 I2C peripheral at 400kHz
- [✓] Implement MPU6050 initialization
  - [✓] Set sensitivity: ±8g accelerometer, ±500°/s gyroscope
  - [✓] Configure digital low-pass filter (DLPF)
  - [ ] Set sample rate to 1kHz
- [✓] Read 6-byte accel data from reg 0x3B-0x40
- [✓] Read 6-byte gyro data from reg 0x43-0x48
- [ ] Implement I2C error handling & retry logic

### IMU Processing (Complementary Filter)
- [✓] Implement `computeIMU()` function
- [✓] Dual gyro sampling with 650µs interleaving
- [✓] Accelerometer low-pass filter (ACC_LPF_FACTOR = 4)
- [✓] Gyro/Accel complementary filter fusion (GYR_CMPF_FACTOR = 10)
- [✓] Calculate roll and pitch angles in degrees*100
- [✓] Output: `att.angle[ROLL]`, `att.angle[PITCH]`

### PID Stabilization
- [ ] Implement 3-axis PID controller (ROLL, PITCH, YAW)
- [ ] Default gains (tuned for brushed 5-inch drone):
  - [ ] PIDROLL: P=90, I=30, D=20
  - [ ] PIDPITCH: P=90, I=30, D=20
  - [ ] PIDYAW: P=100, I=20, D=0
- [ ] Anti-windup for integral term
- [ ] Input: attitude error + RC commands
- [ ] Output: `axisPID[ROLL/PITCH/YAW]`

### Motor Mixer (QUADX)
- [✓] Implement `mixTable()` for X-configuration
  - Motor[0] (Rear):  throttle + pitch + yaw
  - Motor[1] (Right): throttle - roll - yaw
  - Motor[2] (Left):  throttle - pitch + yaw
  - Motor[3] (Front): throttle + roll - yaw
- [✓] Constrain output to MINTHROTTLE (1000) - MAXTHROTTLE (2000)
- [ ] Implement motor start failsafe

### PWM Motor Output (4x Timers)
- [ ] Configure timer frequency for 50Hz PWM (20ms period)
- [✓] Map motors to STM8 timers:
  - [✓] Motor 0: TIM1_CH1 or similar (Rear)
  - [✓] Motor 1: TIM1_CH2 (Right)
  - [✓] Motor 2: TIM2_CH1 (Left)
  - [✓] Motor 3: TIM2_CH2 (Front)
- [✓] Implement 1000-2000µs pulse width control
- [ ] Test each motor individually (prop-off first!)

### NRF24L01 Receiver
- [ ] Configure STM8 SPI for 10MHz (CE=PD3, CSN=PD2)
- [ ] Implement minimal RF24 driver
  - [ ] `NRF24_Init()` - Set to RX mode, pipe address 0xE8E8F0F0E1LL, 250kbps
  - [ ] `NRF24_Read_RC()` - Poll for packets, extract throttle/roll/pitch/yaw/AUX
  - [ ] Timeout detection (>1000ms = signal loss)
- [ ] Map 0-255 radio values to 1000-2000 PWM range
- [ ] Arm/disarm logic (AUX1 channel typically controls arm)

### Main Control Loop
- [ ] Timer/micros() implementation for 2800µs cycle timing
- [ ] Loop execution order:
  1. Read NRF24 RC input
  2. Read MPU6050 sensors
  3. Compute complementary filter
  4. Calculate PID outputs
  5. Mix motors
  6. Write PWM
  7. Process serial MSP commands
  8. Wait for consistent timing
- [ ] Implement cycle time tracking (for debugging)

---

## Phase 2: MultiWii Serial Protocol (PC GUI Integration)

### UART + MSP Protocol
- [ ] Implement circular RX/RX buffers (128-256 bytes)
- [ ] MSP packet parser (format: `$M` + cmd_type + length + data + XOR_checksum)
- [ ] Implement serialization functions:
  - [ ] `serialize8(uint8_t)` - Append with checksum
  - [ ] `serialize16(int16_t)` - Little-endian + checksum
  - [ ] `serialize32(uint32_t)` - Little-endian + checksum
  - [ ] `read8()`, `read16()`, `read32()` - Parse payload

### Required MSP Commands (Minimum)
- [ ] **MSP_IDENT (100)** - Response: multitype + version + protocol
- [ ] **MSP_STATUS (101)** - Response: cycleTime + errors + sensors + flags
- [ ] **MSP_RAW_IMU (102)** - Response: 9 int16_t values (accel[3] + gyro[3] + mag[3])
- [ ] **MSP_MOTOR (104)** - Response: 8 int16_t motor PWM values
- [ ] **MSP_RC (105)** - Response: 8 int16_t RC channel values (1000-2000)
- [ ] **MSP_ATTITUDE (108)** - Response: roll, pitch, yaw angles
- [ ] **MSP_ANALOG (110)** - Response: battery voltage (vbat), RSSI, etc.

### Optional MSP Commands (For Tuning)
- [ ] **MSP_PID (112)** - Response: 9*3 bytes of PID coefficients
- [ ] **MSP_SET_PID (202)** - Accept: 9*3 bytes PID updates (in-flight tuning)
- [ ] **MSP_SET_RAW_RC (200)** - Accept: 8 RC values (for test/debug)
- [ ] **MSP_EEPROM_WRITE (250)** - Command: save config to FLASH

### Serial Integration
- [ ] Adapt existing STM8 UART (from custom_lib/serial.c)
- [ ] Non-blocking transmit via ring buffer
- [ ] RX interrupt handler for bytes
- [ ] MSP frame detection (wait for `$M>` or `$M!`)

---

## Phase 3: Configuration & Polish

### EEPROM/FLASH Storage
- [ ] Allocate 256-512 bytes in STM8 FLASH for config
- [ ] Implement `read_config()` at startup
- [ ] Implement `write_config()` on MSP_EEPROM_WRITE (250)
- [ ] Default PID gains (hardcoded fallback)

### Flight Modes / AUX Channels
- [ ] ARM/DISARM via AUX1 channel (>1500µs = armed)
- [ ] ANGLE mode via AUX2 (self-leveling, locks roll/pitch to ±45°)
- [ ] HORIZON mode (partial self-leveling)

### Failsafe
- [ ] Detect signal loss (>1000ms without NRF24 data)
- [ ] Set throttle to FAILSAFE_THROTTLE (1000 = cut motors)
- [ ] Handle recovery (resume control when signal returns)

### Calibration
- [ ] Gyro zero-point calibration (power-on routine)
- [ ] Accelerometer calibration (optional via MSP)
- [ ] Motor order verification routine

### Status Indicators
- [ ] LED on MCU (armed = solid, disarmed = blink)
- [ ] Serial status messages (debug output via MSP_DEBUGMSG if space allows)

---

## Testing & Validation

### Lab Testing (No Props)
- [ ] [ ] Verify UART communication with MultiWii GUI
- [ ] [ ] Check MSP_IDENT response identifies as MultiWii
- [ ] [ ] Stream MSP_RAW_IMU and verify sensor values make sense
- [ ] [ ] Rotate drone and watch roll/pitch angles change
- [ ] [ ] Test each motor individually (one-by-one command)
- [ ] [ ] Verify RC input parsing (plot stick movements on GUI)
- [ ] [ ] Arm/disarm via AUX channel

### Flight Testing (With Props)
- [ ] [ ] Start hover at 50% gain settings
- [ ] [ ] Test stabilization (small disturbances)
- [ ] [ ] Gradually increase gain settings
- [ ] [ ] Test all flight modes (ANGLE, HORIZON)
- [ ] [ ] Test signal loss failsafe (turn off TX)
- [ ] [ ] Compare with Arduino version performance

---

## Code Statistics Target

| Component | Target LOC | Status |
|-----------|------------|--------|
| IMU (sensors.c) | 200 | |
| Complementary Filter (imu.c) | 150 | |
| PID (pid.c) | 100 | |
| Motor Mixer (motors.c) | 80 | |
| NRF24 Driver (nrf24.c) | 250 | |
| MSP Protocol (msp_protocol.c) | 400 | |
| Main Loop (main.c) | 200 | |
| I2C Driver (i2c.c) | 150 | |
| **TOTAL** | **~1,500 LOC** | |

**Target**: Fit in STM8S005K6T6 (32KB Flash, 2KB RAM) with room for debugging

---

## Known Gotchas & Tips

1. **RAM is tight (2KB)**: Avoid large arrays; use static allocations carefully
2. **No FPU**: Use fixed-point math (multiply by 100 or 1000 before divide)
3. **I2C can stall**: Implement timeout on I2C reads, reset bus if hung
4. **MSP protocol is critical**: Must respond to MSP_IDENT first or GUI won't connect
5. **Timer timing**: Keep main loop exactly 2800µs for consistent control
6. **NRF24 SPI**: Must disable I2C interrupts during SPI transactions to avoid bus conflicts
7. **PWM frequency**: 50Hz standard (20ms period) for ESC compatibility
8. **Motor scaling**: 1000-2000µs range, not Arduino PWM 0-255

---

## Useful Commands (Development)

```bash
# Compile
make

# Program STM8
make flash

# Serial monitor (check MSP transmission)
screen /dev/ttyUSB0 115200

# Verify IMU data
# Use MultiWii GUI to connect and monitor raw sensor values

# Test motor individually
# Upload motor test firmware w/ single motor on AUX1
```

---

## Quick Links
- Arduino Source: `/arduino/MultiWii_NRF24_Brushed_Drone_V5_Code/MultiWii_RF24/`
- STM8 Lib: `/lib/STM8S_StdPeriph_Driver/`
- STM8 UART: `/custom_lib/serial.c` (existing implementation)
- Documentation: `/STM8_MULTIWII_PORT_SUMMARY.md` (main reference)

