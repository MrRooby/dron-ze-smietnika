# STM8 MultiWii Drone Port - Code Summary

## Overview
Arduino Nano MultiWii drone software with brushed motors (QUADX configuration), MPU6050 IMU sensor, and NRF24L01 receiver needs to be ported to **STM8S005K6T6**. The drone must communicate with MultiWii PC software via serial using the MultiWii Serial Protocol (MSP).

**Key Hardware:**
- MCU: STM8S005K6T6 (16MHz, 32KB Flash, 2KB RAM)
- IMU: MPU6050 (6-DOF via I2C)
- RX: NRF24L01 (SPI, 2.4GHz wireless)
- Motors: 4x brushed DC motors (PWM controlled)
- Serial: Serial protocol for MultiWii GUI communication

---

## 1. SERIAL COMMUNICATION LAYER (MultiWii Protocol)

### Critical Components

#### 1.1 MSP Protocol Implementation (`Protocol.cpp` → `protocol.c/h`)
- **Function**: `serialCom()` - Main serial communication handler
- **Responsibilities**:
  - Decode incoming MSP commands from PC
  - Build and send telemetry responses
  - Packet format: `$M` + command_type + length + data + checksum
  
**MSP Commands to Implement** (most critical):
```
MSP_IDENT (100)        - Send drone identification & version
MSP_STATUS (101)       - Send flight status, cycle time, sensor health
MSP_RAW_IMU (102)      - Send raw 9-DOF sensor data (accel, gyro, mag)
MSP_MOTOR (104)        - Send current motor PWM values
MSP_RC (105)           - Send RC channel data
MSP_ATTITUDE (108)     - Send roll, pitch, yaw angles
MSP_ANALOG (110)       - Send battery voltage, RSSI
MSP_PID (112)          - Send/receive PID coefficients
MSP_SET_RAW_RC (200)   - Receive RC commands from GUI
MSP_EEPROM_WRITE (250) - Store settings to memory
```

#### 1.2 Serial Ring Buffer (`Serial.cpp` → custom_lib/serial.c/h`)
- Implement circular TX/RX buffers (128-256 bytes each)
- ISR-driven UART handling for non-blocking I/O
- Current STM8 Implementation: UART2 at 115200 baud with RX interrupt (see `custom_lib/serial.c`)
- **Modification needed**: Add MSP protocol layer on top of existing UART

#### 1.3 Checksum & Serialization
- XOR checksum for MSP packets
- Helper functions:
  - `serialize8()`, `serialize16()`, `serialize32()` - Append data with checksum update
  - `read8()`, `read16()`, `read32()` - Parse incoming packet data
  - `headSerialResponse()` - Build MSP header
  - `tailSerialReply()` - Add checksum and flush

**Key Implementation Detail**: Single UART port (UART2) shared with MultiWii protocol. Must implement proper framing to distinguish MSP packets from other serial data.

---

## 2. IMU SENSOR HANDLING (MPU6050)

### Critical Components

#### 2.1 I2C Communication (`Sensors.cpp` → sensors.c/h`)
- **Baudrate**: 400kHz I2C (from `#define I2C_SPEED 400000L`)
- **Address**: 0x68 (MPU6050 default)
- **Register Map**:
  - 0x3B-0x40: Accelerometer data (X, Y, Z - 6 bytes)
  - 0x43-0x48: Gyroscope data (X, Y, Z - 6 bytes)
  - 0x1C: Accelerometer sensitivity (±8g range)
  - 0x1B: Gyroscope sensitivity (±500°/s range)
  - 0x1A: Digital Low-Pass Filter config

#### 2.2 Sensor Reading Functions
```c
void ACC_getADC()      // Read accelerometer raw values into imu.accADC[3]
void Gyro_getADC()     // Read gyroscope raw values into imu.gyroADC[3]
void initSensors()     // Initialize I2C and MPU6050 at startup
```

**Sensitivity Constants** (from `Sensors.h`):
```c
#define ACC_1G 512      // 1G = 512 LSB (for ±8g range)
#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)  // Convert to rad/s
```

**Sensor Data Structure** (`types.h`):
```c
typedef struct {
  int16_t accSmooth[3];   // Filtered accelerometer
  int16_t gyroData[3];    // Processed gyroscope
  int16_t magADC[3];      // Magnetometer (if used, else zero)
  int16_t gyroADC[3];     // Raw gyroscope
  int16_t accADC[3];      // Raw accelerometer
} imu_t;
```

#### 2.3 Complementary Filter (`IMU.cpp` → imu.c/h`)
- **Function**: `computeIMU()` - Main IMU processing loop
- **Purpose**: Fuse accelerometer and gyroscope data
- **Parameters**:
  ```c
  #define ACC_LPF_FACTOR 4        // Accelerometer low-pass filter (1/16)
  #define GYR_CMPF_FACTOR 10      // Gyro/Acc fusion weight (1/1024)
  ```
- **Output**: `att.angle[]` array containing [ROLL, PITCH] estimates
- **Algorithm**: Complementary filter with gyro integration + accel correction
- **Interleaving**: Dual gyro reads for better accuracy with ~650µs between reads

#### 2.4 Attitude Estimation
```c
void getEstimatedAttitude()  // Derive roll/pitch from accel
int16_t mul(int16_t a, int16_t b)  // Fixed-point multiply for filters
```

**Output Structures**:
```c
typedef struct {
  int16_t angle[2];       // [ROLL, PITCH] in deg*100
  int16_t heading;        // YAW angle in degrees
} att_t;
```

---

## 3. NRF24L01 WIRELESS RECEIVER

### Critical Components (from `NRF24_RX.cpp`)

#### 3.1 SPI Frontend
- **Library**: Uses Arduino RF24 library (open-source C library available)
- **Pins**: CE=PD3, CSN=PD2 (as specified in config comments)
- **SPI Speed**: 10MHz (standard for NRF24)
- **Payload**: 7-byte struct for control data

#### 3.2 RC Data Structures
```c
// RX packet from remote (6 bytes)
typedef struct {
  uint8_t throttle;   // 0-255
  uint8_t yaw;        // 0-255 (mapped to ROLL in code)
  uint8_t pitch;      // 0-255
  uint8_t roll;       // 0-255 (mapped to YAW in code)
  uint8_t AUX1;       // 0-1 (binary)
  uint8_t AUX2;       // 0-1 (binary)
  uint8_t switches;   // Reserve for future
} RF24Data;

// TX packet to remote (feedback telemetry)
typedef struct {
  float lat;          // Latitude
  float lon;          // Longitude
  int16_t heading;    // Compass heading
  int16_t pitch;      // Pitch angle
  int16_t roll;       // Roll angle
  int32_t alt;        // Altitude in cm
  uint8_t flags;      // Flight mode status
} RF24AckPayload;
```

#### 3.3 NRF24 Initialization
```c
void NRF24_Init() {
  // Pipe address: 0xE8E8F0F0E1LL (5 bytes)
  // Data rate: 250kbps
  // AutoACK disabled
  // Listening mode enabled (receiver)
}

void NRF24_Read_RC() {
  // Poll for incoming data
  // Map 0-255 values to PWM range (1000-2000 microseconds)
  // Timeout handling (>1000ms = signal lost, failsafe)
  // Send telemetry back as ACK payload
}
```

#### 3.4 RC Channel Mapping
```c
// Channel indices (enum rc from types.h)
nrf24_rcData[THROTTLE] = map(MyData.throttle, 0, 255, 2000, 1000)
nrf24_rcData[ROLL]     = map(MyData.yaw, 0, 255, 2000, 1000)
nrf24_rcData[PITCH]    = map(MyData.pitch, 0, 255, 1000, 2000)
nrf24_rcData[YAW]      = map(MyData.roll, 0, 255, 2000, 1000)
nrf24_rcData[AUX1]     = map(MyData.AUX1, 0, 1, 2000, 1000)
nrf24_rcData[AUX2]     = map(MyData.AUX2, 0, 1, 2000, 1000)
```

**Implementation Strategy**: 
1. Adapt existing Arduino RF24 library to STM8 SPI interface
2. Or write minimal NRF24 I/O driver using STM8 SPI HAL
3. Focus on: Init, Read(), WriteAckPayload() functions

---

## 4. MOTOR CONTROL OUTPUT

### Critical Components (from `Output.cpp`)

#### 4.1 PWM Configuration
- **Motor Count**: 4 (QUADX configuration)
- **PWM Mode**: Hardware PWM using STM8 timers
- **PWM Range**: 1000-2000 microseconds (standard ESC/brushed motor range)
- **Loop Rate**: 2800µs per cycle (~357 Hz)

**Motor to Timer Mapping** (Arduino Nano NRF24 variant):
```c
// Pin mapping: uint8_t PWM_PIN[8] = {9, 6, 5, 3, ...}
Motor[0] → Pin D9 (Timer1 Channel2) - Rear motor
Motor[1] → Pin D6 (Timer0/software PWM) - Right motor  
Motor[2] → Pin D5 (Timer0/software PWM) - Left motor
Motor[3] → Pin D3 (Timer2 Channel1) - Front motor
```

**For STM8S005K6T6**, available timers:
- TIM1: 4 channels (for motors)
- TIM2: 3 channels (possible motor output)
- TIM3: 2 channels
- TIM4: 2 channels

#### 4.2 Motor Control Functions
```c
void initOutput()           // Configure PWM timers, set base frequency
void mixTable()             // Calculate motor PWM from PID outputs
void writeMotors()          // Update PWM compare registers
```

#### 4.3 Motor Mix Calculation
```c
// Input: axisPID[3] = {ROLL, PITCH, YAW} PID outputs from stabilization
// Output: motor[4] = calculated motor speeds
// QUADX configuration: opposite motors sum/differ for stabilization

// Typical QUADX mix:
motor[0] = axisPID[THROTTLE] + axisPID[PITCH] + axisPID[YAW]  // Rear
motor[1] = axisPID[THROTTLE] - axisPID[ROLL] - axisPID[YAW]   // Right
motor[2] = axisPID[THROTTLE] - axisPID[PITCH] + axisPID[YAW]  // Left
motor[3] = axisPID[THROTTLE] + axisPID[ROLL] - axisPID[YAW]   // Front
```

#### 4.4 PWM Value Constraints
```c
#define MINTHROTTLE 1000   // Minimum motor speed
#define MAXTHROTTLE 2000   // Maximum motor speed
#define MINCOMMAND  1000   // Default (motors off)
```

---

## 5. FLIGHT STABILIZATION & CONTROL LOOP

### Main Loop Architecture (from `MultiWii.cpp`)

#### 5.1 Timing Control
```c
#define LOOP_TIME 2800  // Microseconds per control cycle (357 Hz)

uint32_t currentTime;   // Current timestamp
uint16_t previousTime;  // Previous loop timestamp
uint16_t cycleTime;     // Actual time taken for last cycle
```

#### 5.2 Main Control Loop Flow
```
1. Read RC input from NRF24
2. Read IMU sensors (MPU6050)
3. Compute complementary filter → Attitude estimation
4. Calculate PID control outputs (roll, pitch, yaw, altitude)
5. Mix PID outputs → Motor PWM values
6. Write motor PWM values
7. Send telemetry via serial (MSP protocol)
8. Wait for consistent loop timing
```

#### 5.3 PID Controller (`types.h` - pid enum)
```c
enum pid {
  PIDROLL,    // Roll stabilization
  PIDPITCH,   // Pitch stabilization
  PIDYAW,     // Yaw rotation
  PIDALT,     // Altitude hold (if baro enabled)
  PIDPOS,     // Position (GPS)
  PIDPOSR,    // Position rate
  PIDNAVR,    // Navigation
  PIDLEVEL,   // Level assist
  PIDMAG,     // Magnetic hold
  PIDVEL      // Velocity (not used)
};

// PID coefficients structure
typedef struct {
  uint8_t P8;  // Proportional gain
  uint8_t I8;  // Integral gain
  uint8_t D8;  // Derivative gain
} pids_t;
```

#### 5.4 Arming/Disarming System
```c
flags_struct_t f;  // Contains armed flag and other status bits
uint8_t rcOptions[CHECKBOXITEMS];  // Flight mode switches
```

**Core Flight Modes**:
- ARM: Arm/disarm motors
- ANGLE: Stabilize to level
- HORIZON: Self-leveling assist

#### 5.5 Failsafe Handling
```c
#define FAILSAFE_THROTTLE 1000  // Default throttle on signal loss
int16_t failsafeCnt;             // Counter for signal loss detection
// Timeout: >1000ms without NRF24 data triggers failsafe
```

---

## 5.1 DATA FLOW ARCHITECTURE (MPU6050 → Processing → Motors & Telemetry)

### Overall System Data Flow

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                         SENSOR DATA ACQUISITION                              │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   MPU6050 (I2C Bus)                      NRF24L01 (SPI Bus)                  │
│   ────────────────────                   ─────────────────────               │
│   • Accel (X,Y,Z)                        • Throttle (0-255)                  │
│   • Gyro (X,Y,Z)                         • Roll (0-255)                      │
│   • Temp (optional)                      • Pitch (0-255)                     │
│                                          • Yaw (0-255)                       │
│                                          • AUX1-AUX2 (arm/mode)              │
│                                          • Signal timeout                    │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
                             ↓
┌──────────────────────────────────────────────────────────────────────────────┐
│                      SENSOR DATA PROCESSING                                  │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  MPU PROCESSING CHAIN:                   RF24 PROCESSING CHAIN:              │
│  ─────────────────────────────────       ─────────────────────────────       │
│  Raw accel/gyro → Read ISR               Raw radio packet → SPI ISR          │
│       ↓                                        ↓                             │
│  Orientation correction                  Check timeout/CRC                   │
│       ↓                                        ↓                             │
│  Accel low-pass filter (1/16)            Parse throttle/roll/pitch/yaw       │
│       ↓                                        ↓                             │
│  Gyro/Accel complementary filter         Map 0-255 → 1000-2000 µs            │
│       ↓                                        ↓                             │
│  OUTPUT: att.angle[ROLL, PITCH]          OUTPUT: rcData[8]                   │
│  OUTPUT: att.heading (yaw)               OUTPUT: rcSerial[8]                 │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
                 ↓                                    ↓
    ┌─────────────────────────────────┐  ┌──────────────────────────────────┐
    │  ATTITUDE ESTIMATION            │  │  RC INPUT EXTRACTION             │
    │  (imu.c::computeIMU)            │  │  (nrf24.c::NRF24_Read_RC)        │
    │  ──────────────────────────     │  │  ──────────────────────────────  │
    │  • Roll error = attitude -      │  │  • rcData[ROLL/PITCH/YAW/THROTL] │
    │    desired roll (from RC)       │  │  • rcData[AUX1/AUX2]             │
    │  • Pitch error = attitude -     │  │  • ARM flag (AUX1 > 1500µs)      │
    │    desired pitch (from RC)      │  │  • Flight mode (AUX2)            │
    │  • Yaw error calculated         │  │  • Failsafe if timeout           │
    │                                 │  │                                  │
    │  Storage: att_t struct {        │  │  Storage: int16_t rcData[8]      │
    │    int16_t angle[2];  // [R,P]  │  │                                  │
    │    int16_t heading;   // yaw    │  │                                  │
    │  }                              │  │                                  │
    │                                 │  │                                  │
    └─────────────────────────────────┘  └──────────────────────────────────┘
                 ↓                                    ↓
┌──────────────────────────────────────────────────────────────────────────────┐
│                        FLIGHT CONTROL LOGIC                                  │
│                     (main.c / MultiWii.cpp)                                  │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  INPUTS: attitude (roll, pitch, yaw) + RC commands (stick positions)         │
│                                                                              │
│  1. Read rcData[ROLL/PITCH/YAW/THROTTLE] and convert to angle setpoints      │
│  2. Calculate error: error = setpoint_angle - current_attitude               │
│  3. Apply 3-axis PID control:                                                │
│     output[i] = Kp*error + Ki*integral(error) + Kd*derivative(error)         │
│  4. Output stored in axisPID[0]=ROLL, axisPID[1]=PITCH, axisPID[2]=YAW       │
│                                                                              │
│  Storage: int16_t axisPID[3]  // PID outputs (-500 to +500 range)            │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
                                 ↓
┌──────────────────────────────────────────────────────────────────────────────┐
│                         MOTOR MIXER (QUADX)                                  │
│                    (motors.c / Output.cpp)                                   │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  INPUTS: axisPID[ROLL/PITCH/YAW] + rcData[THROTTLE]                          │
│                                                                              │
│  Mix calculation (QUADX):                                                    │
│  ───────────────────────                                                     │
│  motor[REAR] = throttle + pitch + yaw    // TIM1_CH1                         │
│  motor[RIGHT] = throttle - roll - yaw    // TIM1_CH2                         │
│  motor[LEFT] = throttle - pitch + yaw    // TIM2_CH1                         │
│  motor[FRONT] = throttle + roll - yaw    // TIM2_CH2                         │
│                                                                              │
│  Constrain to PWM range: 1000-2000 µs                                        │
│                                                                              │
│  Storage: int16_t motor[4]  // -500 to +500 (relative to center 1500)        │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
                                 ↓
┌──────────────────────────────────────────────────────────────────────────────┐
│                    PWM OUTPUT TO ESC/MOTORS                                  │
│                   (Output.c / Output.cpp)                                    │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Convert motor[4] → PWM pulse widths (1000-2000 µs)                          │
│  Write to timer compare registers:                                           │
│  • motor[0] → TIM1_SetCompare1() - Rear motor                                │
│  • motor[1] → TIM1_SetCompare2() - Right motor                               │
│  • motor[2] → TIM2_SetCompare1() - Left motor                                │
│  • motor[3] → TIM2_SetCompare2() - Front motor                               │
│                                                                              │
│  PWM Frequency: 50Hz (20ms period)                                           │
│  All 4 motors updated synchronously every control cycle                      │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
                                 ↓
                        ┌─────────────────┐
                        │  BRUSHED MOTORS │
                        │  (4x DC Motors) │
                        └─────────────────┘
```

### Detailed MPU6050 Data Path

**Stage 1: Hardware Acquisition** (I2C, ~1-5ms)
```
Sensor Registers                Data Variables              Processing
────────────────                ──────────────              ────────────
[0x3B-0x40] Accel X,Y,Z  ─→  imu.accADC[3]  ─→  Signed 16-bit integer
[0x43-0x48] Gyro X,Y,Z   ─→  imu.gyroADC[3] ─→  Signed 16-bit integer

I2C Read Function (sensors.c):
void ACC_getADC() {
  i2c_read_reg_to_buf(0x68, 0x3B, &rawADC[0], 6);  // 6 bytes (3 axes × 2)
  ACC_ORIENTATION(rawADC[0]<<8|rawADC[1],          // Combine MSB+LSB
                  rawADC[2]<<8|rawADC[3],          // Create 16-bit values
                  rawADC[4]<<8|rawADC[5]);         // Orient for board
}

Data Range:
• Accelerometer: ±8g range → ±8000 m/s² → 512 LSB per G
  Example: 1G = 512 LSB = 0x0200
• Gyroscope: ±500°/s range → 65.5 LSB per °/s
  Example: 90°/s = 5904 LSB
```

**Stage 2: Filtering & Orientation** (Complementary Filter, ~0.5ms)
```
Raw Sensor Data                Low-Pass Filter              Fused Estimate
─────────────────              ───────────────              ───────────────

imu.accADC[X,Y,Z]    ─→  LPF factor 1/16  ─→  imu.accSmooth[X,Y,Z]
                                                    ↓
                                            Complementary Filter
                                            
imu.gyroADC[X,Y,Z]   ─→  Integrate gyro   ─→  angle_from_gyro[X,Y]
                          dt = 2800 µs          ↓
                                            Weight=1024 (GYR_CMPF_FACTOR)
                                                    
Combined Filter Output:
────────────────────
angle_estimated = (angle_from_gyro * 1024 + accSmooth * 0) / 1024

This gives fast response from gyro with drift correction from accel
```

**Stage 3: Attitude Calculation** (imu.c::getEstimatedAttitude)
```cpp
// Convert accelerometer vector to roll/pitch angles
att.angle[ROLL]  = atan2(accSmooth[Y], accSmooth[Z]) × 180/π × 100
                 = Result in degrees × 100 (e.g., 4500 = 45°)

att.angle[PITCH] = asin(accSmooth[X] / ACC_1G) × 180/π × 100
                 = Result in degrees × 100

// Gyroscope integration for drift correction
// Over 2800µs cycle: integrate gyro rates and blend with accel estimate
```

**Storage Structure**:
```c
// Final output from IMU processing (types.h)
typedef struct {
  int16_t angle[2];       // [ROLL, PITCH] in deg*100 (range: ±9000)
  int16_t heading;        // YAW in degrees (optional, mag-based)
} att_t;

extern att_t att;  // Global used by PID controller
```

---

### Detailed NRF24L01 Data Path

**Stage 1: Wireless Reception** (SPI, ~100-500µs)
```
Air (2.4GHz)            RF24 Module              SPI Bus              MCU
────────────            ───────────              ──────────           ───

Radio Packet:           Demodulate               CE=PD3 (ctrl)
throttle (1 byte)   ─→  payload            ─→   CSN=PD2 (chip sel)
roll (1 byte)           ← FIFO ─→                MOSI/MISO            
pitch (1 byte)                                  SCK=clk               
yaw (1 byte)
AUX1 (1 byte)       
AUX2 (1 byte)
switches (1 byte)
─────────────────────
7 bytes total = 56 bits

Pipe Address: 0xE8E8F0F0E1LL (5-byte address, must match TX)
Frequency: 2.4 GHz (channel configurable)
Data Rate: 250 kbps (slow but robust)
Timeout: 1000ms (drone failsafe if no packets received)
```

**Stage 2: Data Extraction & Timeout** (nrf24.c::NRF24_Read_RC)
```c
// Poll NRF24 for new packets
RF24Data MyData;  // Struct defined in NRF24_RX.h
unsigned long now = millis();

// Check if new data available
while (radio.available()) {
  radio.read(&MyData, sizeof(RF24Data));  // Extract 7 bytes from FIFO
  lastRecvTime = now;                     // Update timestamp
}

// Failsafe detection
if (now - lastRecvTime > 1000) {
  // Signal lost! Reset to safe defaults
  resetRF24Data();  
  // Sets: throttle=0, roll/pitch/yaw=128 (neutral), AUX=0
  // Drone won't respond to sticks until signal returns
}
```

**Stage 3: Value Mapping** (0-255 → 1000-2000 µs PWM range)
```c
// NRF24 sends 0-255 byte values
// Must convert to 1000-2000 µs range (standard RC protocol)

MyData.throttle = 0       →  map() →  2000 µs (full throttle - inverted!)
MyData.throttle = 255     →  map() →  1000 µs (no throttle)
MyData.roll = 0           →  map() →  2000 µs (full right)
MyData.roll = 255         →  map() →  1000 µs (full left)

// map(value, fromLow, fromHigh, toLow, toHigh) formula
// newValue = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow

nrf24_rcData[THROTTLE] = map(MyData.throttle, 0, 255, 2000, 1000);
nrf24_rcData[ROLL]     = map(MyData.yaw, 0, 255, 2000, 1000);     // Note: YAW maps to ROLL
nrf24_rcData[PITCH]    = map(MyData.pitch, 0, 255, 1000, 2000);
nrf24_rcData[YAW]      = map(MyData.roll, 0, 255, 2000, 1000);    // Note: ROLL maps to YAW
nrf24_rcData[AUX1]     = map(MyData.AUX1, 0, 1, 2000, 1000);      // Binary AUX (arm switch)
nrf24_rcData[AUX2]     = map(MyData.AUX2, 0, 1, 2000, 1000);      // Binary AUX
```

**Storage Structure**:
```c
// Final RC data available to flight controller (types.h)
extern int16_t rcData[RC_CHANS];  // RC_CHANS = 8

// Indices (enum rc):
rcData[ROLL]      // 1000-2000 µs equivalent (stick left-right)
rcData[PITCH]     // 1000-2000 µs equivalent (stick forward-back)
rcData[YAW]       // 1000-2000 µs equivalent (stick twist)
rcData[THROTTLE]  // 1000-2000 µs equivalent (stick up-down)
rcData[AUX1]      // 2000 or 1000 (arm/disarm switch)
rcData[AUX2]      // 2000 or 1000 (flight mode switch)
rcData[AUX3]      // Reserved
rcData[AUX4]      // Reserved
```

**Telemetry Feedback** (Drone → Remote):
```c
// Send drone state back to remote as NRF24 ACK payload
RF24AckPayload nrf24AckPayload = {
  .lat = 35.62,                 // Latitude (dummy)
  .lon = 139.68,                // Longitude (dummy)
  .heading = att.heading,       // Current yaw angle
  .pitch = att.angle[PITCH],    // Current pitch
  .roll = att.angle[ROLL],      // Current roll
  .alt = alt.EstAlt,            // Estimated altitude (cm)
  .flags = f.flags              // Status flags (armed/mode/etc)
};

// This gets sent back with every acknowledgment
radio.writeAckPayload(1, &nrf24AckPayload, sizeof(RF24AckPayload));
```

---

### Complete Control Cycle Timing (2800 µs)

```
Time (µs)    Event                                    Duration
──────────   ─────────────────────────────────────    ──────────
0 µs    ┌→  START OF CYCLE
        │   NRF24_Read_RC()                          ~100 µs
~100 µs ├→  Gyro_getADC() [1st read]                 ~5 µs
        │   timeInterleave = micros()
~105 µs ├→  ACC_getADC()                             ~5 µs
        │   annexCode() [other processing]           ~50 µs
~160 µs ├→  WAIT 650 µs for interleave
~810 µs ├→  Gyro_getADC() [2nd read]                 ~5 µs
~815 µs ├→  computeIMU() [complementary filter]      ~30 µs
        │   • Merge gyro readings
        │   • Process accel LPF
        │   • Fuse into attitude angles
~845 µs ├→  PID_calculation()                        ~50 µs
        │   • Calculate roll/pitch/yaw errors
        │   • Apply P, I, D terms
        │   • Output to axisPID[3]
~895 µs ├→  mixTable()                               ~20 µs
        │   • QUADX motor mixing
        │   • Constrain to 1000-2000 µs
~915 µs ├→  writeMotors()                            ~10 µs
        │   • Update TIM1/TIM2 compare registers
~925 µs ├→  serialCom() [MSP protocol]               ~500-1000 µs
        │   • Process incoming commands
        │   • Send telemetry (if polled)
        │   • XOR checksums
~1925 µs├→  WAIT TO MAINTAIN ~2800 µs LOOP TIME
        │   (varies based on serial activity)
~2800 µs└→  END OF CYCLE / START NEXT CYCLE
```

**Critical Timing Constraints**:
- ✅ Gyro dual-read interleaving essential for noise rejection
- ✅ Complementary filter must run every 2800µs for stability
- ✅ PWM update must be synchronous (all motors at same time)
- ✅ Serial processing must be non-blocking (ISR + ring buffer)
- ⚠️ If cycle exceeds ~3500µs, PID becomes unstable
- ⚠️ I2C bus errors can add 50-100µs stalls

---

### Data Variables Summary

```c
// Sensor Data
int16_t imu.accADC[3];      // Raw accel: -4096 to +4096 (±8g)
int16_t imu.gyroADC[3];     // Raw gyro: -32768 to +32767 (±500°/s)
int16_t imu.accSmooth[3];   // Filtered accel
int16_t imu.gyroData[3];    // Processed gyro

// Attitude Estimate (PRIMARY OUTPUT OF IMU)
int16_t att.angle[2];       // ROLL, PITCH in deg×100 (±9000 = ±90°)
int16_t att.heading;        // YAW in degrees

// RC Input (PRIMARY INPUT FROM NRF24)
int16_t rcData[8];          // All channels in 1000-2000 µs range
  ├─ rcData[ROLL]           // ±500 from center 1500
  ├─ rcData[PITCH]
  ├─ rcData[YAW]
  ├─ rcData[THROTTLE]       // 1000-2000 (0-100% throttle)
  ├─ rcData[AUX1]           // 2000 (armed) or 1000 (disarmed)
  └─ rcData[AUX2]           // Flight mode

// Control Outputs
int16_t axisPID[3];         // PID outputs: -500 to +500
  ├─ axisPID[0] = ROLL correction
  ├─ axisPID[1] = PITCH correction
  └─ axisPID[2] = YAW correction

// Motor Command
int16_t motor[4];           // -500 to +500 (relative to 1500 center)
  ├─ motor[0] = Rear motor
  ├─ motor[1] = Right motor
  ├─ motor[2] = Left motor
  └─ motor[3] = Front motor

// System Status
flags_struct_t f;           // Armed flag, flight mode, etc.
uint32_t currentTime;       // Microsecond counter
uint16_t cycleTime;         // Actual time of last cycle
```

---

## 6. EEPROM CONFIGURATION STORAGE

### Persistent Settings (from `EEPROM.cpp`)

#### 6.1 Configuration Structure
```c
typedef struct {
  uint8_t pidkp[PIDITEMS];       // PID proportional gains
  uint8_t pidki[PIDITEMS];       // PID integral gains
  uint8_t pidkd[PIDITEMS];       // PID derivative gains
  uint8_t rcRate8;               // RC sensitivity
  uint8_t rcExpo8;               // RC curve exponential
  uint8_t rollPitchRate;         // Roll/pitch rotation speed
  uint8_t yawRate;               // Yaw rotation speed
  // ... more settings ...
} conf_t;
```

#### 6.2 EEPROM Access Functions
```c
void readEEPROM();               // Load config at startup
void writeEEPROM();              // Save config changes (MSP_EEPROM_WRITE)
```

**STM8 Implementation**:
- STM8S005 has 640 bytes Flash memory available for data
- Use `FLASH` segment for settings storage
- Or use embedded EEPROM simulation via Flash

#### 6.3 MSP Commands
- `MSP_PID` (112): Read current PID values
- `MSP_SET_PID` (202): Write new PID values
- `MSP_EEPROM_WRITE` (250): Commit to permanent storage

---

## 7. CONFIGURATION & DEFINES

### Critical Configuration Items

```c
// ========== COPTER TYPE ==========
#define QUADX              // Quad with X configuration

// ========== SENSORS ==========
#define ACC                // Accelerometer enabled (MPU6050)
#define GYRO               // Gyroscope enabled (MPU6050)
#define MPU6050            // Specific sensor type

// ========== RECEIVER ==========
#define NRF24_RX           // NRF24L01 receiver
#define RC_CHANS 8         // 8 RC channels supported

// ========== TIMINGS ==========
#define LOOP_TIME 2800     // Main loop in microseconds (357 Hz)
#define I2C_SPEED 400000L  // I2C at 400kHz

// ========== MOTOR TIMING ==========
#define MINTHROTTLE 1000   // Minimum motor command
#define MAXTHROTTLE 2000   // Maximum motor command
#define MINCOMMAND 1000    // Motors off value

// ========== FILTERING ==========
#define ACC_LPF_FACTOR 4       // Accel filter: 1/16
#define GYR_CMPF_FACTOR 10     // Gyro/Accel fusion: 1/1024

// ========== SENSORS ENABLED ==========
#define BARO 0             // Barometric altitude (disabled for brushed drone)
#define MAG 0              // Magnetometer (disabled)
#define GPS 0              // GPS (disabled)
```

---

## 8. IMPLEMENTATION PRIORITY & PHASES

### Phase 1: Core Stabilization (Essential)
1. **I2C & MPU6050 driver** - Read IMU data
2. **IMU computation** - Complementary filter for attitude
3. **PID control** - Calculate motor commands from attitude error
4. **PWM output** - Drive 4 motors via timers
5. **NRF24 receiver** - Get RC commands wirelessly
6. **Main control loop** - Timed execution (~357 Hz)

### Phase 2: Serial Communication
1. **UART initialization** - 115200 baud (already have this)
2. **MSP protocol layer** - Packet encode/decode
3. **Critical MSP messages**:
   - MSP_IDENT - Identification
   - MSP_STATUS - Health/timing
   - MSP_RAW_IMU - Sensor telemetry
   - MSP_MOTOR - Motor PWM feedback
   - MSP_ATTITUDE - Flight angle telemetry

### Phase 3: Configuration & Advanced Features
1. **EEPROM storage** - Save/load PID settings
2. **GUI integration** - MSP_SET_PID, MSP_SET_RC_TUNING
3. **Calibration routines** - Accel/gyro zeroing
4. **Advanced flight modes** - ANGLE, HORIZON
5. **Failsafe** - Signal loss detection

---

## 9. FILE STRUCTURE FOR STM8 PORT

```
/media/bartek/LEXAR/DEV/STM8/drone/
├── src/
│   ├── main.c                    # Main control loop
│   ├── imu.c/h                  # IMU processing & complementary filter
│   ├── pid.c/h                  # PID controller implementation
│   ├── motors.c/h               # PWM motor control
│   ├── nrf24.c/h                # NRF24 wireless receiver driver
│   ├── msp_protocol.c/h          # MultiWii Serial Protocol
│   ├── config.h                 # Configuration defines
│   └── types.h                  # Data structure definitions
├── custom_lib/
│   ├── serial.c/h               # UART (already exists)
│   └── i2c.c/h                  # I2C driver for MPU6050
├── lib/                         # STM8 Standard Peripheral Library
└── makefile
```

---

## 10. KEY DEPENDENCIES & CONSTRAINTS

### Hardware Constraints
- **Flash**: 32KB (need to fit all code + lookup tables)
- **RAM**: 2KB (limited for large buffers)
- **CPU**: 16MHz (just enough for 357Hz loop @ 8MHz internal)
- **I2C**: Shared bus for MPU6050 (and optional magnetometer)
- **SPI**: For NRF24L01 (separate from I2C)
- **Timers**: Need 4+ PWM channels for motors

### Timing Constraints
- **Control loop**: Must maintain 2800µs consistency
- **I2C reads**: ~5-10ms for 6-axis sensor
- **Serial processing**: Non-blocking, buffered
- **NRF24 polling**: <100µs per check

### Algorithm Characteristics
- **Fixed-point math**: No FPU, use integer arithmetic (from Arduino code)
- **Lookup tables**: Precomputed sin/cos for angle calculations
- **Interrupt-driven**: Serial RX, potentially I2C and timers

---

## 11. PORTING NOTES

### From Arduino to STM8
1. **Remove Arduino.h dependency** - Replace with STM8 HAL
2. **Interrupt vectors** - Different syntax (use SDCC __interrupt(n) decorator)
3. **Memory model** - Watch RAM usage; may need to load tables from Flash
4. **Clock speed** - Arduino 16MHz → STM8 16MHz (same, good!)
5. **Floating point** - Avoid if possible; use fixed-point (multiplied by 100)
6. **SPI/I2C libraries** - Write minimal STM8 drivers instead of Arduino libraries

### Files to Port (Priority Order)
1. `types.h` - Data structures (minimal changes needed)
2. `config.h` - Defines (just adapt pin definitions)
3. `IMU.cpp` + `Sensors.cpp` - Sensor reading & complementary filter (core algorithm)
4. `Protocol.cpp` + `Serial.cpp` - MSP protocol (adapt to STM8 HAL)
5. `NRF24_RX.cpp` - RF24 driver (needs STM8 SPI wrapper)
6. `Output.cpp` - Motor PWM control (map to STM8 timers)
7. `MultiWii.cpp` - Main loop (adapt timing/interrupts)
8. `PID logic` - Usually in MultiWii.cpp (port core algorithm)

---

## 12. MINIMAL VIABLE PRODUCT (MVP)

To get drone flying and recognized by MultiWii PC software:

**Minimum Required**:
1. ✅ UART + MSP protocol (send identification + telemetry)
2. ✅ I2C + MPU6050 reading (raw IMU data)
3. ✅ Complementary filter (roll/pitch estimation)
4. ✅ Basic 3-axis PID stabilization
5. ✅ PWM motor output for 4 motors
6. ✅ NRF24 receiver polling for RC input
7. ✅ Main control loop (~357 Hz)

**Nice to Have**:
- AC calibration routines
- Config persistence (EEPROM)
- Flight mode switching (ANGLE/HORIZON)
- Failsafe with throttle cutoff
- LED status indicators

---

## 13. ESTIMATED CODE SIZE

| Component | LOC | Priority |
|-----------|-----|----------|
| IMU + Complementary Filter | 300-400 | HIGH |
| PID Controllers | 150-200 | HIGH |
| Motor Mixer | 100-150 | HIGH |
| NRF24 Driver | 200-300 | HIGH |
| MSP Protocol | 400-500 | HIGH |
| UART Ring Buffer | 100-150 | MED |
| I2C Driver | 200-300 | HIGH |
| Main Loop + Init | 200-300 | HIGH |
| EEPROM Config | 100-150 | LOW |
| **Total** | **1.6-2.4 KB** | - |

**Estimate**: 1600-2400 lines of C code for core functionality. Should fit in STM8S005 with careful management.

---

## 14. DEBUGGING STRATEGY

1. **Serial monitor first** - Verify UART + MSP handshake
2. **IMU data streaming** - Confirm accelerometer/gyro readings via MSP
3. **Motor test mode** - Drive each motor individually via PC GUI
4. **Attitude stabilization** - Verify pitch/roll hold without props
5. **RC input verification** - Check NRF24 data reception
6. **Flight testing** - Start with low throttle, minimal gains
7. **MultiWii GUI** - Connect PC software to verify all telemetry streams

---

## References
- Arduino MultiWii Source: `/arduino/MultiWii_NRF24_Brushed_Drone_V5_Code/MultiWii_RF24/`
- STM8S005 Datasheet: 16KB Flash, 2KB RAM, 16MHz internal oscillator
- MultiWii Protocol: Standard MSP format (documented in MultiWii wiki)
- RF24 Library: Available on GitHub (gcopeland/RF24)
- MPU6050 Datasheet: I2C address 0x68, 6-axis IMU

