# MultiWii Arduino Control Flow & Interrupt Architecture

## Overview
This document describes the **control flow**, **interrupt system**, and **data processing pipeline** from the original Arduino MultiWii NRF24 brushed drone codebase that needs to be ported to STM8S005K6T6.

---

## 1. SYSTEM INITIALIZATION (setup() → MultiWii.cpp:629)

### 1.1 Startup Sequence

```
setup() {
  1. GPIO Initialization
     ├─ pinMode(3, OUTPUT);   // Front motor PWM (D3/Timer2)
     ├─ pinMode(5, OUTPUT);   // Left motor PWM  (D5/Timer0)
     ├─ pinMode(6, OUTPUT);   // Right motor PWM (D6/Timer0)
     └─ pinMode(9, OUTPUT);   // Rear motor PWM  (D9/Timer1)

  2. Timer Configuration
     └─ TCCR0A = ... | B00000001;  // Set Timer0 to 62.5kHz PWM frequency

  3. Serial Port Setup (supports multiple UARTs)
     ├─ SerialOpen(0, 115200);  // UART0 for MultiWii PC GUI (MSP protocol)
     ├─ SerialOpen(1, 115200);  // UART1 (optional, for GPS/telemetry)
     ├─ SerialOpen(2, 115200);  // UART2 (MEGA only)
     └─ SerialOpen(3, 115200);  // UART3 (MEGA only)
     
     └─ Enable RX interrupt in each UART:
        UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);  // RX enable + RX complete ISR

  4. LED/Indicator Setup
     ├─ LEDPIN_PINMODE;
     ├─ POWERPIN_PINMODE;
     ├─ BUZZERPIN_PINMODE;
     └─ STABLEPIN_PINMODE;

  5. Motor Output Initialization
     └─ initOutput();  // Configure PWM timers, set default motor values

  6. Configuration Loading from EEPROM
     ├─ readGlobalSet();    // Load global settings
     ├─ readEEPROM();       // Load drone profile (PID gains, etc.)
     └─ Integrity check: Compare flash checksum

  7. RC Receiver Initialization
     └─ configureReceiver();
        └─ #if defined(NRF24_RX)
           NRF24_Init();  // Initialize SPI, RF24 pipe address, listening mode

  8. Sensor Initialization
     └─ initSensors();
        ├─ I2C_Init(400kHz);     // I2C bus for MPU6050
        ├─ MPU6050_Init();       // Configure gyro/accel
        ├─ Acc_Init();           // Set accelerometer sensitivity
        └─ Gyro_Init();          // Configure gyroscope range

  9. Calibration Setup
     ├─ calibratingG = 512;  // Gyro calibration counter
     ├─ calibratingB = 200;  // Barometer calibration (10s + 200*25ms)
     └─ calibratingA = 0;    // Accelerometer calibration (if GIMBAL enabled)

  10. Record Initial Timestamp
      └─ previousTime = micros();

  11. Optional: Initialize LCD/GPS/GPS Serial
      ├─ initLCD();
      ├─ GPS_set_pids();
      └─ GPS_SerialInit();

  12. Optional: LED Flasher
      └─ init_led_flasher();
}
```

**Key Setup Characteristics:**
- **Blocking**: Takes ~2-3 seconds to complete
- **EEPROM integrity** checked to ensure settings are valid
- **Multiple profiles** supported (0, 1, 2)
- **Calibration starts immediately** (gyro & baro)
- **RX receiver enabled** before main loop starts

### 1.2 UART ISR Installation

```cpp
// Arduino attaches ISRs automatically via ISR() macro
// In Serial.cpp - After SerialOpen() enables RX interrupt:

ISR(USART_RX_vect)    // Arduino Nano/ProMini
  store_uart_in_buf(UDR0, 0);

ISR(USART1_RX_vect)   // ProMicro/MEGA
  store_uart_in_buf(UDR1, 1);

ISR(USART2_RX_vect)   // MEGA only
  store_uart_in_buf(UDR2, 2);

ISR(USART3_RX_vect)   // MEGA only
  store_uart_in_buf(UDR3, 3);

// ISR Behavior:
void store_uart_in_buf(uint8_t data, uint8_t portnum) {
  uint8_t h = serialHeadRX[portnum];
  serialBufferRX[h++][portnum] = data;    // Store byte in circular buffer
  if (h >= RX_BUFFER_SIZE) h = 0;         // Wrap around
  serialHeadRX[portnum] = h;              // Advance write pointer
  // ISR does NOT process data - just buffers it
}
```

---

## 2. MAIN CONTROL LOOP (loop() → MultiWii.cpp:834)

### 2.1 Loop Cycle Architecture

```
loop() {
  ┌─────────────────────────────────────────────────────────────────┐
  │  CYCLE START - Timing Control                                   │
  ├─────────────────────────────────────────────────────────────────┤
  │  currentTime = micros();                                         │
  │  cycleTime = currentTime - previousTime;                         │
  │                                                                  │
  │  Target: Maintain LOOP_TIME = 2800 µs (~357 Hz)                 │
  │  Range: 2600 - 3000 µs (with varying serial activity)           │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 1: READ RC COMMANDS (1 of 3 conditional blocks)          │
  ├─────────────────────────────────────────────────────────────────┤
  │  if (currentTime - rcTime > 0) {  // 50Hz = 20ms interval        │
  │    NRF24_Read_RC();               // Poll NRF24 for new packet   │
  │    computeRC();                   // Convert to rcData[8]        │
  │    rcTime = currentTime + 20000;  // Schedule next read (20ms)   │
  │  }                                                              │
  │                                                                  │
  │  Output: rcData[ROLL/PITCH/YAW/THROTTLE] in 1000-2000 µs range  │
  │  Output: rcOptions[] - Flight mode switches (ARM, ANGLE, etc)    │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 1B: FAILSAFE CHECK                                       │
  ├─────────────────────────────────────────────────────────────────┤
  │  failsafeCnt++;                     // Increment every loop      │
  │  if (failsafeCnt > 5*FAILSAFE_DELAY) { // ~500ms no signal      │
  │    rcData[ROLL/PITCH/YAW] = MIDRC;  // Center sticks            │
  │    rcData[THROTTLE] = failsafe_throttle;  // Safe level           │
  │  }                                                              │
  │  if (failsafeCnt > 5*(FAILSAFE + FAILSAFE_OFF)) { // ~1500ms    │
  │    go_disarm();                     // Kill motors              │
  │  }                                                              │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 1C: ARM/DISARM & STICK COMMANDS                           │
  ├─────────────────────────────────────────────────────────────────┤
  │  if (rcData[THROTTLE] <= MINCHECK) {  // Throttle fully down     │
  │    if (rcOptions[BOXARM] && f.OK_TO_ARM)                        │
  │      go_arm();        // Arm motors                             │
  │    else if (f.ARMED)                                           │
  │      go_disarm();     // Disarm motors                          │
  │  }                                                              │
  │                                                                  │
  │  Stick combos available:                                        │
  │  ├─ THR_LO + YAW_LO + PIT_LO + ROL_CE → GYRO calibration        │
  │  ├─ THR_LO + YAW_LO + PIT_HI + ROL_HI → ACC calibration spawn   │
  │  └─ (Custom sticks per configuration)                           │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 2: READ SENSORS (Continuously)                           │
  ├─────────────────────────────────────────────────────────────────┤
  │  Gyro_getADC();        // Read gyro [1st read]                   │
  │  ACC_getADC();         // Read accel                             │
  │                                                                  │
  │  Wait ~650 µs for gyro to settle...                             │
  │                                                                  │
  │  Gyro_getADC();        // Read gyro [2nd read - averaged]        │
  │                                                                  │
  │  Output: imu.gyroADC[3], imu.accADC[3] - Raw sensor values       │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 3: IMU PROCESSING (Complementary Filter)                 │
  ├─────────────────────────────────────────────────────────────────┤
  │  getEstimatedAttitude();   // Fuse gyro + accel                 │
  │    ├─ Integrate gyro rates                                      │
  │    ├─ Apply low-pass filter to accel                            │
  │    ├─ Blend gyro + accel using complementary filter             │
  │    └─ Output: att.angle[ROLL], att.angle[PITCH] in deg×100      │
  │                                                                  │
  │  computeIMU();  // Additional IMU calculations                  │
  │    ├─ Apply sensor orientation correction                       │
  │    ├─ Update accSmooth[] (filtered accelerometer)               │
  │    └─ Calculate rolling measurements                            │
  │                                                                  │
  │  Output: att_t struct with estimated attitude                   │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 4: PID CONTROL CALCULATION                               │
  ├─────────────────────────────────────────────────────────────────┤
  │  for (axis = ROLL; axis <= YAW; axis++) {                       │
  │    // Get RC input setpoint                                     │
  │    error = rcCommand[axis] - att.angle[axis];                   │
  │                                                                  │
  │    // Apply PID gains (Proportional, Integral, Derivative)      │
  │    PTerm = Kp * error;                                          │
  │    ITerm = Ki * integral(error);                                │
  │    DTerm = Kd * derivative(error);                              │
  │                                                                  │
  │    axisPID[axis] = PTerm + ITerm + DTerm;  // Range: -500 to +500│
  │  }                                                              │
  │                                                                  │
  │  Output: axisPID[3] array with stabilization corrections        │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 5: MOTOR MIXING (QUADX Configuration)                    │
  ├─────────────────────────────────────────────────────────────────┤
  │  mixTable() {                                                   │
  │    motor[REAR]  = throttle + pitch + yaw;                       │
  │    motor[RIGHT] = throttle - roll - yaw;                        │
  │    motor[LEFT]  = throttle - pitch + yaw;                       │
  │    motor[FRONT] = throttle + roll - yaw;                        │
  │                                                                  │
  │    Constrain each motor to [1000, 2000] µs range                │
  │  }                                                              │
  │                                                                  │
  │  Output: motor[4] with final PWM values for ESCs/brushed motors │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 6: MOTOR OUTPUT (Update PWM Registers)                   │
  ├─────────────────────────────────────────────────────────────────┤
  │  writeMotors() {                                                │
  │    OCR1B = motor[REAR];   // Timer1 Channel B (D9)               │
  │    OCR0B = motor[RIGHT];  // Timer0 Channel B (D6)               │
  │    OCR0A = motor[LEFT];   // Timer0 Channel A (D5)               │
  │    OCR2B = motor[FRONT];  // Timer2 Channel B (D3)               │
  │  }                                                              │
  │                                                                  │
  │  Hardware: PWM output runs continuously via timer interrupts    │
  │  Frequency: 62.5 kHz (very high, inaudible to humans)           │
  │  Pulse with: 1000-2000 µs (controlled via compare register)     │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 7: SERIAL COMMUNICATION (MSP Protocol)                   │
  ├─────────────────────────────────────────────────────────────────┤
  │  #if !(SERIAL_RX && PROMINI)  // Skip if SERIAL_RX on single UART│
  │    serialCom() {                                                │
  │      // Non-blocking: processes buffered UART RX data           │
  │      // Data arrives via ISR in circular buffer                 │
  │      // This function:                                          │
  │      //  1. Parses MSP packets from buffer                      │
  │      //  2. Executes received commands                          │
  │      //  3. Sends telemetry responses                           │
  │                                                                  │
  │      serialCom_ParseMSP_Packet() {                             │
  │        // Wait for '$M' header                                 │
  │        // Parse <direction><length><cmd><data><checksum>      │
  │        // Validate: XOR checksum                               │
  │        //                                                      │
  │        // Process based on cmdMSP[port]:                       │
  │        case MSP_IDENT:        sendIdent();                     │
  │        case MSP_STATUS:       sendStatus();                    │
  │        case MSP_RAW_IMU:      sendRawIMU();      // 9-DOF data  │
  │        case MSP_ATTITUDE:     sendAttitude();    // Roll/pitch  │
  │        case MSP_MOTOR:        sendMotor();       // PWM values  │
  │        case MSP_RC:           sendRC();          // RC channels │
  │        case MSP_PID:          sendPID();         // PID gains   │
  │        case MSP_SET_PID:      setPID();          // Update PID  │
  │        case MSP_SET_RAW_RC:   setRC();           // Override RC │
  │        case MSP_EEPROM_WRITE: writeEEPROM();     // Save to EE  │
  │        ...more commands...                                      │
  │      }                                                          │
  │    }                                                            │
  │  #endif                                                        │
  │                                                                  │
  │  Timing: Variable, depends on serial activity                   │
  │  Typical: 100-500 µs (small telemetry requests)                 │
  │  Max: 1000+ µs (if large requests or multiple packets)          │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 8: OPTIONAL - Analog Reads (Power, RSSI, Battery Cells)  │
  ├─────────────────────────────────────────────────────────────────┤
  │  Executed sequentially (not every cycle):                       │
  │  ├─ One analog channel per cycle (cycling through)              │
  │  ├─ analogRead(PSENSOR_PIN);  // Current sensor                 │
  │  ├─ analogRead(V_BATPIN);     // Battery voltage                │
  │  ├─ analogRead(RX_RSSI_PIN);  // Receiver signal strength       │
  │  └─ analogRead(VBAT_CELLS);   // Individual cell voltages       │
  │                                                                  │
  │  Output: analog.vbat, analog.amperage, analog.rssi, etc.        │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 9: OPTIONAL - LED & Buzzer Indicators                    │
  ├─────────────────────────────────────────────────────────────────┤
  │  LED Status:                                                    │
  │  ├─ Calibrating: Blink (acc/gyro not ready)                     │
  │  ├─ Armed: Solid ON                                            │
  │  ├─ Not calibrated: Blink (angle error)                         │
  │  └─ Ready: OFF                                                 │
  │                                                                  │
  │  Buzzer:                                                        │
  │  └─ alarmHandler(); // Battery low, arming, etc.               │
  └─────────────────────────────────────────────────────────────────┘
                            ↓
  ┌─────────────────────────────────────────────────────────────────┐
  │  PHASE 10: TIMING SYNCHRONIZATION                               │
  ├─────────────────────────────────────────────────────────────────┤
  │  while (currentTime - previousTime < LOOP_TIME) {               │
  │    currentTime = micros();  // Busy-wait until time elapsed      │
  │  }                                                              │
  │  previousTime = currentTime;  // Ready for next cycle            │
  │                                                                  │
  │  Target: Maintain exact 2800 µs per cycle                      │
  └─────────────────────────────────────────────────────────────────┘
}
```

### 2.2 Loop Timing Breakdown

```
Time (µs)   Phase                              Duration    Cumulative
─────────   ─────────────────────────────────  ──────────  ──────────
0           PHASE 1: Update RC (20 ms every 1st)    0-20   0-20 µs
20          PHASE 1B: Failsafe check               5-10   20-30 µs
30          PHASE 1C: Arm/Disarm & Sticks         50-100  30-130 µs
130         PHASE 2: Read Sensors (650µs inter.)  100-200  130-230 µs
            [Wait for Gyro settling]        ~650        230-880 µs
880         PHASE 3: IMU Complementary Filter    30-50   880-930 µs
930         PHASE 4: PID Control Loop           50-100  930-1030 µs
1030        PHASE 5: Motor Mixer (QUADX)        20-30   1030-1060 µs
1060        PHASE 6: Write Motor PWM            10-20   1060-1080 µs
1080        PHASE 7: Serial MSP Protocol        100-1000 1080-2080 µs
2080        PHASE 8: Analog Reads (optional)    10-50   2080-2130 µs
2130        PHASE 9: LED/Buzzer/Status         20-50   2130-2180 µs
2180        PHASE 10: Timing Sync (Wait)       620     2180-2800 µs
─────────   ─────────────────────────────────  ──────────  ──────────
2800 µs     END OF CYCLE (Start next loop)
```

**Notes:**
- Phase 2 includes ~650 µs interleaved gyro read for noise rejection
- Phase 7 (Serial) is **non-blocking** due to ISR buffering
- Busy-wait in Phase 10 ensures cycle consistency (no hard timer tick)
- Actual cycle time drifts ±100-200 µs due to variable serial activity

---

## 3. INTERRUPT ARCHITECTURE

### 3.1 UART RX Interrupt (ISR)

```cpp
// Platform-dependent ISR names:
// Arduino Nano/ProMini:    ISR(USART_RX_vect)
// ProMicro/MEGA:           ISR(USART1_RX_vect), ISR(USART2_RX_vect), etc.

ISR(USART0_RX_vect) {  // Triggered on every byte received
  uint8_t data = UDR0;  // Read UART data register (must do immediately)
  
  store_uart_in_buf(data, 0);  // Add to circular RX buffer
  // No extensive processing - ISR returns quickly
}

// Circular Buffer Management:
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];  // 128 bytes per UART
static volatile uint8_t serialHeadRX[UART_NUMBER];  // Write pointer
static uint8_t serialTailRX[UART_NUMBER];           // Read pointer

void store_uart_in_buf(uint8_t data, uint8_t portnum) {
  uint8_t h = serialHeadRX[portnum];
  serialBufferRX[h++][portnum] = data;
  if (h >= RX_BUFFER_SIZE) h = 0;  // Circular wrap
  serialHeadRX[portnum] = h;
}

// Main loop polling (non-blocking):
uint8_t SerialAvailable(uint8_t port) {
  uint8_t diff = (serialHeadRX[port] - serialTailRX[port]);
  return diff;  // Bytes available in buffer
}

uint8_t SerialRead(uint8_t port) {
  uint8_t data = serialBufferRX[serialTailRX[port]][port];
  serialTailRX[port]++;
  if (serialTailRX[port] >= RX_BUFFER_SIZE) serialTailRX[port] = 0;
  return data;
}
```

**ISR Characteristics:**
- **Frequency**: ~11.5 kHz @ 115200 baud (one ISR per 87 µs)
- **Duration**: ~2-5 µs per ISR (just store byte)
- **Buffering**: Prevents data loss if main loop is busy
- **Multiple UARTs**: Each UART has separate ISR & buffer

### 3.2 Timer PWM Output (Hardware ISRs)

```cpp
// Timer Compare ISRs (hardware-driven, NOT user code)
// These run automatically at configured frequency

// Timer0 (16-bit, Channels A & B):
ISR(TIMER0_COMPA_vect) {
  // Automatically generated hardware interrupt
  // Compares Timer0 counter with OCR0A register
  // If equal, PWM output transitions (handled by hardware)
}

ISR(TIMER0_COMPB_vect) {
  // Similar for Channel B
}

// Timer1 (16-bit, Channels A/B/C):
ISR(TIMER1_COMPA_vect), ISR(TIMER1_COMPB_vect), ISR(TIMER1_COMPC_vect)
  // Automatic PWM generation

// Timer2 (8-bit, Channels A & B):
ISR(TIMER2_COMPA_vect), ISR(TIMER2_COMPB_vect)
  // Automatic PWM
```

**PWM Generation:**
- **Period**: ~20 ms (50 Hz, standard RC PWM frequency)
- **Resolution**: 1000-2000 µs pulse width (controlled by OCRx register)
- **User code**: Only writes to compare registers
  ```cpp
  OCR1B = motor[REAR];   // Update compare register
  // Hardware handles PWM timing automatically
  ```

### 3.3 Optional: I2C Interrupt

```cpp
// If I2C interrupts used (not in base Arduino code):
ISR(TWI_vect) {
  // Triggered on I2C START/STOP/byte transmitted/received
  // Currently: NOT USED in this codebase
  // Instead: Polling/blocking wait in main loop
}

// Current I2C Implementation:
uint8_t MPU_ReadReg(uint8_t reg) {
  I2C_GenerateSTART(ENABLE);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));  // POLL
  I2C_Send7bitAddress(MPU6050_ADDRESS, I2C_DIRECTION_RX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));  // POLL
  ...
}
```

---

## 4. DATA PROCESSING PIPELINE

### 4.1 RC Input Path (NRF24 → rcData → rcCommand)

```
NRF24 Wireless ──┐
                 ├─→ NRF24_Read_RC() {
                 │      radio.read(&MyData, 7 bytes);  // SPI read
                 │      lastRecvTime = now;
                 │    }
                 └──→ Buffer: MyData (RF24Data struct)
                      ├─ throttle (0-255)
                      ├─ yaw      (0-255)
                      ├─ pitch    (0-255)
                      ├─ roll     (0-255)
                      ├─ AUX1     (0-1)
                      ├─ AUX2     (0-1)
                      └─ switches

                      ↓ (in computeRC())

                 Convert to standard RC range:
                 nrf24_rcData[THROTTLE] = map(MyData.throttle, 0,255, 2000,1000)
                 nrf24_rcData[ROLL]     = map(MyData.yaw,     0,255, 2000,1000)
                 nrf24_rcData[PITCH]    = map(MyData.pitch,   0,255, 1000,2000)
                 nrf24_rcData[YAW]      = map(MyData.roll,    0,255, 2000,1000)

                      ↓

                 int16_t rcData[8]  // 1000-2000 µs range
                 ├─ rcData[ROLL]
                 ├─ rcData[PITCH]
                 ├─ rcData[YAW]
                 ├─ rcData[THROTTLE]
                 ├─ rcData[AUX1]
                 ├─ rcData[AUX2]
                 ├─ rcData[AUX3]
                 └─ rcData[AUX4]
                 |                    ↓
                 |           Apply expo & rate curves:
                 |           ├─ lookupPitchRollRC[5] (lookup table)
                 |           └─ lookupThrottleRC[11] (lookup table)
                 |                    ↓
                 |
                 └──→ int16_t rcCommand[4]
                      ├─ rcCommand[ROLL]      ±500 from center
                      ├─ rcCommand[PITCH]     ±500 from center
                      ├─ rcCommand[YAW]       ±500 from center
                      └─ rcCommand[THROTTLE]  0-1000 (absolute)
                           |
                           └──→ Used by PID controller as setpoints
```

### 4.2 Sensor Path (MPU6050 → IMU → Attitude)

```
MPU6050 I2C ──────┐
                  ├─→ Gyro_getADC() {
                  │     i2c_read(0x68, 0x43, &raw_gyro, 6);
                  │   }
                  ├→ Buffer: imu.gyroADC[3]
                  │   ├─ gyroADC[ROLL]
                  │   ├─ gyroADC[PITCH]
                  │   └─ gyroADC[YAW]
                  │
                  ├─→ ACC_getADC() {
                  │     i2c_read(0x68, 0x3B, &raw_acc, 6);
                  │   }
                  ├→ Buffer: imu.accADC[3]
                  │   ├─ accADC[X]
                  │   ├─ accADC[Y]
                  │   └─ accADC[Z]
                  │
                  └──────→ Wait 650 µs ──────→ Gyro_getADC() [2nd read]
                           └──→ Average both gyro reads
                                    ↓
                           getEstimatedAttitude() {
                             // Accel low-pass filter
                             imu.accSmooth[X] += (imu.accADC[X] - imu.accSmooth[X]) / 16;
                             
                             // Gyro integration (2800 µs per cycle)
                             angle_gyro += gyroADC * 2.8;
                             
                             // Complementary filter (1024:1 blend)
                             angle_est = (angle_gyro * 1024 + accSmooth * 0) / 1024;
                           }
                                    ↓
                           int16_t att.angle[2]  // Attitude estimate
                           ├─ att.angle[ROLL]    (-9000 to +9000 = ±90°, deg×100)
                           ├─ att.angle[PITCH]   (-9000 to +9000 = ±90°, deg×100)
                           └─ att.heading        (0-360°)
                                |
                                └──→ Used by PID controller for error calculation
```

### 4.3 PID Control Path (Attitude Error → Motor Mix)

```
rcCommand[i] - att.angle[i]  (Error = setpoint - current)
     |
     └───→ FOR i = ROLL, PITCH, YAW:
              ├─ Calculate proportional term:
              │  PTerm = Kp * error
              │
              ├─ Calculate integral term:
              │  ITerm = Ki * sum(error)
              │  (Low-pass filtered to prevent windup)
              │
              ├─ Calculate derivative term:
              │  DTerm = Kd * (error - lastError)
              │
              └─ Combine:
                 axisPID[i] = PTerm + ITerm + DTerm  // Range: ±500
                 |
                 └──→ int16_t axisPID[3]
                      ├─ axisPID[ROLL]    (Roll stabilization)
                      ├─ axisPID[PITCH]   (Pitch stabilization)
                      └─ axisPID[YAW]     (Yaw stabilization)
                           |
                           └──→ Motor Mixing (QUADX):
                                motor[0] = throttle + pitch + yaw;    [Rear]
                                motor[1] = throttle - roll - yaw;     [Right]
                                motor[2] = throttle - pitch + yaw;    [Left]
                                motor[3] = throttle + roll - yaw;     [Front]
                                |
                                └───→ int16_t motor[4]  (1000-2000 µs)
                                      |
                                      └──→ writeMotors():
                                           OCR1B = motor[0];  // D9  Timer1
                                           OCR0A = motor[2];  // D5  Timer0
                                           OCR0B = motor[1];  // D6  Timer0
                                           OCR2B = motor[3];  // D3  Timer2
                                           |
                                           └──→ Hardware PWM updates
```

### 4.4 Serial (MSP) Path (PC ↔ Drone)

```
PC (MultiWii GUI) ←─Serial (UART0, 115200 baud)─→ Arduino Drone

┌─ RX Path (PC → Drone) ─────────────────────────┐
│                                                │
│ UART ISR: Byte arrives → Store in buffer       │
│           [triggered ~11.5k times/sec]         │
│                                                │
│ Main Loop: serialCom() {                       │
│   while (SerialAvailable(0)) {                 │
│     c = SerialRead(0);                         │
│     Parse MSP packet:                          │
│       if (c == '$') state = HEADER_START;      │
│       if (c == 'M') state = HEADER_M;          │
│       if (c == '<') state = READ_DATA;         │
│       ...validate checksum...                  │
│       Execute command based on cmdMSP          │
│   }                                            │
│ }                                              │
│                                                │
│ Commands Received:                             │
│ ├─ MSP_SET_PID (202)      - Update PID gains   │
│ ├─ MSP_SET_RAW_RC (200)   - Override RC input  │
│ ├─ MSP_EEPROM_WRITE (250) - Save to EEPROM    │
│ ├─ MSP_RESET_CONF (208)   - Reset to defaults │
│ └─ ... many more ...                          │
│                                                │
└────────────────────────────────────────────────┘

┌─ TX Path (Drone → PC) ──────────────────────────┐
│                                                 │
│ Telemetry Sent (every cycle or when requested):│
│                                                 │
│ MSP_IDENT (100)    ─→ Drone type, version      │
│ MSP_STATUS (101)   ─→ Cycle time, sensors OK   │
│ MSP_RAW_IMU (102)  ─→ 9-DOF sensor data        │
│                       ├─ accel[3]              │
│                       ├─ gyro[3]               │
│                       └─ mag[3] (if present)   │
│ MSP_ATTITUDE (108) ─→ Roll, pitch, heading     │
│ MSP_MOTOR (104)    ─→ Current motor PWM [4]    │
│ MSP_RC (105)       ─→ RC channel values [8]    │
│ MSP_PID (112)      ─→ Current PID gains [9]    │
│ MSP_ANALOG (110)   ─→ Battery, RSSI, etc       │
│                                                 │
│ Packet Format: $M > [len] [cmd] [data...] [CX] │
│                ├─ $ M    = header (ASCII)      │
│                ├─ >      = direction (to PC)   │
│                ├─ [len]  = payload size        │
│                ├─ [cmd]  = message ID          │
│                ├─ [data] = payload bytes       │
│                └─ [CX]   = XOR checksum        │
│                                                 │
└────────────────────────────────────────────────┘
```

---

## 5. KEY GLOBAL VARIABLES & DATA STRUCTURES

### 5.1 Control Loop Variables

```c
// Timing
uint32_t currentTime;      // Current µs timestamp (overflows ~71 minutes)
uint16_t previousTime;     // Previous loop timestamp
uint16_t cycleTime;        // Time taken for last full cycle

// RC Input
int16_t rcData[RC_CHANS];      // Raw input from NRF24  [1000-2000 µs]
int16_t rcCommand[4];          // Processed RC: [-500,+500] or [0,1000]
uint8_t rcOptions[CHECKBOXITEMS]; // Flight mode switches (ARM, ANGLE, etc)

// IMU Data
imu_t imu;                      // Contains:
  int16_t accADC[3];            //   Raw accel
  int16_t accSmooth[3];         //   Filtered accel
  int16_t gyroADC[3];           //   Raw gyro
  int16_t gyroData[3];          //   Processed gyro

att_t att;                      // Attitude, contains:
  int16_t angle[2];             //   [ROLL, PITCH] in deg×100
  int16_t heading;              //   Heading in degrees

// Control Output
int16_t axisPID[3];             // PID outputs [±500]
int16_t motor[8];               // Motor commands [1000-2000 µs]

// Flags & Status
flags_struct_t f;               // Flight flags:
  bit ARMED;                    //   Motors armed
  bit OK_TO_ARM;                //   Conditions met to arm
  bit ACC_CALIBRATED;           //   Accel calibrated
  ... more flags ...

// Configuration
conf_t conf;                    // Loaded from EEPROM:
  uint8_t pid[PIDITEMS].P8;     //   PID proportional gains
  uint8_t pid[PIDITEMS].I8;     //   PID integral gains
  uint8_t pid[PIDITEMS].D8;     //   PID derivative gains
  uint8_t minthrottle;          //   Min motor command
  uint16_t failsafe_throttle;   //   Safe throttle on signal loss
  ... many more ...
```

### 5.2 UART Buffering

```c
// RX Buffering (ISR → Main Loop)
#define RX_BUFFER_SIZE 128
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadRX[UART_NUMBER];  // ISR writes here
static uint8_t serialTailRX[UART_NUMBER];           // Main reads from here

// TX Buffering (Main → ISR)
#define TX_BUFFER_SIZE 256
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER];
static volatile uint8_t serialTailTX[UART_NUMBER];

// MSP State Machine
static uint8_t c_state[UART_NUMBER];     // Parser state per UART
static uint8_t inBuf[64][UART_NUMBER];   // Incoming MSP data
static uint8_t checksum[UART_NUMBER];    // Running checksum
static uint8_t cmdMSP[UART_NUMBER];      // Received command type
```

---

## 6. INTERRUPT PRIORITY & NESTING

### 6.1 Arduino Interrupt Priorities

```
Priority (Lower number = Higher priority)
────────────────────────────────────────
1. External Interrupts (INT0, INT1)     - Highest response needed
2. Timer/Counter Interrupts              - Critical for PWM
3. UART RX Interrupt                     - Moderate (buffered)
4. UART TX Interrupt (UDRE)              - Moderate (buffered)
5. ADC Complete Interrupt                - Low (not time-critical)
6. Other exceptions                      - Lowest

Note: Interrupt nesting is NOT enabled in Arduino code
      (Global 'I' bit can disable all interrupts)
      serialCom() can disable interrupts if necessary:
        cli();  // Clear interrupt flag (disable all)
        ...
        sei();  // Set interrupt flag (enable all)
```

---

## 7. PORTING CONSIDERATIONS FOR STM8

### 7.1 Key Differences

```
Arduino               →  STM8S005K6T6
─────────────────     →  ──────────────────────
setup() / loop()      →  main() - implicit porting
ISR(name)             →  void handler(void) __interrupt(vector)
digitalWrite()        →  GPIO_WriteLow() / GPIO_WriteHigh()
delayMicroseconds()   →  Manual loop (CPU clock dependent)
analogRead()          →  ADC_Read() via STM8 HAL
Serial (HardwareSerial) → UART2 via STM8 HAL
SPI.transfer()        →  SPI_SendReceiveByte() via STM8 SPI HAL
I2C                   →  I2C_SendData() via STM8 I2C HAL
millis() / micros()   →  Timer counter or software
Timer PWM             →  TIM1/TIM2 Channels via STM8 HAL
```

### 7.2 Interrupt Mapping

```
Arduino INT Vector    →  STM8 ITC_IRQ / Vector #
──────────────────    →  ──────────────────────────
UART0 RX              →  UART2 RX (ITC_IRQ_UART2_RX = 21)
Timer0 Compare        →  TIM2 Compare (ITC_IRQ_TIM2_CAPCOM = 14)
Timer1 Compare        →  TIM1 Compare (ITC_IRQ_TIM1_CAPCOM = 12)
Timer2 Compare        →  Built into TIM2
External INT0, INT1   →  EXTI (Port B,C,D,E) - ITC_IRQ_PORTX = 4-7

Syntax:
Arduino:  ISR(USART_RX_vect) { ... }
STM8:     void UART2_RX_IRQHandler(void) __interrupt(21) { ... }
```

### 7.3 Critical Timing Constraints

```
Requirement                          Target      STM8 Impact
──────────────────────────────────  ──────────  ──────────────────
Main loop cycle                     2800 µs     Tight! 4 clocks per µs
Gyro interleave                     650 µs      Critical for stability
Sensor reading latency              <5 ms       I2C @ 400kHz OK
UART ISR overhead                   <5 µs       Must minimize code
Motor PWM frequency                 62.5 kHz    Use Timer HW PWM
RC read frequency                   50 Hz (20ms) Non-deterministic, OK
Serial processing                   <1000 µs    Non-blocking OK
```

---

## 8. MSP PROTOCOL DETAILS (Serial Communication)

### 8.1 Packet Structure

```
Direction: PC → Drone (command)
───────────────────────────────────────────────
Byte Format: $MdL<cmd><data...><checksum>
            ↑ ↑
            Header bytes

Example: Set PID for Roll (P=80, I=10, D=25)
$M < 3   202  80  10  25  chksum
  │ │ │        │
  │ │ └─ Data length
  │ └──── Direction: < = from PC, > = to PC
  └────── Command: 202 = MSP_SET_PID

Checksum: XOR of [length] [cmd] [data...]
chksum = length ^ cmd ^ data[0] ^ data[1] ^ ...

Direction: Drone → PC (telemetry)
────────────────────────────────────────────
$M > L cmd data... checksum

Example: Send attitude (Roll=4500, Pitch=-2000, Heading=120)
$M > 6   108  0x8B 0x11  0x30 0xF8  0x78 0x00  chksum
              └─ 4500 ─┘  └─ -2000 ─┘ └─ 120 ─┘
```

### 8.2 Critical MSP Commands

```
ID   Name                 Direction  Purpose
───  ──────────────────   ──────────  ────────────────────────────
100  MSP_IDENT           OUT        Drone ID, MultiWii version
101  MSP_STATUS          OUT        Cycle time, sensor health
102  MSP_RAW_IMU         OUT        9-DOF: accel + gyro + mag
104  MSP_MOTOR           OUT        Current motor PWM values [4]
105  MSP_RC              OUT        RC channel values [8]
108  MSP_ATTITUDE        OUT        Roll, pitch, heading
110  MSP_ANALOG          OUT        Battery voltage, RSSI
112  MSP_PID             OUT        Current PID gains [9]

200  MSP_SET_RAW_RC      IN         Override RC from GUI
202  MSP_SET_PID         IN         Update PID coefficients
208  MSP_RESET_CONF      IN         Reset to factory defaults
250  MSP_EEPROM_WRITE    IN         Save config to EEPROM
```

---

## SUMMARY: Control Flow Execution Order

```
1. setup() .................................... Initialize all systems
   ├─ GPIO, Timers, PWM
   ├─ UART ISRs enabled
   ├─ NRF24 initialized
   ├─ I2C & Sensors initialized
   ├─ EEPROM config loaded
   └─ Calibration starts

2. loop() REPEATING every 2800 µs ........... Main control cycle
   ├─ Read RC from NRF24 (20 ms interval)
   ├─ Check failsafe timeout
   ├─ Handle arm/disarm/stick commands
   ├─ Read sensors (I2C → MPU6050)
   ├─ IMU Complementary Filter → Attitude
   ├─ PID Control (Roll, Pitch, Yaw)
   ├─ Motor Mixing (QUADX)
   ├─ Update Motor PWM
   ├─ Serial MSP Communication (non-blocking)
   ├─ Analog readings (multiplexed)
   ├─ LED/Buzzer status
   └─ Busy-wait until 2800 µs elapsed

3. ISRs (Asynchronous) ....................... Event-driven
   ├─ UART RX ISR: Store bytes in buffer
   ├─ Timer PWM ISRs: Auto-generate PWM (HW)
   └─ (Optional) I2C ISR: Not implemented in base code

4. Repeat loop() ............................ Every 2.8 ms ≈ 357 Hz
```

---

## END OF DOCUMENT
