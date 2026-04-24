# WCH32V003 Migration Plan: NRF24 Radio Controller

## Project Overview
**Target Microcontroller:** WCH32V003 (20-pin, RISC-V)  
**Current Platform:** Arduino Nano (ATmega328P)  
**Application:** 6-Channel Radio Controller Transmitter  
**Key Components:**
- NRF24L01+ RF Module (SPI)
- 2× Analog Joysticks (4 ADC channels)
- 1× Potentiometer (1 ADC channel)
- 2× Toggle Switches (GPIO Digital Input)
- 16×2 LCD Display (I2C)
- Battery Voltage Monitoring (1 ADC channel)

---

## Phase 1: Development Environment Setup

### 1.1 Toolchain Installation
- [ ] Install WCH32 SDK/IDE or PlatformIO with WCH32V003 support
- [ ] Set up wch32fun Library framework
  - Clone: `https://github.com/cnlohr/ch32v003fun`
  - Configure build system (CMake or direct makefiles)
- [ ] Install OpenOCD for debugging (CH-DEBUG probe compatible)
- [ ] Verify GCC toolchain for RISC-V (`riscv-none-embed-gcc`)

### 1.2 Hardware Setup
- [ ] Verify WCH32V003 20-pin pinout and available peripherals
- [ ] Create pinout mapping document
- [ ] Test development board with LED blink program
- [ ] Verify SPI interface availability
- [ ] Check I2C interface compatibility

---

## Phase 2: Hardware Resource Mapping

### 2.1 Pin Assignment (WCH32V003, 20-pin LQFP)

| Function | Arduino | WCH32V003 | Peripheral | Notes |
|----------|---------|-----------|-----------|-------|
| **SPI Bus** |
| MOSI | D11 | PC5 | SPI1_MOSI | |
| MISO | D12 | PC6 | SPI1_MISO | |
| SCK | D13 | PC4 | SPI1_SCK | |
| **NRF24 Control** |
| NRF24 CE | D9 | PA1 | GPIO | Chip Enable |
| NRF24 CSN | D10 | PA2 | GPIO | Chip Select (active low) |
| **I2C (LCD)** |
| SDA | A4 | PC1 | I2C1_SDA | |
| SCL | A5 | PC2 | I2C1_SCL | |
| **Analog Inputs (ADC0/ADC1)** |
| Throttle Joystick | A3 | PC0 | ADC0_9 | |
| Yaw Joystick | A1 | PC1 | ADC0_11 | *Conflict with I2C - use PA5 instead* |
| Pitch Joystick | A2 | PC2 | ADC0_10 | *Conflict with I2C - use PA6 instead* |
| Roll Joystick | A6 | PA7 | ADC0_7 | |
| Potentiometer (Battery Monitor) | A7 | PC3 | ADC0_13 | Voltage divider input |
| **Digital Inputs (GPIO)** |
| Arm Toggle Switch 1 | D5 | PB5 | GPIO | Pull-up input |
| Arm Toggle Switch 2 | D3 | PB6 | GPIO | Pull-up input |

**⚠️ ADC Reallocation Required:**
- I2C uses PC1/PC2, so reassign joystick inputs to PA5/PA6
- Final ADC mapping:
  - PA1 → Yaw
  - PA5 → Pitch
  - PA7 → Roll
  - PC0 → Throttle
  - PC3 → Battery Voltage

### 2.2 Peripheral Availability Check
| Peripheral | Arduino | WCH32V003 | Status | Notes |
|------------|---------|-----------|--------|-------|
| SPI | Available | SPI1 (3x) | ✅ Compatible | Use SPI1 |
| I2C | Available | I2C1 | ✅ Compatible | 400kHz supported |
| ADC | 8-channel | 10-channel | ✅ Compatible | 12-bit ADC available |
| GPIO | 14 | 14 digital | ✅ Compatible | Plus RESET/BOOT |
| Timer | Available | TIM1/TIM2 | ✅ Compatible | For PWM if needed |
| UART | Available | UART1 | ✅ Compatible | Debug serial optional |

---

## Phase 3: Library Migration Strategy

### 3.1 NRF24 Library (Primary Dependency)

**Current:** Arduino RF24 library (v1.x)  
**Options for WCH32V003:**

1. **Option A: Use wch32fun Repository (Recommended)**
   - [ ] Check if wch32fun includes RF24 examples
   - [ ] If available, adapt existing CH32V003 RF24 code
   - Benefits: Optimized for RISC-V architecture, minimal dependencies

2. **Option B: Port Arduino RF24 Library**
   - [ ] Copy RF24 header/source files
   - [ ] Replace SPI library calls with wch32fun SPI API:
     ```c
     // Arduino:
     rf24.begin(CE_pin, CSN_pin);
     
     // WCH32V003 equivalent:
     spi_init(SPI1);
     gpio_pin_set(PA2, 1);  // CSN high initially
     ```
   - [ ] Update hardware abstraction layer:
     - `SPISettings` → wch32fun SPI configuration
     - `digitalWrite/Read` → `gpio_pin_set/get`
     - `delayMicroseconds` → `delay_us` from wch32fun
   - [ ] Verify SPI clock rate (NRF24 max 10MHz)

3. **Option C: Minimal Custom Implementation**
   - [ ] Write lightweight NRF24 driver using wch32fun
   - [ ] Implement only required commands:
     - Initialize RF24 (config registers)
     - Write TX payload
     - Read status
     - Set frequency/data rate
   - Benefits: Reduced code size, full optimization for WCH32V003
   - Effort: ~400-600 lines of code

**Recommendation:** Start with Option B (port existing library), fall back to Option C if needed.

### 3.2 I2C LCD Library

**Current:** LiquidCrystal_I2C  
**Migration Path:**
- [ ] Check wch32fun for I2C examples
- [ ] Replace with lightweight I2C implementation:
  ```c
  #include "ch32v003_i2c.h"
  
  // Initialize I2C1 at 400kHz
  i2c_init(I2C1, 400000);
  
  // Create simplified LCD wrapper
  void lcd_write_char(uint8_t addr, uint8_t data) {
    i2c_write(I2C1, addr, &data, 1);
  }
  ```
- [ ] Adapt LiquidCrystal code or use minimal LCD library

### 3.3 ADC Library

**Current:** Arduino `analogRead()`  
**Migration Path:**
- [ ] Use wch32fun ADC API:
  ```c
  #include "ch32v003_adc.h"
  
  uint16_t throttle = adc_read(ADC0, PA0);  // 0-4095 (12-bit)
  uint16_t voltage = adc_read(ADC0, PC3);
  ```
- [ ] Create ADC wrapper function:
  ```c
  uint8_t map_adc_8bit(uint16_t val, uint16_t lower, uint16_t middle, uint16_t upper) {
    // Convert 12-bit ADC to 8-bit channel value (0-255)
  }
  ```

### 3.4 GPIO Library

**Current:** Arduino `digitalWrite()`, `digitalRead()`  
**Migration Path:**
- [ ] Use wch32fun GPIO API:
  ```c
  #include "ch32v003_gpio.h"
  
  gpio_pin_set_mode(PA1, GPIO_MODE_OUT_PP, GPIO_SPEED_10MHZ);
  gpio_pin_set(PA1, 1);  // Set high
  uint8_t state = gpio_pin_get(PB5);  // Read toggle switch
  ```

### 3.5 Timing/Delay Library

**Current:** Arduino `millis()`, `delay()`, `delayMicroseconds()`  
**Migration Path:**
- [ ] Use wch32fun timer API or simple SysTick counter:
  ```c
  // Initialize 1kHz SysTick for millis()
  systick_init(48000000 / 1000);
  
  uint32_t get_millis(void) { return systick_ticks; }
  ```

---

## Phase 4: Code Structure Refactoring

### 4.1 Project Organization

```
wch32_nrf24_controller/
├── CMakeLists.txt
├── src/
│   ├── main.c                    # Entry point
│   ├── config.h                  # Hardware configuration
│   ├── radio.h / radio.c         # NRF24 driver
│   ├── joystick.h / joystick.c   # ADC reading & mapping
│   ├── lcd.h / lcd.c             # I2C LCD wrapper
│   ├── adc.h / adc.c             # ADC initialization
│   ├── spi.h / spi.c             # SPI wrapper
│   └── timers.h / timers.c       # Timing utilities
├── lib/
│   └── wch32fun/                 # Submodule or local copy
└── build/
```

### 4.2 Create Hardware Abstraction Layer (HAL)

**File: `src/hal.h`**
```c
// Pin definitions
#define RF24_CE_PIN    PA1
#define RF24_CSN_PIN   PA2
#define THROTTLE_PIN   PC0
#define YAW_PIN        PA1
#define PITCH_PIN      PA5
#define ROLL_PIN       PA7
#define BATTERY_PIN    PC3
#define ARM_SWITCH_1   PB5
#define ARM_SWITCH_2   PB6

// Common functions
void hal_init(void);
void hal_gpio_set_mode(GPIO_TypeDef *port, uint8_t pin, uint32_t mode);
void hal_gpio_set(GPIO_TypeDef *port, uint8_t pin, uint8_t state);
uint8_t hal_gpio_get(GPIO_TypeDef *port, uint8_t pin);
uint16_t hal_adc_read(uint8_t channel);
void hal_delay_ms(uint32_t ms);
void hal_delay_us(uint32_t us);
```

### 4.3 Main Execution Flow

```c
// main.c structure
#include "wch32v003fun.h"
#include "config.h"
#include "radio.h"
#include "joystick.h"
#include "lcd.h"

typedef struct {
  uint8_t throttle;
  uint8_t yaw;
  uint8_t pitch;
  uint8_t roll;
  uint8_t aux1;
  uint8_t aux2;
} ControlData;

ControlData control_data = {0};

void setup(void) {
  // 1. Initialize clocks (wch32fun framework)
  SystemInit();
  
  // 2. Initialize hardware peripherals
  spi_init();
  i2c_init();
  adc_init();
  gpio_init();
  
  // 3. Initialize modules
  radio_init();           // NRF24 setup
  lcd_init();            // LCD display
  control_data_reset();  // Zero all channels
  
  // 4. Show splash screen on LCD
  lcd_display_startup();
}

void loop(void) {
  // 1. Read analog inputs (joysticks, battery voltage)
  control_data.throttle = read_throttle();
  control_data.yaw = read_yaw();
  control_data.pitch = read_pitch();
  control_data.roll = read_roll();
  
  // 2. Read digital inputs (arm switches)
  control_data.aux1 = gpio_pin_get(ARM_SWITCH_1);
  control_data.aux2 = gpio_pin_get(ARM_SWITCH_2);
  
  // 3. Transmit via NRF24
  radio_write(&control_data, sizeof(control_data));
  
  // 4. Update LCD every 100ms
  if (should_update_lcd()) {
    lcd_update_display();
  }
}

int main(void) {
  setup();
  
  while (1) {
    loop();
  }
  
  return 0;
}
```

---

## Phase 5: Driver Implementation Details

### 5.1 NRF24 Driver Core Functions

```c
// radio.h
#ifndef RADIO_H
#define RADIO_H

#define NRF24_MAX_PAYLOAD 32

void radio_init(void);
void radio_write(const uint8_t *data, uint8_t len);
uint8_t radio_is_tx_ready(void);
void radio_set_channel(uint8_t channel);
void radio_set_power(uint8_t power);

#endif

// radio.c - Key implementation points:
// 1. SPI commands: W_REGISTER, R_REGISTER, W_TX_PAYLOAD, FLUSH_TX
// 2. Register init: CONFIG, EN_AA, EN_RXADDR, SETUP_AW, SETUP_RETR, RF_CH, RF_SETUP, STATUS
// 3. TX transmission with interrupt polling or status checking
```

### 5.2 ADC Mapping & Calibration

```c
// joystick.c - Porting the mapJoystickValues() function
uint8_t map_joystick_value(uint16_t adc_val, uint16_t lower, uint16_t middle, uint16_t upper, bool reverse) {
  adc_val = CLAMP(adc_val, lower, upper);
  
  if (adc_val < middle)
    adc_val = MAP(adc_val, lower, middle, 0, 128);
  else
    adc_val = MAP(adc_val, middle, upper, 128, 255);
  
  return reverse ? (255 - adc_val) : adc_val;
}

// ADC calibration values (from original Arduino code):
// Throttle: lower=13, middle=524, upper=1015
// Yaw: lower=50, middle=505, upper=1020
// Pitch: lower=12, middle=544, upper=1021
// Roll: lower=34, middle=522, upper=1020
```

### 5.3 I2C LCD Communication

```c
// lcd.c - Simplified I2C LCD wrapper
#define LCD_ADDR 0x27
#define LCD_WIDTH 16
#define LCD_HEIGHT 2

void lcd_init(void) {
  i2c_init(I2C1, 400000);
  // ... LCD initialization sequence
}

void lcd_print(const char *str) {
  // Write char-by-char via I2C
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
  // Set cursor position
}
```

---

## Phase 6: Building & Compilation

### 6.1 CMakeLists.txt Setup

```cmake
cmake_minimum_required(VERSION 3.10)
project(WCH32V003_NRF24_Controller)

# Toolchain
set(CMAKE_C_COMPILER riscv-none-embed-gcc)
set(CMAKE_CXX_COMPILER riscv-none-embed-g++)

# WCH32V003 Flags
set(MCU_FLAGS "-march=rv32ec -mabi=ilp32e -mno-div")
set(OPTIMIZE_FLAGS "-Os")

set(CMAKE_C_FLAGS "${MCU_FLAGS} ${OPTIMIZE_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -Wl,--gc-sections -nostartfiles")

# Source files
file(GLOB_RECURSE SOURCES "src/*.c")
file(GLOB_RECURSE HEADERS "src/*.h")

# Executable
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}.elf PRIVATE
  src/
  lib/wch32fun/
)

# Linking scripts
set(LINKER_SCRIPT "lib/wch32fun/linker.ld")
target_link_options(${PROJECT_NAME}.elf PRIVATE -T${LINKER_SCRIPT})

# Post-build: generate hex/bin
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND riscv-none-embed-objcopy -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
  COMMAND riscv-none-embed-size ${PROJECT_NAME}.elf
)
```

### 6.2 Makefile Alternative

```makefile
TOOLCHAIN_PATH ?= /opt/riscv-none-embed
CC = $(TOOLCHAIN_PATH)/bin/riscv-none-embed-gcc
OBJCOPY = $(TOOLCHAIN_PATH)/bin/riscv-none-embed-objcopy

MCU_FLAGS = -march=rv32ec -mabi=ilp32e -mno-div
OPTIMIZE = -Os
CFLAGS = $(MCU_FLAGS) $(OPTIMIZE) -Isrc/ -Ilib/wch32fun/

SOURCES = src/main.c src/radio.c src/joystick.c src/adc.c src/lcd.c
OBJECTS = $(SOURCES:.c=.o)

TARGET = wch32_controller.elf

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^ -Llib/wch32fun/ld -Twch32v003.ld

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)
```

---

## Phase 7: Testing & Validation

### 7.1 Unit Testing Plan

| Component | Test Case | Verification |
|-----------|-----------|---------------|
| **ADC Readings** | Read each joystick at min/center/max | Verify 0-4095 range |
| **Joystick Mapping** | Map 12-bit ADC to 8-bit channels | Output 0-255 range |
| **SPI Communication** | Send test pattern to NRF24 | Verify via SPI analyzer |
| **NRF24 Init** | Read CONFIG register | Expected value: 0x0C (PWR_UP + CRCO) |
| **TX Transmission** | Write 32-byte payload | Monitor CE pin timing |
| **GPIO Digital Inputs** | Read toggle switches | High/Low state changes |
| **I2C LCD** | Initialize & display text | Text appears on display |
| **Timing** | Measure loop execution | Should be ~15-20ms |

### 7.2 Debug Points

```c
// Insert these breakpoints/debug messages:
- radio_init(): Verify CONFIG register reads correctly
- spi_transfer(): Log all SPI transactions (optional logic analyzer)
- adc_read(): Verify 12-bit values in range
- main loop(): Monitor timing and cycle frequency
```

### 7.3 Functional Testing Flow

1. **Basic Hardware Test:**
   - Blink LED on GPIO
   - Read battery voltage
   - Display on LCD

2. **Joystick Calibration:**
   - Move each stick through full range
   - Verify ADC readings on LCD
   - Save calibration min/center/max values

3. **NRF24 Communication:**
   - Initialize radio with correct channel/address
   - Transmit test packet to receiver
   - Verify reception with receiver code

4. **Full System Integration:**
   - Read all inputs simultaneously
   - Transmit complete control packet (6 bytes)
   - Verify at 1000Hz transmission rate (if needed)

---

## Phase 8: Performance & Optimization

### 8.1 Memory Requirements

| Component | Size | Notes |
|-----------|------|-------|
| Code | ~8KB | GCC optimized |
| SPI driver | 1.5KB | |
| NRF24 driver | 3KB | |
| I2C driver | 1KB | |
| ADC driver | 0.5KB | |
| LCD interface | 1.5KB | |
| Global data | ~1KB | Control data + buffers |
| **Total** | **~16KB** | WCH32V003 has 16KB (fits!) |

WCH32V003 has:
- 16KB Flash (sufficient for this project)
- 2KB RAM (plenty for stack and variables)

### 8.2 Timing Budget

| Task | Time | Notes |
|------|------|-------|
| SPI init | <1ms | One-time in setup |
| I2C init | <1ms | One-time in setup |
| ADC read (1ch) | ~10µs | 12-bit @ 48MHz |
| Read 6 ADC channels | ~60µs | Throttle, Yaw, Pitch, Roll, Battery, spare |
| NRF24 TX | ~150µs | SPI transfer + status check |
| GPIO reads (2 switches) | <1µs | Direct port read |
| I2C LCD update (100ms interval) | ~5-10ms | Non-blocking if async |
| **Loop cycle** | **~170µs** | ≈5.9kHz loop rate |

### 8.3 Power Consumption

- WCH32V003 @ 48MHz: ~15mA (core)
- NRF24L01+ (TX mode): ~12-14mA
- I2C LCD: ~2-3mA (backlight on)
- **Total (TX mode):** ~30-40mA
- **Total (RX idle):** ~2-5mA

**Recommendation:** Use single 3.7V LiPo battery with 650mAh capacity (≈1-2 hours flight time)

---

## Phase 9: Implementation Checklist

### 9.1 Pre-Implementation
- [ ] Clone wch32fun repository
- [ ] Read WCH32V003 datasheet (register definitions)
- [ ] Procure development board (CH32V003 EVT board)
- [ ] Set up debugger (CH-DEBUG or similar)

### 9.2 Core Development
- [ ] **Week 1:** HAL & basic GPIO/SPI/ADC drivers
  - [ ] Pin configuration
  - [ ] SPI master init
  - [ ] ADC multi-channel reader
  - [ ] GPIO input with pull-up
  - [ ] Test with LED blink + ADC read

- [ ] **Week 2:** NRF24 Driver
  - [ ] Port RF24 SPI commands
  - [ ] Initialize NRF24 registers
  - [ ] Implement TX function
  - [ ] Test with original receiver

- [ ] **Week 3:** Input Processing
  - [ ] ADC calibration (joystick min/center/max)
  - [ ] Joystick value mapping
  - [ ] Toggle switch debouncing

- [ ] **Week 4:** Integration & Testing
  - [ ] LCD I2C interface
  - [ ] Battery voltage monitoring
  - [ ] Full system integration
  - [ ] Bench testing with receiver

### 9.3 Validation
- [ ] [ ] All joysticks produce 0-255 output
- [ ] [ ] NRF24 transmits at correct frequency
- [ ] [ ] Receiver decodes packets correctly
- [ ] [ ] Toggle switches register state changes
- [ ] [ ] Battery voltage displayed on LCD
- [ ] [ ] Loop executes at ~5kHz minimum
- [ ] [ ] Power consumption < 50mA in TX mode
- [ ] [ ] Flight range > 50 meters (outdoor)

---

## Phase 10: Known Issues & Troubleshooting

### 10.1 Common Pitfalls

| Issue | Cause | Solution |
|-------|-------|----------|
| NRF24 not responding | Wrong SPI pinout | Verify CE/CSN/MOSI/MISO pins |
| NRF24 in low power | CONFIG register PWR_UP bit not set | Check init sequence |
| ADC reads 0 always | ADC clock not enabled in RCC | Enable ADC in SystemInit() |
| I2C LCD no response | Address not 0x27, or clock too fast | Try 100kHz, check address |
| SPI glitches | GPIO not in push-pull mode | Set GPIO_MODE_OUT_PP |
| Joystick values inverted | Reverse flag not working | Check mapJoystickValues logic |

### 10.2 Debug Options

1. **UART Debug Serial:**
   - Add UART1 for printf() debugging
   - Print joystick raw values
   - Log NRF24 register states

2. **Logic Analyzer:**
   - Monitor SPI transactions
   - Verify I2C protocol
   - Check GPIO timing

3. **Oscilloscope:**
   - Verify SPI clock rate
   - Check CE/CSN pulse timing
   - Monitor voltage rail stability

---

## Phase 11: Optimization & Advanced Features (Future)

### 11.1 Optional Enhancements

- [ ] **EEPROM Storage:** Save calibration data on WCH32V003's internal EEPROM
- [ ] **Telemetry RX:** Read battery voltage from receiver via second NRF24 pipe
- [ ] **Fail-Safe:** Detect lost connection, cut throttle automatically
- [ ] **LED Indicators:** Status LED (power, TX, battery warning)
- [ ] **Bootloader:** Makes re-flashing easier

### 11.2 Frequency/Data Rate Options

```c
// NRF24 data rates:
#define RF24_1MBPS   0x00
#define RF24_2MBPS   0x08
#define RF24_250KBPS 0x20

// Original used RF24_250KBPS for better range
// WCH32V003 can handle any rate without performance issues
```

### 11.3 Power Saving

```c
// Low-power mode for idle periods:
void radio_sleep_mode(void) {
  // Set PRIM_RX = 0, PWR_UP = 0
  // WCH32V003 can enter STOP mode
  // Wake on joystick movement (ADC interrupt)
}
```

---

## References

### Official Documentation
- [WCH32V003 Datasheet](https://www.wch.cn/downloads/ch32v003_datasheet.pdf)
- [WCH32fun GitHub](https://github.com/cnlohr/ch32v003fun)
- [nRF24L01+ Datasheet](https://www.sparkfun.com/datasheets/RF/nRF24L01p_Preliminary_Product_Specification_v1_0.pdf)

### Related Projects
- [WCH32V003 Mini Example Code](https://github.com/cnlohr/ch32v003fun/tree/master/examples)
- [Original Arduino RF24 Library](https://github.com/tmrh20/RF24)
- [RISC-V Toolchain Setup](https://github.com/riscv-collab/riscv-gnu-toolchain)

### Tools & Resources
- **PlatformIO:** Simplified project setup
- **OpenOCD:** Debugging via JTAG
- **STM32CubeiDE:** Can work with CH32 files (check compatibility)

---

## Summary

This migration plan provides a systematic approach to porting the Arduino Nano NRF24 controller to the WCH32V003. The main steps are:

1. **Set up WCH32fun development environment**
2. **Create HAL for hardware abstraction**
3. **Port NRF24 driver** (most complex part)
4. **Implement ADC joystick reading** (straightforward)
5. **Add I2C LCD support** (optional but recommended)
6. **Integrate all modules** and test with original receiver
7. **Optimize for performance & power**

**Estimated Timeline:** 4-5 weeks for full functional system  
**Complexity Level:** Intermediate (requires embedded systems knowledge)  
**Success Probability:** High (~90%) with wch32fun framework + existing RF24 library

---

*Last Updated: 2026-04-24*  
*Target Platform: WCH32V003 20-pin LQFP*  
*Ported From: Arduino Nano ATmega328P*
