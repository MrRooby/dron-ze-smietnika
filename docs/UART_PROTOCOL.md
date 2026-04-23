# STM8S005 Drone UART Protocol Documentation

## Overview
The BrushDC drone communicates with an ESP32 controller via UART2 at **115200 baud** using interrupt-driven reception. The protocol transmits rotor PWM values in a simple ASCII packet format.

## Packet Format
```
O1:<8-bit value>-O2:<8-bit value>-B1:<8-bit value>-B2:<8-bit value>\n
```

### Field Definitions
| Field | Description | PWM Timer |
|-------|-------------|-----------|
| **O1** | Front rotor PWM (0-255) | TIM1_SetCompare3 |
| **O2** | Back rotor PWM (0-255) | TIM1_SetCompare4 |
| **B1** | Left rotor PWM (0-255) | TIM2_SetCompare2 |
| **B2** | Right rotor PWM (0-255) | TIM2_SetCompare3 |

## Data Format Details

### Values
- **Format**: Hexadecimal (0x00 - 0xFF or 00 - FF in decimal)
- **Range**: 0-255 PWM output
- **Separator**: Single dash `-` between fields
- **Terminator**: Newline character `\n` (0x0A)

### Examples

**Test Packet 1** (All rotors at 50% speed)
```
O1:7F-O2:7F-B1:7F-B2:7F\n
```
(Hexadecimal 7F = 127 decimal)

**Test Packet 2** (Ramp up front rotors, keep rear at 0)
```
O1:50-O2:00-B1:30-B2:30\n
```

**Test Packet 3** (Maximum throttle)
```
O1:FF-O2:FF-B1:FF-B2:FF\n
```

**Test Packet 4** (Hover - moderate speed)
```
O1:A0-O2:A0-B1:80-B2:80\n
```

## Implementation Details

### The Code
The protocol is implemented in `custom_lib/serial.c` and `custom_lib/serial.h`:

**Key Features:**
- **Circular RX Buffer**: 128-byte buffer handles bursts safely
- **Interrupt-Driven**: UART2 interrupt (vector 21) processes incoming data
- **Packet Validation**: Verifies all four fields present before updating
- **Hex Parsing**: Automatically converts hex strings to 8-bit values

### Using the Protocol in Code

```c
// In main.c, get the latest rotor data
RotorPWM_t* pwm_data = Serial_getRotorPWM();

if (pwm_data->valid) {
    // New packet received, update outputs
    TIM1_SetCompare3(pwm_data->O1);  // Front
    TIM1_SetCompare4(pwm_data->O2);  // Back
    TIM2_SetCompare2(pwm_data->B1);  // Left
    TIM2_SetCompare3(pwm_data->B2);  // Right
    
    // Clear valid flag after processing
    pwm_data->valid = 0;
}
```

## Testing

### Via Serial Terminal (Linux)
```bash
# Connect to your programmer/terminal
screen /dev/ttyUSB0 115200

# Send a test packet (press manual newline after)
# Type: O1:7F-O2:7F-B1:7F-B2:7F
# Press: Ctrl+J (or Enter)
```

### Via Python Script
```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Send rotor command: Front=128, Back=128, Left=80, Right=80
packet = "O1:80-O2:80-B1:50-B2:50\n"
ser.write(packet.encode())

time.sleep(0.1)
ser.close()
```

### Via ESP32 (C++ Example)
```cpp
#include <HardwareSerial.h>

HardwareSerial stm8Serial(1);  // UART1 on ESP32

void setup() {
    stm8Serial.begin(115200, SERIAL_8N1, RXD, TXD);
}

void loop() {
    // Send rotor PWM command
    uint8_t O1 = 100, O2 = 100, B1 = 50, B2 = 50;
    
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "O1:%02X-O2:%02X-B1:%02X-B2:%02X\n", 
             O1, O2, B1, B2);
    
    stm8Serial.print(buffer);
    delay(10);
}
```

## Timing & Constraints

| Parameter | Value |
|-----------|-------|
| Baud Rate | 115200 |
| Packet Format | O1:XX-O2:XX-B1:XX-B2:XX\n |
| Min Bytes/Packet | 28 bytes |
| Max Bytes/Packet | 28 bytes |
| Max Transfer Time | ~2.4 ms (at 115200 baud) |
| RX Buffer Size | 128 bytes |

## Error Handling

The parser automatically handles:
- ✓ Hexadecimal values (uppercase or lowercase)
- ✓ Field reordering (scans for field markers)
- ✓ Extra whitespace (ignores non-hex characters between values)
- ✗ Missing fields → packet rejected
- ✗ Invalid format → packet rejected

If a packet is malformed, the `valid` flag remains 0 and the previous PWM values persist.

## Hardware Connections

**STM8S005 UART2 Pins:**
- **TX (PD5)**: To ESP32 RX (optional, for debug/feedback)
- **RX (PD6)**: From ESP32 TX (main data input)

## Performance Notes

- **Interrupt-based reception**: Minimal CPU overhead, deterministic
- **No DMA**: STM8S005 has very limited DMA, interrupt method is optimal
- **Buffer overflow protection**: Circular buffer handles rapid packets
- **CPU time**: <100µs per interrupt at 115200 baud

## Troubleshooting

### No data received
- Check UART2 RX (PD6) connection to ESP32 TX
- Verify baud rate matches (115200)
- Check that RX interrupt is enabled: `UART2_ITConfig(UART2_IT_RXNE, ENABLE)`

### Intermittent data loss
- May indicate buffer overflow; increase RX_BUFFER_SIZE in serial.c
- Check for competing ISRs with higher priority

### Incorrect PWM values
- Verify hex format (must be 2 characters per value)
- Check for extra characters between fields
- Use Python/serial terminal to debug raw packet transmission

---
**Last Updated**: April 2026  
**Tested with**: STM8S005K6T6, SDCC compiler, ESP32
