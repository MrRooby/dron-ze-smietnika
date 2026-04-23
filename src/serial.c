#include "serial.h"
#include "stm8s_uart2.h"
#include <string.h>

// Circular RX buffer
#define RX_BUFFER_SIZE 128
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_head = 0;
static uint16_t rx_tail = 0;

// Rotor PWM data
static RotorPWM_t rotor_pwm = {0, 0, 0, 0, 0};

// Function prototypes
static void parse_rotor_packet(void);
static uint8_t parse_hex_value(const char* str);

void Serial_begin(uint32_t baud_rate) {
  // Enable UART2 clock
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE);
  
  // Initialize UART2 for TX and RX
  UART2_Init(
      (uint32_t)baud_rate, 
      UART2_WORDLENGTH_8D, 
      UART2_STOPBITS_1, 
      UART2_PARITY_NO, 
      UART2_SYNCMODE_CLOCK_DISABLE, 
      UART2_MODE_TXRX_ENABLE);

  // Enable RX interrupt
  UART2_ITConfig(UART2_IT_RXNE, ENABLE);
  
  // Enable UART2
  UART2_Cmd(ENABLE);
}

void Serial_print_char(char value) {
    while (!(UART2->SR & 0x80)); // Wait for TXE
    UART2->DR = value;
    while (!(UART2->SR & 0x40)); // Wait for TC
}

bool Serial_available(void) {
    return (UART2->SR & 0x20) ? TRUE : FALSE;
}

RotorPWM_t* Serial_getRotorPWM(void) {
    return &rotor_pwm;
}

// UART2 interrupt handler - receives data from UART
void UART2_RxIRQHandler(void) __interrupt(21)
{
    uint8_t byte = UART2->DR;
    
    // Add byte to circular buffer
    rx_buffer[rx_head] = byte;
    rx_head = (rx_head + 1) % RX_BUFFER_SIZE;
    
    // Check for frame delimiter (newline indicates end of packet)
    if (byte == '\n') {
        parse_rotor_packet();
    }
}

// Parse incoming rotor packet
// Expected format: "O1:XX-O2:XX-B1:XX-B2:XX\n"
static void parse_rotor_packet(void) {
    uint8_t temp_buffer[RX_BUFFER_SIZE];
    uint16_t packet_size = 0;
    uint8_t o1 = 0, o2 = 0, b1 = 0, b2 = 0;
    uint8_t valid = 0;
    
    // Extract packet from circular buffer up to newline
    uint16_t idx = rx_tail;
    while (idx != rx_head && packet_size < RX_BUFFER_SIZE - 1) {
        uint8_t c = rx_buffer[idx];
        if (c == '\n') {
            temp_buffer[packet_size] = '\0';
            rx_tail = (idx + 1) % RX_BUFFER_SIZE;
            break;
        }
        temp_buffer[packet_size++] = c;
        idx = (idx + 1) % RX_BUFFER_SIZE;
    }
    
    // Parse format: "O1:XX-O2:XX-B1:XX-B2:XX"
    if (packet_size > 20) { // Minimum expected length
        
        // Find O1 value (should start with O1:)
        if (temp_buffer[0] == 'O' && temp_buffer[1] == '1' && temp_buffer[2] == ':' && packet_size > 5) {
            o1 = parse_hex_value((char*)&temp_buffer[3]);
            valid = 1;
        } else {
            return;  // Invalid first field
        }
        
        // Find O2 value (look for -O2: or O2:)
        uint16_t i = 6;  // Start after O1:XX-
        while (i < packet_size && temp_buffer[i] != 'O') i++;
        if (i < packet_size - 3 && temp_buffer[i+1] == '2' && temp_buffer[i+2] == ':') {
            o2 = parse_hex_value((char*)&temp_buffer[i+3]);
        } else {
            return;  // Invalid O2 field
        }
        
        // Find B1 value (look for -B1: or B1:)
        i += 6;  // Start after O2:XX-
        while (i < packet_size && temp_buffer[i] != 'B') i++;
        if (i < packet_size - 3 && temp_buffer[i+1] == '1' && temp_buffer[i+2] == ':') {
            b1 = parse_hex_value((char*)&temp_buffer[i+3]);
        } else {
            return;  // Invalid B1 field
        }
        
        // Find B2 value (look for -B2: or B2:)
        i += 6;  // Start after B1:XX-
        while (i < packet_size && temp_buffer[i] != 'B') i++;
        if (i < packet_size - 3 && temp_buffer[i+1] == '2' && temp_buffer[i+2] == ':') {
            b2 = parse_hex_value((char*)&temp_buffer[i+3]);
        } else {
            return;  // Invalid B2 field
        }
        
        // All fields parsed successfully, update rotor PWM
        rotor_pwm.O1 = o1;
        rotor_pwm.O2 = o2;
        rotor_pwm.B1 = b1;
        rotor_pwm.B2 = b2;
        rotor_pwm.valid = 1;
    }
}

// Parse two hex characters to 8-bit value
static uint8_t parse_hex_value(const char* str) {
    uint8_t result = 0;
    
    if (str[0] >= '0' && str[0] <= '9') {
        result += (str[0] - '0') << 4;
    } else if (str[0] >= 'A' && str[0] <= 'F') {
        result += (str[0] - 'A' + 10) << 4;
    } else if (str[0] >= 'a' && str[0] <= 'f') {
        result += (str[0] - 'a' + 10) << 4;
    }
    
    if (str[1] >= '0' && str[1] <= '9') {
        result += (str[1] - '0');
    } else if (str[1] >= 'A' && str[1] <= 'F') {
        result += (str[1] - 'A' + 10);
    } else if (str[1] >= 'a' && str[1] <= 'f') {
        result += (str[1] - 'a' + 10);
    }
    
    return result;
}

// Retarget printf to use Serial_print_char
int putchar(int c) {
    Serial_print_char((char)c);
    return c;
}
