#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm8s.h"
#include <stdint.h>

// Rotor PWM data structure
typedef struct {
  uint8_t O1;  // Front rotor
  uint8_t O2;  // Back rotor
  uint8_t B1;  // Left rotor
  uint8_t B2;  // Right rotor
  uint8_t valid; // Flag indicating if data is valid
} RotorPWM_t;

// Initialize UART2 with a custom baud rate and RX interrupts
void Serial_begin(uint32_t baud_rate);

// Send a single character
void Serial_print_char(char value);

// Check if UART has received data
bool Serial_available(void);

// Get the latest rotor PWM values
// Returns pointer to RotorPWM_t structure, valid flag indicates if data is current
RotorPWM_t* Serial_getRotorPWM(void);

#endif
