#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm8s.h"

// Initialize USART1 with a custom baud rate
void Serial_begin(uint32_t baud_rate);

// Send a single character
void Serial_print_char(char value);

// Read a single character (blocking)
char Serial_read_char(void);

// Check if data is waiting in the buffer
bool Serial_available(void);

#endif
