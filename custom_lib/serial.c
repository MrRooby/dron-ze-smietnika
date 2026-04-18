#include "serial.h"

void Serial_begin(uint32_t baud_rate) {
  UART2_Init(
      (uint32_t)baud_rate, 
      UART2_WORDLENGTH_8D, 
      UART2_STOPBITS_1, 
      UART2_PARITY_NO, 
      UART2_SYNCMODE_CLOCK_DISABLE, 
      UART2_MODE_TXRX_ENABLE);

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

// Retarget printf to use Serial_print_char
int putchar(int c) {
    Serial_print_char((char)c);
    return c;
}
