#include "utils.h"

void delay_ms_dumb(const uint32_t ms) {
  for (volatile uint32_t i = 0; i < (ms * 800); i++);
}
