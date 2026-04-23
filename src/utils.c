#include "utils.h"

inline void delayDumb(const uint32_t ms) {
  for (volatile uint32_t i = 0; i < (ms * 800); i++);
}
