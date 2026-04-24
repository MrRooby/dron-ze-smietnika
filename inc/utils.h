#ifndef UTILS_H
#define UTILS_H
#include "stm8s.h"

static inline void DelayDumb(const uint32_t ms){
  for (volatile uint32_t i = 0; i < (ms * 800); i++);
}

#endif // !UTILS_H
