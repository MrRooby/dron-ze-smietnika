#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>

/**
 * Crude delay using CPU loops (no timer)
 * CPU: 16MHz, roughly 1us per 16 cycles
 * @param ms: Milliseconds to delay
 */
inline void delayDumb(const uint32_t ms);

#endif
