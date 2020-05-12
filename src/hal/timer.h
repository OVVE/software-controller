
#ifndef __TIMER_HAL_H__
#define __TIMER_HAL_H__

#include <stdint.h>
#include <stdbool.h>

#include "../hal/hal.h"

// Time units
#define SEC   * 1000000UL
#define MSEC  * 1000UL
#define USEC  * 1UL

struct timer {
  uint32_t start;
  uint32_t duration : 31;
  bool     periodic : 1;
};

// TODO: Doc
int timerHalInit(void);

// TODO: Doc
int timerHalBegin(struct timer* timer, uint32_t duration, bool periodic);

// TODO: Doc
int timerHalRun(struct timer* timer);

// TODO: Doc
uint32_t timerHalCurrent(struct timer* timer);

#endif /* __TIMER_HAL_H__ */