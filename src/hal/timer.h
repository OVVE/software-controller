
#ifndef __TIMER_HAL_H__
#define __TIMER_HAL_H__

#include "../hal/hal.h"

#include <stdint.h> 

// Time units normalized to microseconds.
#define  SEC  * 1000000UL
#define MSEC  * 1000UL
#define USEC  * 1UL

struct timer {
  uint32_t start;
  uint32_t duration;
};

// TODO: Doc
int timerHalInit(void);

// TODO: Doc
int timerHalBegin(struct timer* timer, uint32_t duration);

// TODO: Doc
int timerHalRun(struct timer* timer);

#endif /* __TIMER_HAL_H__ */
