
#ifndef __TIMER_HAL_H__
#define __TIMER_HAL_H__

#include "../hal/hal.h"

// Time units
#define SEC   * 1
#define MSEC  * 1000
#define USEC  * 1000000

struct timer {
  unsigned int start;
  unsigned int duration;
};

// TODO: Doc
int timerHalInit(void);

// TODO: Doc
int timerHalBegin(struct timer* timer, unsigned int duration);

// TODO: Doc
int timerHalRun(struct timer* timer);

#endif /* __TIMER_HAL_H__ */