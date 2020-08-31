
#include <stdint.h>
#include <stdbool.h>

#include "../hal/hal.h"
#include "../hal/timer.h"

#include <Arduino.h>

int timerHalInit(void)
{
  // No initialized needed in this case
  return HAL_OK;
}

int timerHalBegin(struct timer* timer, uint32_t duration, bool periodic)
{
  if (timer) {
    timer->start = (uint32_t) micros();
    timer->duration = duration;
    timer->periodic = periodic;
    return HAL_OK;
  }
  return HAL_FAIL;
}

int timerHalRun(struct timer* timer)
{
  if (timer) {
    // Check to see if the time has expired
    if (((uint32_t) micros()) - timer->start < timer->duration) {
      return HAL_IN_PROGRESS;
    } else {
      // In order to keep more precise periodic timers, add the duration
      // to the start time once the timer has timed out
      if (timer->periodic) {
        timer->start += timer->duration;
      }
      return HAL_TIMEOUT;
    }
  }
  return HAL_FAIL;
}

uint32_t timerHalCurrent(struct timer* timer)
{
  return ((uint32_t) micros()) - timer->start;
}