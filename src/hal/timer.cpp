
#include <stdint.h>

#include "../hal/hal.h"
#include "../hal/timer.h"

#include <Arduino.h>

int timerHalInit(void)
{
  // No initialized needed in this case
  return HAL_OK;
}

int timerHalBegin(struct timer* timer, uint32_t duration)
{
  if (timer) {
    timer->start = (uint32_t) micros();
    timer->duration = duration;
    return HAL_OK;
  }
  return HAL_FAIL;
}

int timerHalRun(struct timer* timer)
{
  if (timer) {
    if (((uint32_t) micros()) - timer->start < timer->duration) {
      return HAL_IN_PROGRESS;
    } else {
      return HAL_TIMEOUT;
    }
  }
  return HAL_FAIL;
}

uint32_t timerHalCurrent(struct timer* timer)
{
  return ((uint32_t) micros()) - timer->start;
}