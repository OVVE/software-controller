
#include <Arduino.h>

#include "../hal/hal.h"
#include "../hal/timer.h"

// #define DEBUG
#define DEBUG_MODULE "timer"
#include "../util/debug.h"

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
    // TODO: Handle overflow of the timer used by micros().
    if ((((uint32_t) micros()) - timer->start) < timer->duration) {
      return HAL_IN_PROGRESS;
    } else {
      return HAL_TIMEOUT;
    }
  }
  return HAL_FAIL;
}
