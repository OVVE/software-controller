
#include <Arduino.h>

#include "../hal/hal.h"
#include "../hal/timer.h"

int timerHalInit(void)
{
  // No initialized needed in this case
  return HAL_OK;
}

int timerHalBegin(struct timer* timer, unsigned int duration)
{
  if (timer) {
    timer->start = (unsigned int) micros();
    timer->duration = duration;
    return HAL_OK;
  }
  return HAL_FAIL;
}

int timerHalRun(struct timer* timer)
{
  if (timer) {
    if ((unsigned int) micros() - timer->start < timer->duration) {
      return HAL_IN_PROGRESS;
    } else {
      return HAL_TIMEOUT;
    }
  }
  return HAL_FAIL;
}