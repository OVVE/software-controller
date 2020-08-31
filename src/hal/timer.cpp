/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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