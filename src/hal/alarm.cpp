/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdbool.h>

#include "../hal/hal.h"
#include "../hal/alarm.h"
#include "../hal/timer.h"

#include <Arduino.h>

int alarmHalInit(void)
{
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, ALARM_ASSERT_OFF);
  
  return HAL_OK;
}

int alarmHalRing(unsigned int pattern)
{
  static unsigned int currentPattern = ALARM_HAL_OFF;
  static bool alarmOn = false;
  static struct timer timer;
  uint32_t alarmTime;
  
  // Check validity of pattern
  if (pattern > ALARM_HAL_CONSTANT) {
    return HAL_FAIL;
  }

  // Check to see if new tone needs to be played
  if (pattern != currentPattern) {
    digitalWrite(ALARM_PIN, ALARM_ASSERT_OFF);
    currentPattern = pattern;
    alarmOn = false;
  }
  
  // Convert alarm frequency into time in seconds
  switch (pattern) {
    case ALARM_HAL_0_25HZ:
    alarmTime = 4 SEC;
    break;
    case ALARM_HAL_0_5HZ:
    alarmTime = 2 SEC;
    break;
    case ALARM_HAL_1HZ:
    alarmTime = 1 SEC;
    break;
    default:
    alarmTime = 0 SEC;
    break;
  }
  
  if (alarmOn) {
    // If a blinking buzzer, check the running timer
    if (currentPattern != ALARM_HAL_CONSTANT) {
      if (timerHalRun(&timer) != HAL_IN_PROGRESS) {
        digitalWrite(ALARM_PIN, ALARM_ASSERT_OFF);
        alarmOn = false;
        timerHalBegin(&timer, alarmTime, false);
      }
    }
  } else {
    // If a constant alarm, just turn it on an leave it on until a pattern change
    if (currentPattern == ALARM_HAL_CONSTANT) {
      digitalWrite(ALARM_PIN, ALARM_ASSERT_ON);
      alarmOn = true;
    } else if (currentPattern != ALARM_HAL_OFF) {
      // If a blinking buzzer (ie, not off or constant), check the running timer
      if (timerHalRun(&timer) != HAL_IN_PROGRESS) {
        digitalWrite(ALARM_PIN, ALARM_ASSERT_ON);
        alarmOn = true;
        timerHalBegin(&timer, alarmTime, false);
      }
    }
  }

  return HAL_IN_PROGRESS;
}