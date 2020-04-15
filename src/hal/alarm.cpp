
#include <stdbool.h>

#include "../hal/hal.h"
#include "../hal/alarm.h"
#include "../hal/timer.h"

#include <Arduino.h>

int alarmHalInit(void)
{
  pinMode(ALARM_PIN, OUTPUT);
  
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
    noTone(ALARM_PIN);
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
        noTone(ALARM_PIN);
        alarmOn = false;
        timerHalBegin(&timer, alarmTime);
      }
    }
  } else {
    // If a constant alarm, just turn it on an leave it on until a pattern change
    if (currentPattern == ALARM_HAL_CONSTANT) {
      tone(ALARM_PIN, ALARM_FREQUENCY);
      alarmOn = true;
    } else if (currentPattern != ALARM_HAL_OFF) {
      // If a blinking buzzer (ie, not off or constant), check the running timer
      if (timerHalRun(&timer) != HAL_IN_PROGRESS) {
        tone(ALARM_PIN, ALARM_FREQUENCY);
        alarmOn = true;
        timerHalBegin(&timer, alarmTime);
      }
    }
  }

  return HAL_IN_PROGRESS;
}