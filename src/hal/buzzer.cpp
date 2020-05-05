#include "../hal/buzzer.h"

#include <Arduino.h>

#define PIN_BUZZER 3

int buzzerHalInit(void)
{
  pinMode(PIN_BUZZER, OUTPUT);
  return buzzerHalOff();
}

int buzzerHalOff(void)
{
  digitalWrite(PIN_BUZZER, LOW);

  return HAL_OK;
}

int buzzerHalOn(void)
{
  digitalWrite(PIN_BUZZER, HIGH);
  return HAL_OK;
}

