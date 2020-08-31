
#include <stdbool.h>

#include <Arduino.h>

#include "../hal/hal.h"
#include "../hal/estop.h"

#define ESTOP_PIN                4
#define ESTOP_ASSERTION_LEVEL  LOW

int estopHalInit(void)
{
  pinMode(ESTOP_PIN, INPUT);
  return HAL_OK;
}

bool estopHalAsserted(void)
{
  return (digitalRead(ESTOP_PIN) == ESTOP_ASSERTION_LEVEL);
}