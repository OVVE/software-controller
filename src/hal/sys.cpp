
#include <stdint.h>
#include <stdbool.h>

#include <Arduino.h>

#include "../hal/sys.h"

int sysHalInit(void)
{
  pinMode(POWERBTN_PIN, INPUT);
  digitalWrite(POWEROFF_PIN, POWEROFF_DEASSERT);
  pinMode(POWEROFF_PIN, OUTPUT);
  
  return HAL_OK;
}

uint32_t sysHalTime(void)
{
  return (uint32_t) millis();
}

void sysHalPowerOff(void)
{
  digitalWrite(POWEROFF_PIN, POWEROFF_DEASSERT);
  
  // Wait for either the system to power off, or the watchdog to fire...
  while(1);
}

bool sysHalPowerButtonAsserted(void)
{
  return (digitalRead(POWERBTN_PIN) == POWERBTN_ASSERT_OFF);
}