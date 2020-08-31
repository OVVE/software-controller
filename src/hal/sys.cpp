/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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
  digitalWrite(POWEROFF_PIN, POWEROFF_ASSERT);
  
  // Wait for either the system to power off, or the watchdog to fire...
  while(1);
}

bool sysHalPowerButtonAsserted(void)
{
  return (digitalRead(POWERBTN_PIN) == POWERBTN_ASSERT_OFF);
}
