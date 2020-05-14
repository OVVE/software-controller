
#include <stdint.h>

#include "../../hal/hal.h"
#include "../../hal/sensor/adc.h"
#include "../../hal/sensor/battery.h"

#define BATTERY_SENSOR_PIN  3
#define SENSOR_RESOLUTION   1024
#define BATTERY_RANGE       5
#define VOLTAGE_DIVISION    4

static int16_t reading;

int batterySensorHalInit(void)
{
  adcHalInit();

  return HAL_OK;
}

int batterySensorHalFetch(void)
{
  if (!adcHalBusy()) {
    adcHalBegin(BATTERY_SENSOR_PIN);
  } else if ((adcHalGetCurrentPin() == BATTERY_SENSOR_PIN) && !adcHalInProgress()) {
    reading = adcHalGetValue();
    adcHalComplete();
    return HAL_OK;
  }

  return HAL_IN_PROGRESS;
}

int batterySensorHalGetValue(float16_t* value)
{
  *value = ((float16_t) reading / SENSOR_RESOLUTION) * (BATTERY_RANGE * VOLTAGE_DIVISION)
  return HAL_OK;
}