
#include <stdint.h>

#include "../../hal/hal.h"
#include "../../hal/sensor/adc.h"
#include "../../hal/sensor/pressure.h"

#define PRESSURE_SENSOR_PIN 1

static int16_t reading;

int pressureSensorHalInit(void)
{
  adcHalInit();

  return HAL_OK;
}

int pressureSensorHalFetch(void)
{
  // If the ADC is free, kick off our conversion; otherwise if its no longer
  // converting our pin, take the value and free the ADC
  if (!adcHalBusy()) {
    adcHalBegin(PRESSURE_SENSOR_PIN);
  } else if ((adcHalGetCurrentPin() == PRESSURE_SENSOR_PIN) && !adcHalInProgress()) {
    reading = adcHalGetValue();
    adcHalComplete();
    return HAL_OK;
  }
    
  return HAL_IN_PROGRESS;
}

int pressureSensorHalGetValue(int16_t* value)
{
  // For the MPXV7025 pressure sensor: (reading / 1024 - 0.5) / 0.018 = P in kPa
  int32_t pascals = (((int32_t)reading * 217L) - 110995L)>>2; // convert to Pascals
  int32_t u100umH2O = (pascals * 4177L)>>12;                  // convert Pascals to 0.1mmH2O
  *value = (int16_t)u100umH2O;                                // return as 16 bit signed int
  return HAL_OK;
}