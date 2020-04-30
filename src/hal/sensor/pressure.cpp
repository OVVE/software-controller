
#include <stdint.h>

#include "../../hal/hal.h"
#include "../../hal/sensor/adc.h"
#include "../../hal/sensor/pressure.h"

#define PRESSURE_SENSOR_PIN 1

//#define PRESSURE_SENSOR_MPXV7025
#define PRESSURE_SENSOR_MPXV7007

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
#if defined(PRESSURE_SENSOR_MPXV7025)
  // For the MPXV7025 pressure sensor: (reading / 1024 - 0.5) / 0.018 = P in kPa
  // From Calvin, why these exact values and the /4, Im not sure
  int32_t pascals = (((int32_t)reading * 217L) - 110995L)>>2; // convert to Pascals
#elif defined(PRESSURE_SENSOR_MPXV7007)
  // For the MPXV7007 pressure sensor: (reading / 1024 - 0.5) / 0.057 = P in kPa
  //                                   (reading / 1024 - 0.5) / (57 / 1000) * 1000 in Pa
  //                                   (reading / 1024 - 0.5) * 1000000 / 57
  //                                   (reading * 1000000 / 57 / 1024 - 500000 / 57)
  //                                   (reading * 17.1327 - 8771.9298) * 8 / 8  ; Help make multiplier error smaller
  //                                   (reading * 137.0614 - 70175.4386) / 8
  int32_t pascals = (((int32_t)reading * 137L) - 70175L)>>3; // convert to Pascals
#endif
  int32_t u100umH2O = (pascals * 4177L)>>12;                  // convert Pascals to 0.1mmH2O
  *value = (int16_t)u100umH2O;                                // return as 16 bit signed int
  return HAL_OK;
}