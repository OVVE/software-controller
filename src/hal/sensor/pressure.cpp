/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdint.h>

#include "../../config.h"

#include "../../hal/hal.h"
#include "../../hal/sensor/adc.h"
#include "../../hal/sensor/pressure.h"
#include "../../util/alarm.h"
#include "../../modules/sensors.h"

#define DEBUG_MODULE "pressure"
#include "../../util/debug.h"

#define PRESSURE_SENSOR_PIN 1

// Pressure sensor analog voltage reading limit specs
#define PRESSURE_SENSOR_MPXV7025_MIN_VOUT           116     // mV
#define PRESSURE_SENSOR_MPXV7025_MAX_VOUT           4890

#define PRESSURE_SENSOR_MPXV7007_MIN_VOUT           330        
#define PRESSURE_SENSOR_MPXV7007_MAX_VOUT           4700

#define PRESSURE_SENSOR_SSCDRRN100MDAA5_MIN_VOUT    125     // Assumes 5V Vsupply (2.5%)
#define PRESSURE_SENSOR_SSCDRRN100MDAA5_MAX_VOUT    4875    // Assumes 5V Vsupply (97.5%)

#define MILLIVOLTS_TO_ADC * 1023L / 5000L

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
  // Alarm for faulty pressure sensor
  if (reading > PRESSURE_SENSOR_MPXV7025_MAX_VOUT MILLIVOLTS_TO_ADC) {
    DEBUG_PRINT_EVERY(100, "Faulty pressure sensor (HIGH)! Measured: %i , Limit: %i", (int)reading, (int)(PRESSURE_SENSOR_MPXV7025_MAX_VOUT MILLIVOLTS_TO_ADC));
    alarmSet(&sensors.badPressureSensorAlarm);
  }
  else if (reading < PRESSURE_SENSOR_MPXV7025_MIN_VOUT MILLIVOLTS_TO_ADC) {
    DEBUG_PRINT_EVERY(100, "Faulty pressure sensor (LOW)! Measured: %i , Limit: %i", (int)reading, (int)(PRESSURE_SENSOR_MPXV7025_MIN_VOUT MILLIVOLTS_TO_ADC));
    alarmSet(&sensors.badPressureSensorAlarm);
  }

  // For the MPXV7025 pressure sensor: (reading / 1024 - 0.5) / 0.018 = P in kPa
  // From Calvin, why these exact values and the /4, Im not sure
  int32_t pascals = (((int32_t)reading * 217L) - 110995L)>>2; // convert to Pascals
#elif defined(PRESSURE_SENSOR_MPXV7007)

  // Alarm for faulty pressure sensor
  if (reading > PRESSURE_SENSOR_MPXV7007_MAX_VOUT MILLIVOLTS_TO_ADC) {
    alarmSet(&sensors.badPressureSensorAlarm);
  }
  else if (reading < PRESSURE_SENSOR_MPXV7007_MIN_VOUT MILLIVOLTS_TO_ADC) {
    alarmSet(&sensors.badPressureSensorAlarm);
  }

  // For the MPXV7007 pressure sensor: (reading / 1024 - 0.5) / 0.057 = P in kPa
  //                                   (reading / 1024 - 0.5) / (57 / 1000) * 1000 in Pa
  //                                   (reading / 1024 - 0.5) * 1000000 / 57
  //                                   (reading * 1000000 / 57 / 1024 - 500000 / 57)
  //                                   (reading * 17.1327 - 8771.9298) * 8 / 8  ; Help make multiplier error smaller
  //                                   (reading * 137.0614 - 70175.4386) / 8
  int32_t pascals = (((int32_t)reading * 137L) - 70175L)>>3; // convert to Pascals
#elif defined(PRESSURE_SENSOR_SSCDRRN100MDAA5)

  // Alarm for faulty pressure sensor
  if (reading > PRESSURE_SENSOR_SSCDRRN100MDAA5_MAX_VOUT MILLIVOLTS_TO_ADC) {
    alarmSet(&sensors.badPressureSensorAlarm);
  }
  else if (reading < PRESSURE_SENSOR_SSCDRRN100MDAA5_MIN_VOUT MILLIVOLTS_TO_ADC) {
    alarmSet(&sensors.badPressureSensorAlarm);
  }

  // For the SSCDRRN100MDAA5 pressure sensor: (reading - 0.1 * 1024) * (200 / (0.8 * 1024)) + (-100) = P in mbar
  //                                          ((reading * 200 - 1024 * 20) / 0.8 / 1024 - 100) * 100 in Pa
  //                                          (reading * 200 - 20480) * 125 / 1024 - 10000
  //                                          (reading * 25000 - 2560000) / 1024 - 10240000 / 1024
  //                                          (reading * 25000 - 12800000) / 1024
  int32_t pascals = (((int32_t)reading * 25000L) - 12800000L)>>10;
#endif
  int32_t u100umH2O = (pascals * 4177L)>>12;                  // convert Pascals to 0.1mmH2O
  *value = (int16_t)u100umH2O;                                // return as 16 bit signed int
  return HAL_OK;
}