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
#include "../../hal/sensor/airflow.h"
#include "../../util/alarm.h"
#include "../../modules/sensors.h"

#define LOG_MODULE "airflow"
#define LOG_LEVEL  LOG_SENSORS_MODULE
#include "../../util/log.h"

#define AIRFLOW_SENSOR_PIN 0

// Airflow sensor analog voltage reading limit specs
// #define AIRFLOW_SENSOR_PMF4103A_MIN_VOUT        1000        // mV
#define AIRFLOW_SENSOR_PMF4103A_MIN_VOUT        500        // mV
#define AIRFLOW_SENSOR_PMF4103A_MAX_VOUT        5000

#define MILLIVOLTS_TO_ADC * 1023L / 5000L

static int16_t reading;

int airflowSensorHalInit(void)
{
  adcHalInit();

  return HAL_OK;
}

int airflowSensorHalFetch(void)
{
  // If the ADC is free, kick off our conversion; otherwise if its no longer
  // converting our pin, take the value and free the ADC
  if (!adcHalBusy()) {
    adcHalBegin(AIRFLOW_SENSOR_PIN);
  } else if ((adcHalGetCurrentPin() == AIRFLOW_SENSOR_PIN) && !adcHalInProgress()) {
    reading = adcHalGetValue();
    adcHalComplete();
    return HAL_OK;
  }
    
  return HAL_IN_PROGRESS;
}

int airflowSensorHalGetValue(int16_t* value)
{
  // Alarm for faulty airflow sensor
  if (reading > AIRFLOW_SENSOR_PMF4103A_MAX_VOUT MILLIVOLTS_TO_ADC) {
    LOG_PRINT_EVERY(100, INFO, "Faulty airflow sensor! Measured: %i", (int)reading);
    alarmSet(&sensors.badAirflowSensorAlarm);
  }
  else if (reading < AIRFLOW_SENSOR_PMF4103A_MIN_VOUT MILLIVOLTS_TO_ADC) {
    LOG_PRINT_EVERY(100, INFO, "Faulty airflow sensor! Measured: %i", (int)reading);
    alarmSet(&sensors.badAirflowSensorAlarm);
  }

  // reading is ADC values, from PMF4003V data sheet:
  // flow = (Vout - 1[V]) / 4[V] * Range = (Vout - 1[V]) / 4[V] * 20000[0.01SLM]
  // Vout = ADCVal / 2^10 * 5[V] = ADCVal * 5[V] / 1024
  // So:
  // flow = (ADCVal / 1024 * 5[V] - 1[V]) / 4[V] * 20000[0.01SLM]
  // flow = (ADCVal * 5 - 1 * 1024) / 1024 / 4 * 20000
  // flow = (ADCVal * 5 * 20000 - 1024 * 20000) / 4096
  // flow = (ADCVal * 100000 - 20480000) / 4096
  *value = (int16_t) (((((int32_t) reading) * 100000L) - 20480000L) >> 12);
  return HAL_OK;
}