
#include <stdint.h>

#include "../../hal/hal.h"
#include "../../hal/sensor/adc.h"
#include "../../hal/sensor/airflow.h"

#define AIRFLOW_SENSOR_PIN 0

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