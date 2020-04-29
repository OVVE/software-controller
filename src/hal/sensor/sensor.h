
#ifndef __SENSOR_HAL_H__
#define __SENSOR_HAL_H__

#include <stdint.h>
#include <Arduino.h>
#include "../../hal/hal.h"

// Sensor pin defines
#define FLOW_SENSE_PIN     0
#define PRESSURE_SENSE_PIN 1

// Initializes the pressure and airflow sensors
int sensorHalInit(void);

// Gets the airflow sensor value in 0.01 SLM
int airflowSensorHalGetFlow(int16_t adcRawData, int16_t *value);

// Gets the pressor sensor value (measured in units of 0.1mmH2O)
int pressureSensorHalGetValue(int16_t adcRawData, int16_t *value);

// Gets the volume value in ml
int airflowSensorHalGetVolume(int16_t *value);

// Resets volume integrator
int airflowSensorHalResetVolume(void);

// This routine updated the integrated volume value
int airflowSensorHalUpdateVolume(int16_t flowValue);

//Get current volume
int32_t airflowSensorHalGetVolume(void);

//calibrates bias (blocking for ~ 16ms, only call at startup!)
int airflowSensorCalibrateBias(void);

//returns current airflow sensor bias
int16_t airflowSensorGetBias(void);

//low level ADC routines

//starts an ADC reading
int sensorAdcStartAnalogRead(int pin);

//checks if ADC data is available. Needs to return HAL_OK before data can be read with sensorAdcCheckGetData()
int sensorAdcCheckRead(void);

//returns raw ADC data if available
int sensorAdcGetData(int16_t* data);

#endif /* __SENSOR_HAL_H__ */
