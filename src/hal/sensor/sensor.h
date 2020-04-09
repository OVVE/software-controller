
#ifndef __SENSOR_HAL_H__
#define __SENSOR_HAL_H__

#include <stdint.h>
#include "../../hal/hal.h"

// Initializes the pressure and airflow sensors
int16_t sensorHalInit(void);

// Gets the airflow sensor value
int16_t airflowSensorHalGetValue(int16_t *value);

// Gets the pressor sensor value (measured in units of 0.1mmH2O
int16_t pressureSensorHalGetValue(int16_t *value);

#endif /* __SENSOR_HAL_H__ */