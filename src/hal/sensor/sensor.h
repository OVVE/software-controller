
#ifndef __SENSOR_HAL_H__
#define __SENSOR_HAL_H__

#include <stdint.h>
#include "../../hal/hal.h"

// Initializes the pressure and airflow sensors
int sensorHalInit(void);

// Gets the airflow sensor value in 0.01 SLM
int airflowSensorHalGetValue(int16_t *value);

// Gets the pressor sensor value (measured in units of 0.1mmH2O)
int pressureSensorHalGetValue(int16_t *value);

// Gets the volume value in ml
int airVolumeSensorHalGetValue(int16_t *value);

// Resets volume integrator
int airVolumeSensorHalReset(void);


#endif /* __SENSOR_HAL_H__ */