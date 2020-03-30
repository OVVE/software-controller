
#ifndef __PRESSURE_SENSOR_HAL_H__
#define __PRESSURE_SENSOR_HAL_H__

#include "../../hal/hal.h"

// TODO: Doc
int pressureSensorHalInit(void);

// TODO: Doc
int pressureSensorHalFetch(void);

// TODO: Doc
int pressureSensorHalGetValue(int* value);

#endif /* __PRESSURE_SENSOR_HAL_H__ */