
#ifndef __AIRFLOW_SENSOR_HAL_H__
#define __AIRFLOW_SENSOR_HAL_H__

#include <stdint.h>

#include "../../hal/hal.h"

// TODO: Doc
int airflowSensorHalInit(void);

// TODO: Doc
int airflowSensorHalFetch(void);

// TODO: Doc
int airflowSensorHalGetValue(int16_t* value);

#endif /* __AIRFLOW_SENSOR_HAL_H__ */