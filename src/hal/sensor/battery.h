
#ifndef __BATTERY_SENSOR_HAL_H__
#define __BATTERY_SENSOR_HAL_H__

#include <stdint.h>
#include <stdbool.h>

#include "../../hal/hal.h"

// TODO: Doc
int batterySensorHalInit(void);

// TODO: Doc
int batterySensorHalFetch(void);

// TODO: Doc
int batterySensorHalGetValue(uint8_t* value, bool* isCharging);

#endif /* __BATTERY_SENSOR_HAL_H__ */