
#ifndef __WATCHDOG_HAL_H__
#define __WATCHDOG_HAL_H__

#include "../hal/hal.h"

#define WATCHDOG_PERIOD_MS 500

// TODO: Doc
int watchdogHalInit(void);

// TODO: Doc
int watchdogHalReset(void);

#endif /* __WATCHDOG_HAL_H__ */