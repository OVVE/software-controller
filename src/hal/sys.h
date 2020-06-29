
#ifndef __SYS_HAL_H__
#define __SYS_HAL_H__

#include <stdint.h>
#include <stdbool.h>

#include "../hal/hal.h"

#define POWERBTN_PIN         65
#define POWEROFF_PIN         12

#define POWERBTN_ASSERT_OFF   0
#define POWERBTN_ASSERT_ON    1

#define POWEROFF_ASSERT       0
#define POWEROFF_DEASSERT     1

// TODO: Doc
int sysHalInit(void);

// TODO: Doc
uint32_t sysHalTime(void);

// TODO: Doc
void sysHalPowerOff(void);

// TODO: Doc
bool sysHalPowerButtonAsserted(void);

#endif /* __SYS_HAL_H__ */