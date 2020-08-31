
#ifndef __ESTOP_HAL_H__
#define __ESTOP_HAL_H__

#include <stdbool.h>

#include "../hal/hal.h"

// TODO: Doc
int estopHalInit(void);

// TODO: Doc
bool estopHalAsserted(void);

#endif /* __ESTOP_HAL_H__ */