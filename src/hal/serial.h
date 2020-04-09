
#ifndef __SERIAL_HAL_H__
#define __SERIAL_HAL_H__

#define SERIAL_DEBUG

#include "../hal/hal.h"

int serialHalInit(void);
int serialHalGetData(void);
int serialHalSendData();

#endif /* __SERIAL_HAL_H__ */