
#ifndef __I2C_HAL_H__
#define __I2C_HAL_H__

#include <stdint.h>

#include "../hal/hal.h"

// TODO: Doc
int i2cHalInit(void);

// TODO: Doc
int i2cHalWrite(uint8_t deviceAddr, uint8_t* data, uint8_t size);

// TODO: Doc
int i2cHalRead(uint8_t deviceAddr, uint8_t* data, uint8_t size);

#endif /* __I2C_HAL__ */