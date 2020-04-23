#ifndef __MOTOR_HAL_H__
#define __MOTOR_HAL_H__

#include "../hal/hal.h"

#include <stdint.h>

// TODO: Doc
int8_t motorHalInit(void);

// TODO: Doc
int8_t motorHalCommand(uint8_t position, uint16_t speed);

// TODO: Doc
int8_t motorHalStatus(void);

#endif /* __MOTOR_HAL_H__ */

