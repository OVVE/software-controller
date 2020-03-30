
#ifndef __MOTOR_HAL_H__
#define __MOTOR_HAL_H__

#include "../hal/hal.h"

#define MOTOR_HAL_DIRECTION_INHALATION 0x55
#define MOTOR_HAL_DIRECTION_EXHALATION 0xAF

// TODO: Doc
int motorHalInit(void);

// TODO: Doc
int motorHalBegin(unsigned int direction, unsigned int distance, unsigned int duration);

// TODO: Doc
int motorHalRun(void);

#endif /* __MOTOR_HAL_H__ */