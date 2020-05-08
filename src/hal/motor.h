#ifndef __MOTOR_HAL_H__
#define __MOTOR_HAL_H__

#include "../hal/hal.h"

#include <stdint.h>

// General Definitions
#define DEGREES_PER_REVOLUTION 360
#define SECONDS_PER_MINUTE 60

// Unit Conventions
#define MOTOR_HAL_DEGREE_MULTIPLIER 1000
#define MOTOR_HAL_RPM_MULTIPLIER    1000

#define MOTOR_HAL_COMMAND_OFF   0
#define MOTOR_HAL_COMMAND_HOLD  1
#define MOTOR_HAL_COMMAND_OPEN  2        
#define MOTOR_HAL_COMMAND_CLOSE 3

#define MOTOR_HAL_SPEED_MAX (28*MOTOR_HAL_RPM_MULTIPLIER)

#define MOTOR_HAL_STATUS_ERROR       -1
#define MOTOR_HAL_STATUS_STOPPED      0
#define MOTOR_HAL_STATUS_MOVING       1
#define MOTOR_HAL_STATUS_LIMIT_BOTTOM 2
#define MOTOR_HAL_STATUS_LIMIT_TOP    3

// TODO: This probably should not be located in this file, as all position
// control should be performed outside the motor HAL.
#define MOTOR_HAL_INIT_POSITION (2*MOTOR_HAL_DEGREE_MULTIPLIER)

// TODO: Documentation
int8_t motorHalInit(void);

// TODO: Documentation
// Motor speed in RPM*MOTOR_HAL_RPM_MULTIPLIER.
int8_t motorHalCommand(uint8_t command, uint16_t speed);

// TODO: Documentation
// Motor position in degrees*MOTOR_HAL_DEGREE_MULTIPLIER.
int32_t motorHalGetPosition(void);

// TODO: Documentation
int8_t motorHalGetStatus(void);

#endif /* __MOTOR_HAL_H__ */

