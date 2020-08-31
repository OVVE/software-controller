/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
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

