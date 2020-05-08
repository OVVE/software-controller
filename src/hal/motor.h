#ifndef __MOTOR_HAL_H__
#define __MOTOR_HAL_H__

#include "../hal/hal.h"

#include <stdint.h>

// Motor Selection
#define MOTOR_NANOTEC__ST6018D4508__GP56_T2_26_HR
// #define MOTOR_STEPPERONLINE__23HS30_2804S_HG10
// #define MOTOR_STEPPERONLINE__23HS22_2804S_HG15
// #define MOTOR_STEPPERONLINE__23HS22_2804S_HG20
// #define MOTOR_STEPPERONLINE__23HS22_2804S_HG50

// Motor Controller Selection
#define MOTOR_CONTROLLER_NANOTEC__CL4_E_2_12_5VDI
// #define MOTOR_CONTROLLER_STEPPERONLINE_ISD08
// #define MOTOR_CONTROLLER_STEPPERONLINE_DM332T

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

