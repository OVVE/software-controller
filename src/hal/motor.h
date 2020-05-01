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

extern volatile int16_t motor_position;

// TODO: Temporary
#define ABS(N) ((N<0)?(-N):(N))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// TODO: Doc
int8_t motorHalInit(void);

// TODO: Doc
int8_t motorHalCommand(int8_t position, uint16_t speed);

// TODO: Doc
int8_t motorHalStatus(void);

#endif /* __MOTOR_HAL_H__ */

