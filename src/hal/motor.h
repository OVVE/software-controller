#ifndef __MOTOR_HAL_H__
#define __MOTOR_HAL_H__

#include "../hal/hal.h"

#include <stdint.h>

#define MOTOR_DIR_STOP_NOW 3
#define MOTOR_DIR_CLOSE 2
#define MOTOR_DIR_OPEN 1
#define MOTOR_DIR_STOP 0

#define MOTOR_STATUS_MOVING 0
#define MOTOR_STATUS_SWITCH_TRIPPED_BOTTOM 1
#define MOTOR_STATUS_SWITCH_TRIPPED_TOP 2

#define MOTOR_INIT_POSITION 2000 //in mDeg

// TODO: Doc
int8_t motorHalInit(void);

// TODO: Doc
int8_t motorHalCommand(uint8_t dir, uint16_t speed);

//return last position in mdeg 
int32_t motorHalGetPosition(void);

int8_t motorHalGetStatus(void);

struct motor{
  // Precomputed value so that we can minimize the amount of math operations
  // inside of an ISR.
  bool     counter_update;
  uint16_t counter_TOP;

  int32_t  step_position;
  int8_t   microstep_position;
  uint8_t  direction;
  int32_t speedSet; // +/- -> direction
  int32_t speedRequested; // +/-> direction
} ;
extern struct motor motor_control;
#endif /* __MOTOR_HAL_H__ */

