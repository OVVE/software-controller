//
// Control Module
//

#include "../pt/pt.h"

#include "../hal/motor.h"
#include "../hal/timer.h"

#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/parameters.h"

// Public Variables
int controlState = CONTROL_IDLE;

// Private Variables
static struct pt controlThread;
static struct timer controlTimer;

static PT_THREAD(controlThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  while (1) {
    if (controlState == CONTROL_IDLE) {
      // Wait for the parameters to enter the run state before
      if (parameters.run) {
        controlState = CONTROL_BEGIN_INHALATION;
      }
      
    } else if (controlState == CONTROL_BEGIN_INHALATION) {
      if (!parameters.assist) {
        // TODO: Calculate motor
        unsigned int motorCompressionDistance = 0;
        unsigned int motorCompressionDuration = 0;
        // TODO: Error check
        motorHalBegin(MOTOR_HAL_DIRECTION_INHALATION, motorCompressionDistance, motorCompressionDuration);
      } else {
        // TODO: Implement Assist mode setup
      }
      controlState = CONTROL_INHALATION;
      
    } else if (controlState == CONTROL_INHALATION) {
      if (!parameters.assist) {
        PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS);
      } else {
        // TODO: Implement Assist mode run
      }
      controlState = CONTROL_BEGIN_HOLD_IN;
      
    } else if (controlState == CONTROL_BEGIN_HOLD_IN) {
      timerHalBegin(&controlTimer, 150 MSEC);
      controlState = CONTROL_HOLD_IN;
      
    } else if (controlState == CONTROL_HOLD_IN) {
      PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
      controlState = CONTROL_BEGIN_EXHALATION;
      
    } else if (controlState == CONTROL_BEGIN_EXHALATION) {
      unsigned int motorDecompressionDistance = 0;
      unsigned int motorDecompressionDuration = 0;
      motorHalBegin(MOTOR_HAL_DIRECTION_EXHALATION, motorDecompressionDistance, motorDecompressionDuration);
      controlState = CONTROL_EXHALATION;
      
    } else if (controlState == CONTROL_EXHALATION) {
      PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS);
      controlState = (parameters.run) ? CONTROL_BEGIN_INHALATION : CONTROL_IDLE;
      
    } else {
      // TODO: Error, unknown control state!!!
      controlState = CONTROL_IDLE;
    }
  }
  
  // Should never reach here
  PT_END(pt);
}

int controlModuleInit(void)
{
  if (motorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  
  PT_INIT(&controlThread);
  
  return MODULE_OK;
}

int controlModuleRun(void)
{
  return (PT_SCHEDULE(controlThreadMain(&controlThread))) ? MODULE_OK : MODULE_FAIL;
}