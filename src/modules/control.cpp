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
struct control control = {
  .state = CONTROL_IDLE
};

// Private Variables
static struct pt controlThread;
static struct timer controlTimer;

static PT_THREAD(controlThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  while (1) {
    if (control.state == CONTROL_IDLE) {
      // Wait for the parameters to enter the run state before
      if (parameters.run) {
        control.state = CONTROL_BEGIN_INHALATION;
      }
      
    } else if (control.state == CONTROL_BEGIN_INHALATION) {
      if (!parameters.assist) {
        // TODO: Calculate motor
        unsigned int motorCompressionDistance = 0;
        unsigned int motorCompressionDuration = 0;
        // TODO: Error check
        motorHalBegin(MOTOR_HAL_DIRECTION_INHALATION, motorCompressionDistance, motorCompressionDuration);
      } else {
        // TODO: Implement Assist mode setup
      }
      control.state = CONTROL_INHALATION;
      
    } else if (control.state == CONTROL_INHALATION) {
      if (!parameters.assist) {
        PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS);
      } else {
        // TODO: Implement Assist mode run
      }
      control.state = CONTROL_BEGIN_HOLD_IN;
      
    } else if (control.state == CONTROL_BEGIN_HOLD_IN) {
      timerHalBegin(&controlTimer, 150 MSEC);
      control.state = CONTROL_HOLD_IN;
      
    } else if (control.state == CONTROL_HOLD_IN) {
      PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
      control.state = CONTROL_BEGIN_EXHALATION;
      
    } else if (control.state == CONTROL_BEGIN_EXHALATION) {
      unsigned int motorDecompressionDistance = 0;
      unsigned int motorDecompressionDuration = 0;
      motorHalBegin(MOTOR_HAL_DIRECTION_EXHALATION, motorDecompressionDistance, motorDecompressionDuration);
      control.state = CONTROL_EXHALATION;
      
    } else if (control.state == CONTROL_EXHALATION) {
      PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS);
      control.state = (parameters.run) ? CONTROL_BEGIN_INHALATION : CONTROL_IDLE;
      
    } else {
      // TODO: Error, unknown control state!!!
      control.state = CONTROL_IDLE;
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