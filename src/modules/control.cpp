//
// Control Module
//

#include "../pt/pt.h"

#include "../hal/motor.h"

#include "../modules/module.h"
#include "../modules/control.h"

// Public Variables
int controlState = CONTROL_IDLE;

// Private Variables
static struct pt controlThread;

static PT_THREAD(controlThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  while (1) {
    if (controlState == CONTROL_IDLE) {
      // TODO: Logic to move out of idle
    } else if (controlState == CONTROL_BEGIN_INHALATION) {
      // TODO: Calculate motor
      unsigned int motorCompressionDistance = 0;
      unsigned int motorCompressionDuration = 0;
      motorHalBegin(MOTOR_HAL_DIRECTION_INHALATION, motorCompressionDistance, motorCompressionDuration);
      controlState = CONTROL_INHALATION;
    } else if (controlState == CONTROL_INHALATION) {
      PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS);
      controlState = CONTROL_BEGIN_HOLD_IN;
    } else if (controlState == CONTROL_BEGIN_HOLD_IN) {
      // TODO: Implement begin hold state
    }
    // TODO: Implement rest of control FSM
  }
  
  // Should never reach here
  PT_END(pt);
}

int controlModuleInit(void)
{
  if (motorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  return MODULE_OK;
}

int controlModuleRun(void)
{
  return (PT_SCHEDULE(controlThreadMain(&controlThread))) ? MODULE_OK : MODULE_FAIL;
}