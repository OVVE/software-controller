//
// Control Module
//

#include "../pt/pt.h"

#include "../hal/motor.h"
#include "../hal/timer.h"

#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/parameters.h"

// #define DEBUG
#define DEBUG_MODULE "control"
#include "../util/debug.h"

// Public Variables
struct control control = {
  .state = CONTROL_IDLE
};

// Private Variables
static struct pt controlThread;
static struct timer controlTimer;

static unsigned int motorCompressionDistance;
static unsigned int motorCompressionDuration;

static PT_THREAD(controlThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  // Current Settings
  // 15 BPM
  // I:E 1:2
  // TV - (dependent upon lung compliance and PEEP setting)

  while (true) {
    if (control.state == CONTROL_IDLE) {
      DEBUG_PRINT_EVERY(10000, "state: CONTROL_IDLE");

      // Wait for the parameters to enter the run state before
      if (parameters.startVentilation) {
        control.state = CONTROL_BEGIN_INHALATION;
      }
      
    } else if (control.state == CONTROL_BEGIN_INHALATION) {
      DEBUG_PRINT("state: CONTROL_BEGIN_INHALATION");

      if (parameters.ventilationMode == VENTILATOR_MODE_VC) {
        // TODO: Calculate breath parameters and motor control.
        motorCompressionDistance = 40; // Fixed to 40 degrees excursion for now.
        motorCompressionDuration = 0;
        // TODO: Error check
        motorHalBegin(MOTOR_HAL_DIRECTION_INHALATION, motorCompressionDistance, motorCompressionDuration);
      } else {
        // TODO: Implement Assist mode setup
      }
      control.state = CONTROL_INHALATION;
      
    } else if (control.state == CONTROL_INHALATION) {
      DEBUG_PRINT("state: CONTROL_INHALATION");

      if (parameters.ventilationMode == VENTILATOR_MODE_VC) {
        PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS);
      } else {
        // TODO: Implement Assist mode run
      }
      control.state = CONTROL_BEGIN_HOLD_IN;
      
    } else if (control.state == CONTROL_BEGIN_HOLD_IN) {
      DEBUG_PRINT("state: CONTROL_BEGIN_HOLD_IN");

      timerHalBegin(&controlTimer, 500 MSEC);
      control.state = CONTROL_HOLD_IN;
      
    } else if (control.state == CONTROL_HOLD_IN) {
      DEBUG_PRINT("state: CONTROL_HOLD_IN");

      PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
      control.state = CONTROL_BEGIN_EXHALATION;
      
    } else if (control.state == CONTROL_BEGIN_EXHALATION) {
      DEBUG_PRINT("state: CONTROL_BEGIN_EXHALATION");

      // Since exhalation is not dependent on the bag, allow the bag to decompress with the same parameters as compression
      // In order to time exhalation, set a timer to time the exhalation cycle
      // TODO: consider if patient takes breathe before motor has completely moved back
      motorHalBegin(MOTOR_HAL_DIRECTION_EXHALATION, motorCompressionDistance, motorCompressionDuration);
      timerHalBegin(&controlTimer, 2666 MSEC); // TODO: calculate this value from the breath parameters.
      control.state = CONTROL_EXHALATION;
      
    } else if (control.state == CONTROL_EXHALATION) {
      DEBUG_PRINT("state: CONTROL_EXHALATION");

      // TODO: Fix this condition for assisted modes
      PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS && timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
      control.state = (parameters.startVentilation) ? CONTROL_BEGIN_INHALATION : CONTROL_IDLE;
      
    } else {
      DEBUG_PRINT("state: (unknown)");

      // TODO: Error, unknown control state!!!
      control.state = CONTROL_IDLE;
    }
    
    PT_YIELD(pt);
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
