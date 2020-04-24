//
// Control Module
//

#include "../pt/pt.h"

#include "../hal/motor.h"
#include "../hal/timer.h"

#include "Arduino.h"

#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/parameters.h"

#define DEBUG
#define DEBUG_MODULE "control"
#include "../util/debug.h"

// Public Variables
struct control control = {
  .state = CONTROL_IDLE
};

// Private Variables
static struct pt controlThread;
static struct timer controlTimer;

static int8_t motorCompressionDistance;

// *****************
// Control variables
// *****************
// Position uint8_t
// Velocity millirev/min
// Velocity max = 20 rpm
static unsigned int currentPosition = 0;
static unsigned int targetPosition = 0;
static unsigned int targetVelocity = 0;
static unsigned int tolerancePosition = 0;
static unsigned int trapezoidRatio = 2;             // TEMPORARILY SET
static uint32_t rampTime = 0;
static uint32_t platTime = 0;
static uint32_t elapsedTime = 0;
static uint32_t totalBreathTime = 0;
static uint32_t targetInhalationTime = 0;
static float velocityScale = 0.0;
static float nomVelocity = 0.0;


// Pre-compute the trajectory based on known set points and timing requirements
static void computeTrajectory()
{
  // Debug
  DEBUG_PRINT("COMPUTING TRAJECTORY");

  // Update known measurements (position, volume estimation, etc.)

  // Calculate time sections of trajectory (CURRENTLY SAME FOR I/E PHASES DUE TO SENSOR DRIVER HOLD)
  if (control.state == CONTROL_BEGIN_EXHALATION) {

    // Position information needs to be integrated to allow for adaptive positioning on exhalation
    targetPosition = 5;
    rampTime = (totalBreathTime - targetInhalationTime) / (2 + trapezoidRatio);
    platTime = trapezoidRatio * rampTime;
    nomVelocity = abs((float)targetPosition - (float)currentPosition) * 1000.0 * 1000000.0 * 60.0 / ((float)(rampTime + platTime) * 360.0);

  } else if (control.state == CONTROL_BEGIN_INHALATION) {

    // EVENTUALLY REFACTORING FOR VOLUME TRAJECTORY, temporarily position tracking for testing
    targetPosition = 40;
    rampTime = targetInhalationTime / (2 + trapezoidRatio);
    platTime = trapezoidRatio * rampTime;
    nomVelocity = abs((float)targetPosition - (float)currentPosition) * 1000.0 * 1000000.0 * 60.0 / ((float)(rampTime + platTime) * 360.0);

  } else {
    // Blank
  }
}

// Track timing and scale output based on location in trajectory
static void updateTrajectory()
{

  // Determine elapsed time and place along trajectory
  elapsedTime = (((uint32_t)micros()) - controlTimer.start);

  if (elapsedTime < rampTime) {
    // Ramp up
    velocityScale = ((float)elapsedTime / (float)rampTime);
    targetVelocity = (unsigned int)(nomVelocity * velocityScale);

  } else if ((elapsedTime > rampTime) && (elapsedTime < (platTime + rampTime))) {
    // Plateau
    velocityScale = 1.0;
    targetVelocity = (unsigned int)(nomVelocity * velocityScale);

  } else {
    // Ramp down
    velocityScale = (float)(2 * rampTime + platTime - elapsedTime) / (float)rampTime;
    targetVelocity = (unsigned int)(nomVelocity * velocityScale);

  }

  // Debug Output Section
  DEBUG_PRINT_EVERY(100, "Target Position: %lu ; Nominal Velocity: %lu ; Target Velocity: %lu", 
    (uint32_t)targetPosition, (uint32_t)nomVelocity, (uint32_t)targetVelocity);
  // DEBUG_PRINT_EVERY(1000, "Elapsed Time: %lu ; Ramp Time: %lu ; Velocity Scale: %lu", elapsedTime, rampTime, (uint32_t)(1000.0 * velocityScale));
  // DEBUG_PRINT_EVERY(1000, "Proportion of Phase: %lu", velocityScale);

}

// Update control commands and handle ISR/control flags
static int updateControl()
{

  // Update trajectory to get instantaneous target air flow
  updateTrajectory();

  // ADD FUNCTIONALITY FOR ISR FLAG
  if ((timerHalRun(&controlTimer)) == HAL_TIMEOUT ) {
    return 1;
  }

  // **************************************************************
  // Here is where closed loop control structure will be developed
  // **************************************************************
  // Depending on whether we are in inhalation or exhalation, we will
  // switch between closed loop air flow tracking and closed loop position
  // tracking. The updateControl() function will accept a flag set by an ISR.

  // Send motor commands
  motorHalCommand(targetPosition, targetVelocity);

  // Return unfinished
  return 0;

}

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
      
      // Manually set per current settings above
      totalBreathTime = (60 SEC) / 15;
      targetInhalationTime = totalBreathTime / 3;

      timerHalBegin(&controlTimer, targetInhalationTime);

      // Compute trajectory
      computeTrajectory();

      control.state = CONTROL_INHALATION;
      
    } else if (control.state == CONTROL_INHALATION) {
      DEBUG_PRINT("state: CONTROL_INHALATION");

      // Update control loop
      if (true) {
        PT_WAIT_UNTIL(pt, updateControl() != 0);
      } else {
        //
      }

      // TEMPORARY UPDATE POSITION
      currentPosition = targetPosition;

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

      timerHalBegin(&controlTimer, (totalBreathTime - targetInhalationTime));

      // Compute trajectory
      computeTrajectory();
      
      control.state = CONTROL_EXHALATION;
      
    } else if (control.state == CONTROL_EXHALATION) {
      DEBUG_PRINT("state: CONTROL_EXHALATION");

      if (true) {
        PT_WAIT_UNTIL(pt, updateControl() != 0);
      } else {
        //
      }

      // TEMPORARY UPDATE POSITION
      currentPosition = targetPosition;
      
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
