//
// Control Module
//

#include <stdint.h>

#include "../pt/pt.h"

#include "../hal/motor.h"
#include "../hal/timer.h"

#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/sensors.h"
#include "../modules/parameters.h"

// #define DEBUG
#define DEBUG_MODULE "control"
#include "../util/debug.h"

#define INHALATION_OVERTIME 4 / 3
#define HOLD_TIME           (200 MSEC)

// Public Variables
struct control control = {
  .state = CONTROL_IDLE
};

// Private Variables
static struct pt controlThread;
static struct timer breathTimer;
static struct timer controlTimer;

static uint16_t totalMotorCompressionDistance;
static uint16_t totalMotorCompressionDuration;

static uint16_t targetVolume;
static uint32_t totalBreathTime;
static uint32_t targetInhalationTime;

static uint32_t measuredInhalationTime;
static uint32_t measuredExhalationTime;

// Mid-level volume control function
static void updateVolumeControl(uint16_t* distance, uint16_t* duration)
{
  // TODO: fill-in this function; for now just perform a 40 degree motion
  if (totalMotorCompressionDistance == 0) {
    *distance = 40;
    *duration = 0;
  }
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

      // Reset any parameters
      control.breathCount = 0;
      control.ieRatioMeasured = 0;
      control.respirationRateMeasured = 0;
      
      // Wait for the parameters to enter the run state before
      PT_WAIT_UNTIL(pt, parameters.startVentilation);
      
      control.state = CONTROL_BEGIN_INHALATION;
      
    } else if (control.state == CONTROL_BEGIN_INHALATION) {
      DEBUG_PRINT("state: CONTROL_BEGIN_INHALATION");
      
      // Collect all set points from parameters
      totalBreathTime = (60 SEC) / parameters.respirationRateRequested;
      targetInhalationTime = (parameters.ieRatioRequested * totalBreathTime) >> 8;
      targetVolume = parameters.volumeRequested;
      
      // Initialize all 
      totalMotorCompressionDistance = 0;
      totalMotorCompressionDuration = 0;
      measuredInhalationTime = 0;
      measuredExhalationTime = 0;
      
      // Kickoff timers for monitoring breath/stages of breathing
      timerHalBegin(&breathTimer, totalBreathTime);
      timerHalBegin(&controlTimer, targetInhalationTime * INHALATION_OVERTIME);
      
      control.state = CONTROL_INHALATION;
      
    } else if (control.state == CONTROL_INHALATION) {
      DEBUG_PRINT("state: CONTROL_INHALATION");

      uint16_t motorDistance = 0;
      uint16_t motorDuration = 0;
  
      // If in a volume controlled mode, run the Mid-level volume controller
      if ((parameters.ventilationMode == VENTILATOR_MODE_VC) ||
          (parameters.ventilationMode == VENTILATOR_MODE_AC)) {
        
        // Mid-level controller function
        updateVolumeControl(&motorDistance, &motorDuration);
      }
      
      // Update distance/duration accumulators (for exhalation)
      totalMotorCompressionDistance += motorDistance;
      totalMotorCompressionDuration += motorDuration;
      
      // If the controller reports zero distance or the timer has expired, move
      // on to next state, otherwi<<se run the motor as specified by the controller
      if ((timerHalRun(&controlTimer) == HAL_TIMEOUT) || (motorDistance == 0)) {
        measuredInhalationTime = timerHalCurrent(&breathTimer);
        control.state = CONTROL_BEGIN_HOLD_IN;
      } else {
        motorHalBegin(MOTOR_HAL_DIRECTION_INHALATION, motorDistance, motorDuration);
        // TODO: Wait until motor reaches destination? Or try to parallelize
        // parameters re-issue motor control setpoints before completing last ones?
        PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS);
      }
      
    } else if (control.state == CONTROL_BEGIN_HOLD_IN) {
      DEBUG_PRINT("state: CONTROL_BEGIN_HOLD_IN");

      // Setup the hold timer
      timerHalBegin(&controlTimer, HOLD_TIME);

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
      motorHalBegin(MOTOR_HAL_DIRECTION_EXHALATION,
                    totalMotorCompressionDistance,
                    totalMotorCompressionDuration);
      control.state = CONTROL_EXHALATION;
      
    } else if (control.state == CONTROL_EXHALATION) {
      DEBUG_PRINT("state: CONTROL_EXHALATION");
      // Wait for the motor to move back
      // TODO: consider if patient takes breathe before motor has completely moved back?
      // probably cant start the breath until the motor moves back anyway, right?
      PT_WAIT_UNTIL(pt, ((motorHalRun() != HAL_IN_PROGRESS) ||
                         (timerHalRun(&breathTimer) != HAL_IN_PROGRESS)));
      
      // Now wait for the current exhalation time (now the remainder of the breath
      // timer) to expire or for a breath to be detected if in an assisted mode
      PT_WAIT_UNTIL(pt, ((timerHalRun(&breathTimer) != HAL_IN_PROGRESS) ||
                         ((parameters.ventilationMode == VENTILATOR_MODE_AC) &&
                           sensors.inhalationDetected)));

      
      // Calculate and update the measured times
      uint32_t currentBreathTime = timerHalCurrent(&breathTimer);
      measuredExhalationTime = currentBreathTime - HOLD_TIME - measuredInhalationTime;
      control.ieRatioMeasured = (measuredExhalationTime << 8) / measuredInhalationTime;
      control.respirationRateMeasured = (60 SEC) / (currentBreathTime USEC);
      control.breathCount++;

      // Check if we need to continue onto another breath or if ventilation has stopped
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
