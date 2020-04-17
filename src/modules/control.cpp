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

#define INHALATION_OVERTIME 4 / 3
#define HOLD_TIME           (150 MSEC)

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

// Control Variables (to be potentially added to control struct)
static uint16_t currentVolume;
static uint16_t desiredVolume;
static uint16_t volumeToleramce;
static uint16_t airFlow;

static uint16_t volumeControlGain;

// Mid-level volume control function
//static void updateVolumeControl(uint16_t* distance, uint16_t* duration)
static void updateVolumeControl(uint16_t* velocity, uint16_t* duration)
{
	// Handle ventilation mode
	if (parameters.ventilationMode == VENTILATOR_MODE_VC) {
		// Update current volume
		getVolume(&currentVolume);
		// Determine control state
		if (control.state == CONTROL_INHALATION) {
			// Set desired volume to be target volume for inhalation
			desiredVolume = targetVolume;
		}
		else if (control.state == CONTROL_EXHALATION) {
			// Set desired volume to be 0 for exhalation
			desiredVolume = 0;
		}

		// Check if reached desired volume within some tolerance
		if (abs(desiredVolume - currentVolume) < volumeToleramce) {
			// Modulate velocity to move toward desired volume
			velocity = volumeControlGain * (desiredVolume - currentVolume);
		}

	}
	else if (parameters.ventilationMode == VENTILATOR_MODE_AC) {
		// Update current volume
		getVolume(&currentVolume);
		// Determine control state
		if (control.state == CONTROL_INHALATION) {
			// Set desired volume to be target volume for inhalation
			desiredVolume = targetVolume;
		}
		else if (control.state == CONTROL_EXHALATION) {
			// Set desired volume to be 0 for exhalation
			desiredVolume = 0;
		}

		// Check if reached desired volume within some tolerance
		if (abs(desiredVolume - currentVolume) < volumeToleramce) {
			// Modulate velocity to move toward desired volume
			velocity = volumeControlGain * (desiredVolume - currentVolume);
		}
	}
}

// Calculate current volume
static void getVolume(uint16_t* volume)
{
	//
	volume += airFlow * controlTimer;
}

static PT_THREAD(controlThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  while (1) {
    if (control.state == CONTROL_IDLE) {
      // Reset any parameters
      control.breathCount = 0;
      control.ieRatioMeasured = 0;
      control.respirationRateMeasured = 0;
      
      // Wait for the parameters to enter the run state before
      PT_WAIT_UNTIL(pt, parameters.startVentilation);
      
      control.state = CONTROL_BEGIN_INHALATION;
      
    } else if (control.state == CONTROL_BEGIN_INHALATION) {
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
      //uint16_t motorDistance = 0;
			uint16_t motorVelocity = 0;
      uint16_t motorDuration = 0;
  
      // If in a volume controlled mode, run the Mid-level volume controller
      if ((parameters.ventilationMode == VENTILATOR_MODE_VC) ||
          (parameters.ventilationMode == VENTILATOR_MODE_AC)) {
        
        // Mid-level controller function
				//updateVolumeControl(&motorDistance, &motorDuration);
        updateVolumeControl(&motorVelocity, &motorDuration);
      }
      
      // Update distance/duration accumulators (for exhalation)
      totalMotorCompressionDistance += motorDistance;
      totalMotorCompressionDuration += motorDuration;
      
      // If controller reports nonzero distance, need to move motor
        if (motorDistance != 0) {
          
        }
      
      // If the controller reports zero distance or the timer has expired, move
      // on to next state, otherwise run the motor as specified by the controller
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
      // Setup the hold timer
      timerHalBegin(&controlTimer, HOLD_TIME);
      control.state = CONTROL_HOLD_IN;
      
    } else if (control.state == CONTROL_HOLD_IN) {
      PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
      control.state = CONTROL_BEGIN_EXHALATION;
      
    } else if (control.state == CONTROL_BEGIN_EXHALATION) {
      // Since exhalation is not dependent on the bag, allow the bag to decompress with the same parameters as compression
      // In order to time exhalation, set a timer to time the exhalation cycle
      // TODO: Andrew, thoughts?; also, if this is a motor home, maybe need different HAL function for it
			motorHalBegin(MOTOR_HAL_DIRECTION_EXHALATION,
                    totalMotorCompressionDistance,
                    totalMotorCompressionDuration);
      control.state = CONTROL_EXHALATION;
      
    } else if (control.state == CONTROL_EXHALATION) {
      // Wait for the motor to move back
      // TODO: consider if patient takes breathe before motor has completely moved back?
      // probably cant start the breath until the motor moves back anyway, right?
      PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS);
      
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