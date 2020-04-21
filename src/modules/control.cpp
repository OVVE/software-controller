//
// Control Module
//

#include <stdint.h>

#include "../pt/pt.h"

#include "../hal/motor.h"
#include "../hal/timer.h"
#include "../hal/sensor/sensor.h"

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

// Control variables
static uint16_t currentVolume = 0;
static uint16_t currentAirFlow = 0;
static uint16_t toleranceAirFlow = 0;					// *ENTER VALUE* Allowable deviation from desired air flow
static uint16_t toleranceVolume = 0;					// *ENTER VALUE* Allowable deviation from desired volume
static uint16_t maxAirAcceleration = 0;				// *ENTER VALUE*
static uint16_t nomAirFlow = 0;
static uint16_t targetAirFlow = 0;
static uint16_t desiredAirFlow = 0;
static uint16_t trapezoidRatio = 2 / 1;				// *ENTER VALUE* Ratio of plateau to ramp (softness of trapezoid)
static uint32_t rampTime = 0;
static uint32_t platTime = 0;
static uint32_t elapsedTime = 0;
static uint16_t airFlowControlGain = 0;				// *ENTER VALUE*

// Calculate trajectory parameters once at initiation of inhalation at current volume
static void computeTrajectory()
{
	// Update current volume
	airVolumeSensorHalGetValue(&currentVolume);

	// Calculate time sections
	if (control.state == CONTROL_BEGIN_EXHALATION) {
		rampTime = (totalBreathTime - targetInhalationTime) / (2 + trapezoidRatio);
		platTime = trapezoidRatio * rampTime;
		nomAirFlow = (0 - currentVolume) / (rampTime + platTime);
	}
	else if {
		rampTime = targetInhalationTime / (2 + trapezoidRatio);
		platTime = trapezoidRatio * rampTime;
		nomAirFlow = (targetVolume - currentVolume) / (rampTime + platTime);
	}

	// Check acceleration limits
	if ((nomAirFlow / rampTime) > maxAirAcceleration) {
		// EXCEEDED MAX ACCEL MESSAGE
	}
}

// Update trajectory at every call to control
static void updateTrajectory()
{
	// Determine elapsed time and place along trajectory
	elapsedTime = (((uint32_t)micros()) - controlTimer.start);

	if (elapsedTime < rampTime) {
		// Ramp up
		targetAirFlow = nomAirFlow * (elapsedTime / rampTime);
	}
	else if ((elapsedTime > rampTime) && (elapsedTime < platTime)) {
		// Plateau
		targetAirFlow = nomAirFlow;
	}
	else {
		// Ramp down
		targetAirFlow = nomAirFlow * (elapsedTime + 2*rampTime - platTime);
	}
}

// Update volume control by tracking air flow trajectory
static int updateVolumeControl(uint16_t* velocity)
{
	// Update trajectory to get instantaneous target air flow
	updateTrajectory();

	// Handle ventilation mode
	if (parameters.ventilationMode == VENTILATOR_MODE_VC) {

		// Update current air flow
		airflowSensorHalGetValue(&currentAirFlow);

		// Determine control state
		if (control.state == CONTROL_INHALATION) {
			// Set desired air flow to be target air flow for inhalation trajectory
			desiredAirFlow = targetAirFlow;
		}
		else if (control.state == CONTROL_EXHALATION) {
			// Set desired air flow to be negative for exhalation
			desiredAirFlow = -1 * targetAirFlow;
		}

		// Check if timed out or reached desired volume within some tolerance
		if ((timerHalRun(&controlTimer) == HAL_TIMEOUT) || (abs(targetVolume - currentVolume) < toleranceVolume)) {
			// Reached target volume
			return 1;
		}
		// Check if reached desired air flow within some tolerance
		else if (abs(desiredAirFlow - currentAirFlow) > toleranceAirFlow) {
			// Modulate velocity to move toward desired volume
			velocity += airFlowControlGain * (desiredAirFlow - currentAirFlow);

			// SEND MOTOR COMMANDS HERE

			// Motor still running
			return 0;
		}
		else {
			// Motor still running
			return 0;
		}
	}
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
      
			// Reset motorVelocity
			uint16_t motorVelocity = 0;

			// Compute trajectory
			computeTrajectory();

    } else if (control.state == CONTROL_INHALATION) {
			/*
      // If in a volume controlled mode, run the Mid-level volume controller
      if ((parameters.ventilationMode == VENTILATOR_MODE_VC) ||
          (parameters.ventilationMode == VENTILATOR_MODE_AC)) {
        
        // Mid-level controller function
				//updateVolumeControl(&motorDistance, &motorDuration);
      }
			*/
      
			// ***** HERE NEEDS INTERFACING WITH LOW-LEVEL CONTROL FOR VELOCITY COMMAND

      // If the controller reports reached volume or the timer has expired, move
      // on to next state, otherwise run the motor as specified by the controller
      if ((timerHalRun(&controlTimer) == HAL_TIMEOUT) || (abs(targetVolume - currentVolume) < toleranceVolume)) {
        measuredInhalationTime = timerHalCurrent(&breathTimer);
        control.state = CONTROL_BEGIN_HOLD_IN;
      } else {
        motorHalBegin(MOTOR_HAL_DIRECTION_INHALATION, motorDistance, motorDuration);
        // TODO: Wait until motor reaches destination? Or try to parallelize
        // parameters re-issue motor control setpoints before completing last ones?
        //PT_WAIT_UNTIL(pt, motorHalRun() != HAL_IN_PROGRESS);

				if ((parameters.ventilationMode == VENTILATOR_MODE_VC) ||
					(parameters.ventilationMode == VENTILATOR_MODE_AC)) {

					PT_WAIT_UNTIL(pt, updateVolumeControl(&motorVelocity) != 0);
				}
      }
      
    } else if (control.state == CONTROL_BEGIN_HOLD_IN) {

			// Reset motorVelocity
			uint16_t motorVelocity = 0;

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
      
			// Compute trajectory
			computeTrajectory();

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

			/*
			// If in a volume controlled mode, run the Mid-level volume controller
			if ((parameters.ventilationMode == VENTILATOR_MODE_VC) ||
				(parameters.ventilationMode == VENTILATOR_MODE_AC)) {

				// Mid-level controller function
				//updateVolumeControl(&motorDistance, &motorDuration);
				updateVolumeControl(&motorVelocity);
			}
			*/

			// ***** HERE NEEDS INTERFACING WITH LOW-LEVEL CONTROL FOR VELOCITY COMMAND

			// If the controller reports reached volume or the timer has expired, move
			// on to next state, otherwise run the motor as specified by the controller
			if ((timerHalRun(&controlTimer) == HAL_TIMEOUT) || (abs(targetVolume - currentVolume) < toleranceVolume)) {
				measuredInhalationTime = timerHalCurrent(&breathTimer);
				// Check if we need to continue onto another breath or if ventilation has stopped
				control.state = (parameters.startVentilation) ? CONTROL_BEGIN_INHALATION : CONTROL_IDLE;
			}
			else {
				motorHalBegin(MOTOR_HAL_DIRECTION_INHALATION, motorDistance, motorDuration);
				// TODO: Wait until motor reaches destination? Or try to parallelize
				// parameters re-issue motor control setpoints before completing last ones?
				
				if ((parameters.ventilationMode == VENTILATOR_MODE_VC) ||
					(parameters.ventilationMode == VENTILATOR_MODE_AC)) {

					PT_WAIT_UNTIL(pt, updateVolumeControl(&motorVelocity) != 0);
				}
			}
      
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