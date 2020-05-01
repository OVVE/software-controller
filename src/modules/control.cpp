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

#include "../util/metrics.h"

#define DEBUG
#define DEBUG_MODULE "control"
#include "../util/debug.h"

#define CONTROL_LOOP_PERIOD (10 MSEC)

#define HOLD_TIME           (200 MSEC)

#define INHALATION_OVERTIME(t) ((t) * 4 / 3)

#define MIN_VELOCITY  2 // TODO: Fix this


// Public Variables
struct control control = {
  .state = CONTROL_IDLE
};

// Private Variables
static struct pt controlThread;
static struct timer breathTimer;
static struct timer controlTimer;

static uint16_t targetVolume;
static uint32_t totalBreathTime;
static uint32_t targetInhalationTime;

static uint32_t measuredInhalationTime;
static uint32_t measuredExhalationTime;

static bool controlComplete;

// *****************
// Control variables
// *****************
// Position uint8_t
// Velocity millirev/min
// Velocity max = 20 rpm
static unsigned int currentPosition = 0;
static unsigned int targetPosition = 0;
static float targetAirFlow = 0.0f;
static unsigned int tolerancePosition = 0;
static unsigned int trapezoidRatio = 2;             // TEMPORARILY SET
static uint32_t rampTime = 0;
static uint32_t platTime = 0;
static uint32_t elapsedTime = 0;
static float velocityScale = 0.0f;
static float nomVelocity = 0.0f;
static uint32_t breathTimerStateStart = 0;

static struct metrics midControlTiming;
static float controlI=0.0f;
static float controlOutputFiltered=0.0f;


//this is our initial guess scaling factor to match the trajectory to flow sensor values
#define SCALING_FACTOR 1.89f

// Pre-compute the trajectory based on known set points and timing requirements
static void computeTrajectory()
{
  // Debug
  DEBUG_PRINT("COMPUTING TRAJECTORY");

  // Update known measurements (position, volume estimation, etc.)

  // Calculate time sections of trajectory (CURRENTLY SAME FOR I/E PHASES DUE TO SENSOR DRIVER HOLD)
  if (control.state == CONTROL_BEGIN_EXHALATION) {

    // Position information needs to be integrated to allow for adaptive positioning on exhalation
    targetPosition = 20;
    rampTime = (totalBreathTime - targetInhalationTime) / (2.0f + trapezoidRatio);
    platTime = trapezoidRatio * rampTime;
    nomVelocity = abs((float)targetPosition - (float)currentPosition) * 1000.0f * 1000000.0f * 60.0f / ((float)(rampTime + platTime) * 360.0f);
    nomVelocity/=SCALING_FACTOR;

  } else if (control.state == CONTROL_BEGIN_INHALATION) {

    // EVENTUALLY REFACTORING FOR VOLUME TRAJECTORY, temporarily position tracking for testing
    targetPosition = 40;
    rampTime = targetInhalationTime / (2.0f + trapezoidRatio);
    platTime = trapezoidRatio * rampTime;
    nomVelocity = abs((float)targetPosition - (float)currentPosition) * 1000.0f * 1000000.0f * 60.0f / ((float)(rampTime + platTime) * 360.0f);
    nomVelocity/=SCALING_FACTOR;

  } else {
    // Blank
  }
}

// Track timing and scale output based on location in trajectory
static int updateTrajectory()
{

  // Determine elapsed time and place along trajectory
  elapsedTime = timerHalCurrent(&breathTimer) - breathTimerStateStart;

  if (elapsedTime < rampTime) {
    // Ramp up
    velocityScale = ((float)elapsedTime / (float)rampTime);

  } else if ((elapsedTime > rampTime) && (elapsedTime < (platTime + rampTime))) {
    // Plateau
    velocityScale = 1.0;

  } else if (elapsedTime<(platTime + 2*rampTime)) {
    // Ramp down
    velocityScale = (float)(2 * rampTime + platTime - elapsedTime) / (float)rampTime;
  }else
  {
        velocityScale=0.0f;
	targetAirFlow=0.0f;
        return 1;
  }

 targetAirFlow = (nomVelocity * velocityScale);
 // Debug Output Section
  DEBUG_PRINT_EVERY(10, "Target Position: %lu ; Nominal Velocity: %lu ; Target Velocity: %lu", 
    (uint32_t)targetPosition, (uint32_t)nomVelocity, (uint32_t)targetAirFlow);
  // DEBUG_PRINT_EVERY(1000, "Elapsed Time: %lu ; Ramp Time: %lu ; Velocity Scale: %lu", elapsedTime, rampTime, (uint32_t)(1000.0 * velocityScale));
  // DEBUG_PRINT_EVERY(1000, "Proportion of Phase: %lu", velocityScale);
 return 0;
}

// Update control commands and handle ISR/control flags
static int updateControl(void)
{

    float flowSensorInput;
    static float lastFlowSensorInput=0.0f;
    float controlP,controlD;
    float controlOut;
    float controlOutLimited;

    //TEMPORARY ASSIGNMENT. SHOULD BE STORED IN PARAMETERS
    float Kf=0.9f;
    float Kp=1.0f;
    float Kd=0.8f;
    float Ki=.25f;
    float KiMax=80.0f; //max speed adjustment because of I-part in % of nominalVelocity
    float controlMax=100.0f; //max speed adjustment of current target velocity
    

   if (control.state == CONTROL_EXHALATION)
    {
	//deactivate controller for exhilation
	Kf=1.0f;
        Kp=.0f;
        Ki=.0f;
        Kd=.0f;
    }

    metricsStart(&midControlTiming);

    // Update trajectory to get instantaneous target air flow
    if (updateTrajectory()>0)
    {
	motorHalCommand(targetPosition,0);
	DEBUG_PRINT("Target Reached");
	return 1;
    }

    // **************************************************************
    // Here is where closed loop control structure will be developed
    // **************************************************************
    // Depending on whether we are in inhalation or exhalation, we will
    // switch between closed loop air flow tracking and closed loop position
    // tracking. The updateControl() function will accept a flag set by an ISR.


    //read sensor inputs
    flowSensorInput=sensors.currentFlow;//needs to be getFlowSensorInput

    //calculate proportional part
    controlP=targetAirFlow-flowSensorInput;

    //calculate differential part
    controlD=lastFlowSensorInput-flowSensorInput;

    //calculate integral part
    controlI+=controlP;

    //limit integral part
    if ((Ki*controlI)>(nomVelocity*KiMax/100.0f))
        controlI=(nomVelocity*KiMax/100.0f)/Ki;

    if ((Ki*controlI)<(-nomVelocity*KiMax/100.0f))
       controlI=-(nomVelocity*KiMax/100.0f)/Ki;

    //limit and output

    controlOut=Kp*controlP+Kd*controlD+Ki*controlI;


    if (controlOut>(controlMax/100.0f*nomVelocity))
        controlOut=(controlMax/100.0f*nomVelocity);

    if (controlOut<-(controlMax/100.0f*nomVelocity))
        controlOut=-(controlMax/100.0f*nomVelocity);

    //final control output is feed forward + limited controlOut
    controlOutLimited=Kf*targetAirFlow+controlOut;

    controlOutLimited*=SCALING_FACTOR;

    //IIR filter
    controlOutputFiltered=0.9f*controlOutputFiltered+0.1f*controlOutLimited; 


    //TODO: Remove this diry fix here. It needs to be replace with a better trajectory planning.
    if (control.state == CONTROL_EXHALATION)
    {
      // Send motor commands
      motorHalCommand(targetPosition, controlOutputFiltered);
    }else
    {
      // Send motor commands
      motorHalCommand(targetPosition*3, controlOutputFiltered);
    }
    
    lastFlowSensorInput=flowSensorInput;

    metricsStop(&midControlTiming);

    DEBUG_PRINT_EVERY(100,"Control Stats: Avg us: %u\n",midControlTiming.average);
    
    PLOT((int32_t)(flowSensorInput),(int32_t)(targetAirFlow),(int32_t)(controlOutLimited),(int32_t)(Kp*controlP),(int32_t)(controlOutputFiltered),(int32_t)(Ki*controlD));

    // Return unfinished
    return 0;
}

static bool checkInhalationTimeout(void)
{
  return ((timerHalCurrent(&breathTimer) - breathTimerStateStart) >=
          INHALATION_OVERTIME(targetInhalationTime));
}

static bool checkExhalationTimeout(void)
{
  return (timerHalRun(&breathTimer) != HAL_IN_PROGRESS);
}

static PT_THREAD(controlThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  metricsReset(&midControlTiming);

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
      
      // TODO: Actually sync up with how ie is supposed to be represented
      //       for next, fix IE at 1:1.5 (1 / 2.5 = 0x0066)
      parameters.ieRatioRequested = 0x0066;
      parameters.respirationRateRequested = 15;

      // Collect all set points from parameters
      totalBreathTime = (60 SEC) / parameters.respirationRateRequested;
      targetInhalationTime = (parameters.ieRatioRequested * totalBreathTime) >> 8; // TODO: address fixed point math
      targetVolume = parameters.volumeRequested;
      
      // Initialize all 
      measuredInhalationTime = 0;
      measuredExhalationTime = 0;
      
      // Kickoff timer for monitoring breath of breathing
      timerHalBegin(&breathTimer, totalBreathTime, false);
      
      breathTimerStateStart = timerHalCurrent(&breathTimer);
      controlComplete = false;

      // Compute trajectory
      computeTrajectory();

      controlI=0.0f; //reset I-part
      controlOutputFiltered=0.0f;
      control.state = CONTROL_INHALATION;
      
    } else if (control.state == CONTROL_INHALATION) {
      DEBUG_PRINT("state: CONTROL_INHALATION");
      
      // Begin control loop timer when control starts
      timerHalBegin(&controlTimer, CONTROL_LOOP_PERIOD, true);

      while (1) {
        // TODO: Different modes?
        controlComplete = updateControl();
      
        // If the control still hasnt reached its destination and the timeout
        // condition hasnt been met, continue the control loop, waiting for the
        // next control cycle; otherwise exit this state
        if (!controlComplete && !checkInhalationTimeout()) {
          PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
        } else {
          break;
        }
      }

      // Update some things on state exit
      currentPosition = targetPosition;
      measuredInhalationTime = timerHalCurrent(&breathTimer);
        
      control.state = CONTROL_BEGIN_HOLD_IN;
      
    } else if (control.state == CONTROL_BEGIN_HOLD_IN) {
      DEBUG_PRINT("state: CONTROL_BEGIN_HOLD_IN");

      // Setup the hold timer
      timerHalBegin(&controlTimer, HOLD_TIME, false);

      control.state = CONTROL_HOLD_IN;
      
    } else if (control.state == CONTROL_HOLD_IN) {
      DEBUG_PRINT("state: CONTROL_HOLD_IN");

      PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
      control.state = CONTROL_BEGIN_EXHALATION;
      
    } else if (control.state == CONTROL_BEGIN_EXHALATION) {
      DEBUG_PRINT("state: CONTROL_BEGIN_EXHALATION");

      // Compute trajectory
      computeTrajectory();
      
      breathTimerStateStart = timerHalCurrent(&breathTimer);
      controlComplete = false;
      
      control.state = CONTROL_EXHALATION;

    } else if (control.state == CONTROL_EXHALATION) {
      DEBUG_PRINT("state: CONTROL_EXHALATION");
      
      // Begin control loop timer when control starts
      timerHalBegin(&controlTimer, CONTROL_LOOP_PERIOD, true);
      
      while (1) {
        // TODO: Different modes?
        controlComplete = updateControl();
      
        // If the control still hasnt reached its destination and the timeout
        // condition hasnt been met, continue the control loop, waiting for the
        // next control cycle; otherwise move on to the next steps in exhalation
        // TODO: Consider what a timeout here means, it means the motor wasn't
        //       able to get all the way back, should we really keep going?
        if (!controlComplete && !checkInhalationTimeout()) {
          PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
        } else {
          break;
        }
      }
      
      // TODO: look into this
      currentPosition = targetPosition;
      
      // At this point, either wait for the breath timer to expire or find a new
      // breath to sync with
      PT_WAIT_UNTIL(pt, ((timerHalRun(&breathTimer) != HAL_IN_PROGRESS) ||
                         (sensors.inhalationDetected)));
      
      // Calculate and update the measured times
      uint32_t currentBreathTime = timerHalCurrent(&breathTimer);
      measuredExhalationTime = currentBreathTime - HOLD_TIME - measuredInhalationTime;
      control.ieRatioMeasured = (measuredExhalationTime << 8) / measuredInhalationTime; // TODO: address fixed point math
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
