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

// Uncomment the following to enable the current closed loop control
 #define CONTROL_CLOSED_LOOP

// Public Variables
struct control control = {
  .state = CONTROL_IDLE
};

// Private Variables
static struct pt controlThread;
static struct timer breathTimer;
static struct timer controlTimer;

static uint32_t targetVolume=500;
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
static float targetAirFlow = 0.0f;
static uint32_t breathTimerStateStart = 0;

static struct metrics midControlTiming;
static float controlI=0.0f;
static float controlOutputFiltered=0.0f;
static int32_t exhalationTargetPosition=MOTOR_INIT_POSITION;

//this is our initial guess scaling factor to match the trajectory to flow sensor values
#define SCALING_FACTOR 1.20f

/*Flow Trajectory generation
* Depends on: last measured tidal volume, current pressure, diffrence between peak pressure and platau pressure
*

* Initial form (after start of breathing process):
          ----------------t--------------
          |                              |
          |                              |
         h|                              | hb
          |                              |
          |                              |
----------|                              |-----------

Volume= h*t

h=inhalationTrajectoryInitialFlow
t=targetInhalationTime

* goal is to reduce difference between peak pressure and platau pressure AND deliver the set tidal volue
*
* Modified form (slowly adaption to difference between peak pressure and platau pressure):
          |-----
          |      ------                              
          |            ---t2--                         
        h2|                  -------                   
          |                         -----|      
          |                              | hb2
----------|                              |-----------
Volume=hb2*t2+0.5*(h2-hb2)*t2 = 0.5*(h2+hb2)*t2

h2=inhalationTrajectoryStartFlow
hb2=inhalationTrajectoryEndFlow
t2=targetInhalationTime

h and hb for the next cycle need to be adapted to meet the tidal volume
*/

static float inhalationTrajectoryVolumeScale=1.0f; //scale to volume to match desired volume
static float inhalationTrajectoryStartFlow=1.0f; //modified start flow
static float inhalationTrajectoryEndFlow=1.0f; //modified end flow
static float inhalationTrajectoryInitialFlow=1.0f; //flow calculation based on square form
static float inhalationTrajectoryLastVolume=0.0f; //volume delivered in last cycle
static float inhalationTrajectoryCurrentTimeInCycle=0.0f; //0..1

#define INITIAL_FLOW_SAFETY_FACTOR 0.3f //start unmeasured ventilation at lower value and rather increase over time than to deliver too much
float initialFlow()
{
  float timeInS=(float)targetInhalationTime/(float)(1.0f SEC);
  return INITIAL_FLOW_SAFETY_FACTOR*(float)targetVolume/timeInS;
}

float inhalationTrajectoryVolume()
{
    return 0.5f*(inhalationTrajectoryEndFlow+inhalationTrajectoryStartFlow)*((float)targetInhalationTime)/1000000.0f;
}

#define INHALATION_TRAJECTORY_PRESSURE_DIFFERENCE_DEADZONE 5000 //in 0.1mmH20
#define INHALATION_TRAJECTORY_MAX_PRESSURE_DIFFERENCE 30000 //in 0.1mmH20 //TODO: Verify these values.
#define INHALATION_TRAJECTORY_MAX_ADJUSTMENT_PER_CYCLE_AT_MAX_PRESSURE 10.0f //in %
#define INHALATION_TRAJECTORY_DOWN_ADJUSTMENT_PER_CYCLE 0.5f //in %

void inhalationTrajectoryUpdateStartAndEndFlow(void)
{
  //only modify inhalationTrajectory if we see a difference >INHALATION_TRAJECTORY_PRESSURE_DIFFERENCE_DEADZONE, otherwise drag slowly bag to normal

  //TODO: Fix plateau sensor reading
  //we sometimes get negative plateau values -> filter them for now
  if ((sensors.peakPressure<0) || (sensors.plateauPressure<0))
    return;


  if (sensors.peakPressure-sensors.plateauPressure>INHALATION_TRAJECTORY_PRESSURE_DIFFERENCE_DEADZONE)
  {
    float diff=sensors.peakPressure-sensors.plateauPressure;
    if (diff>INHALATION_TRAJECTORY_MAX_PRESSURE_DIFFERENCE)
      diff=INHALATION_TRAJECTORY_MAX_PRESSURE_DIFFERENCE;
        
    float newScale=(1.0f+(INHALATION_TRAJECTORY_MAX_ADJUSTMENT_PER_CYCLE_AT_MAX_PRESSURE/100.0f)*diff/(float)INHALATION_TRAJECTORY_MAX_PRESSURE_DIFFERENCE); 

    DEBUG_PRINT("inh trj: peak: %li plateau: %li scale:%li",(int32_t)((float)sensors.peakPressure*100.0f),(int32_t)((float)sensors.plateauPressure*100.0f),(int32_t)((float)newScale*1000.0f));

    //currently deactivated here
    //scale up start
    //inhalationTrajectoryStartFlow*=newScale;

    //scale down end
    //inhalationTrajectoryEndFlow/=newScale;
  }else
  {
    float newScale=(1.0f-(INHALATION_TRAJECTORY_DOWN_ADJUSTMENT_PER_CYCLE/100.0f)); 

    DEBUG_PRINT("inh trj: peak: %li plateau: %li scale:%li",(int32_t)((float)sensors.peakPressure*100.0f),(int32_t)((float)sensors.plateauPressure*100.0f),(int32_t)((float)newScale*1000.0f));

    //currently deactivated here
    //scale up start
    //inhalationTrajectoryStartFlow*=newScale;

    //scale down end
    //inhalationTrajectoryEndFlow/=newScale;
  }
}

#define INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE 10.0f //in %

float inhalationTrajectoryUpdateVolumeScale(void)
{
  float newScale;
  
  if (inhalationTrajectoryLastVolume==0.0f)
    newScale=1.0f;
  else
    newScale=targetVolume/inhalationTrajectoryLastVolume;

  DEBUG_PRINT("inh trj: vol: %li lastVol: %li",(int32_t)targetVolume,(int32_t)inhalationTrajectoryLastVolume);
  //limit adaption rate
  if (newScale>(1.0f+(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f)))
    newScale=(1.0f+(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f));
 
  if (newScale<(1.0f-(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f)))
    newScale=(1.0f-(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f));

 DEBUG_PRINT("inh trj:  scale:%li",(int32_t)((float)newScale*1000.0f));

  inhalationTrajectoryStartFlow*=newScale;
  inhalationTrajectoryEndFlow*=newScale;
}

//in case the user requested a new volume we take the current form, but scale up to 50% in one cyle
#define INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_NEW_VOLUME_PER_CYCLE 50.0f //in %
float inhalationTrajectoryUpdateToNewVolume(void)
{
  float newScale;
  
  if (inhalationTrajectoryLastVolume==0.0f)
    newScale=1.0f;
  else
    newScale=targetVolume/inhalationTrajectoryLastVolume;

  DEBUG_PRINT("inh trj new vol: vol: %li lastVol: %li",(int32_t)targetVolume,(int32_t)inhalationTrajectoryLastVolume);

  //limit adaption rate
  if (newScale>(1.0f+(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f)))
    newScale=(1.0f+(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f));
 
  if (newScale<(1.0f-(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f)))
    newScale=(1.0f-(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f));

 DEBUG_PRINT("inh trj:  scale:%li",(int32_t)newScale*1000.0f);
  
  inhalationTrajectoryStartFlow*=newScale;
  inhalationTrajectoryEndFlow*=newScale;
}

#define STATE_INHALATION_TRAJECTORY_INIT 0
#define STATE_INHALATION_TRAJECTORY_RAISE_FLOW 1
#define STATE_INHALATION_TRAJECTORY_LOWER_FLOW 2
#define STATE_INHALATION_TRAJECTORY_END 3

#define INHALATION_TRAJECTORY_RAMP_UP_TIME 5.0f //in %
#define INHALATION_TRAJECTORY_NON_ADAPTED_CYCLES 3 //how many breathing cycles do we wait after the user started the process before we adapt to our sensor measurements
static int generateFlowInhalationTrajectory(uint8_t init)
{
  float currentTime;
  static uint8_t inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_INIT;
  static uint8_t initialCycleCnt=0;
 
  // TODO: Actually sync up with how ie is supposed to be represented
  //       for next, fix IE at 1:1.5 (1 / 2.5 = 0x0066)
     
  parameters.ieRatioRequested = 0x0066;
  
  if (init)
  {
    inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_INIT;
    // Collect all set points from parameters
    totalBreathTime = (60 SEC) / parameters.respirationRateRequested;
    targetInhalationTime = (parameters.ieRatioRequested * totalBreathTime) >> 8; // TODO: address fixed point math
    targetVolume = parameters.volumeRequested;
    

    DEBUG_PRINT("InH param: vol:%li respRate:%i ieRatio:%i",targetVolume,parameters.respirationRateRequested,parameters.ieRatioRequested);
  
    inhalationTrajectoryLastVolume=(float)sensors.volumeIn;
    inhalationTrajectoryInitialFlow=initialFlow();
    inhalationTrajectoryStartFlow=inhalationTrajectoryInitialFlow;
    inhalationTrajectoryEndFlow=inhalationTrajectoryInitialFlow;

    DEBUG_PRINT("InH Trj: Init");
  
    initialCycleCnt=INHALATION_TRAJECTORY_NON_ADAPTED_CYCLES; //the first cycles, we don't adapt to measurements

    inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_INIT;
    
    return inhalationTrajectoryState;
  }


  currentTime = (float)(timerHalCurrent(&breathTimer) - breathTimerStateStart)/(float)targetInhalationTime;

  
  if (inhalationTrajectoryState==STATE_INHALATION_TRAJECTORY_INIT)
  {
    // Collect all set points from parameters
    totalBreathTime = (60 SEC) / parameters.respirationRateRequested;
    targetInhalationTime = (parameters.ieRatioRequested * totalBreathTime) >> 8; // TODO: address fixed point math
    targetVolume = parameters.volumeRequested;

    inhalationTrajectoryLastVolume=(float)sensors.volumeIn;

    if (!initialCycleCnt)
    {
      //check if volume or timing got changed during an active process
      if (inhalationTrajectoryInitialFlow!=initialFlow())
      {
        inhalationTrajectoryInitialFlow=initialFlow();
        inhalationTrajectoryUpdateStartAndEndFlow();
        inhalationTrajectoryUpdateToNewVolume();
      }else
      {
        inhalationTrajectoryUpdateStartAndEndFlow();
        inhalationTrajectoryUpdateVolumeScale();
      }
    }else
    {
      inhalationTrajectoryInitialFlow=initialFlow();
      inhalationTrajectoryStartFlow=inhalationTrajectoryInitialFlow;
      inhalationTrajectoryEndFlow=inhalationTrajectoryInitialFlow;
      initialCycleCnt--;
    }

    DEBUG_PRINT("InH Trj: param: if:%li tsF:%li teF: %li tv:%li",(int32_t)(inhalationTrajectoryInitialFlow*100.0f),(int32_t)(inhalationTrajectoryStartFlow*100.0f),(int32_t)(inhalationTrajectoryEndFlow*100.0f),(int32_t)targetVolume);
  
    inhalationTrajectoryCurrentTimeInCycle=0.0f;
    
    targetAirFlow=.0f;
    inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_RAISE_FLOW;


  }else if (inhalationTrajectoryState==STATE_INHALATION_TRAJECTORY_RAISE_FLOW)
  {
    float time=currentTime;
    if (time>INHALATION_TRAJECTORY_RAMP_UP_TIME/100.0f)
      time=INHALATION_TRAJECTORY_RAMP_UP_TIME/100.0f;
    targetAirFlow=inhalationTrajectoryStartFlow*time/(INHALATION_TRAJECTORY_RAMP_UP_TIME/100.0f);

     if (currentTime>=(INHALATION_TRAJECTORY_RAMP_UP_TIME/100.0f))
     {
       inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_LOWER_FLOW;
     }

  }else if (inhalationTrajectoryState==STATE_INHALATION_TRAJECTORY_LOWER_FLOW)
  {
    targetAirFlow=inhalationTrajectoryStartFlow-(inhalationTrajectoryStartFlow-inhalationTrajectoryEndFlow)*(currentTime-(INHALATION_TRAJECTORY_RAMP_UP_TIME/100.0f))/(1.0f-(INHALATION_TRAJECTORY_RAMP_UP_TIME/100.0f));

    if (currentTime>1.0f)
    {
      inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_END;
    }
     
  }else if ((inhalationTrajectoryState==STATE_INHALATION_TRAJECTORY_END) && (currentTime<1.0f))
  {
    inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_INIT;
  }

  targetAirFlow*=10.0f; 

 
   DEBUG_PRINT_EVERY(10, "Inh Trj: Target Velocity: %lu", 
    (uint32_t)targetAirFlow);

  return inhalationTrajectoryState;

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
    // TODO: Currently, enable open loop control if needed, disabling closed loop while in development
#ifdef CONTROL_CLOSED_LOOP
    float Kf=0.9f;
    float Kp=1.0f;
    float Kd=0.8f;
    float Ki=.45f;
#else
    float Kf=1.0f;
    float Kp=0.0f;
    float Kd=0.0f;
    float Ki=0.0f;
#endif
    float KiMax=150.0f; //max speed adjustment because of I-part in % of nominalVelocity
    float controlMax=200.0f; //max speed adjustment of current target velocity
    

    if (control.state == CONTROL_EXHALATION)
    {
	//deactivate controller for exhilation
        //goto a fixed movement and deactivate the controller
	      
        if (motorHalGetPosition()-exhalationTargetPosition<8000)
        {
          targetAirFlow=-(motorHalGetPosition()-exhalationTargetPosition)/2;
        }
        else
          targetAirFlow=-5000.0;
        
        Kf=1.0f;
        Kp=.0f;
        Ki=.0f;
        Kd=.0f;
     
      if ((motorHalGetPosition()<=exhalationTargetPosition)  || (motorHalGetStatus()==MOTOR_STATUS_SWITCH_TRIPPED_TOP))
      {
        motorHalCommand(MOTOR_DIR_STOP,0);
        DEBUG_PRINT("Target Reached %li of %li",motorHalGetPosition(),exhalationTargetPosition);
        return 1;
      }

    }else if (control.state == CONTROL_INHALATION)
    {
      if (generateFlowInhalationTrajectory(0)==STATE_INHALATION_TRAJECTORY_END)
      {
        motorHalCommand(MOTOR_DIR_STOP,0);
        DEBUG_PRINT("Target Reached");
        return 1;

      }

    }
    

    metricsStart(&midControlTiming);

    // Update trajectory to get instantaneous target air flow

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
    if ((Ki*controlI)>(inhalationTrajectoryInitialFlow*KiMax/100.0f))
        controlI=(inhalationTrajectoryInitialFlow*KiMax/100.0f)/Ki;

    if ((Ki*controlI)<(-inhalationTrajectoryInitialFlow*KiMax/100.0f))
       controlI=-(inhalationTrajectoryInitialFlow*KiMax/100.0f)/Ki;

    //limit and output

    controlOut=Kp*controlP+Kd*controlD+Ki*controlI;


    if (controlOut>(controlMax/100.0f*inhalationTrajectoryInitialFlow))
        controlOut=(controlMax/100.0f*inhalationTrajectoryInitialFlow);

    if (controlOut<-(controlMax/100.0f*inhalationTrajectoryInitialFlow))
        controlOut=-(controlMax/100.0f*inhalationTrajectoryInitialFlow);

    //final control output is feed forward + limited controlOut
    controlOutLimited=Kf*targetAirFlow+controlOut;

    //scale with motor angle from 0 to 90deg with scaling factor
    uint32_t currentPos=motorHalGetPosition();
    if (currentPos>90000)
      currentPos=90000;

    float currentScale=90000.0f-(float) currentPos;

    currentScale/=90000.0f;

    currentScale*=SCALING_FACTOR;

    controlOutLimited*=(1.0f+currentScale);

    //IIR filter
    controlOutputFiltered=0.85f*controlOutputFiltered+0.15f*controlOutLimited; 


    //TODO: Remove this diry fix here. It needs to be replace with a better trajectory planning.
    if (controlOutputFiltered<0.0f)
    {
      // Send motor commands
      motorHalCommand(MOTOR_DIR_OPEN, -controlOutputFiltered);
    }else
    {
      // Send motor commands
      motorHalCommand(MOTOR_DIR_CLOSE,controlOutputFiltered);
    }
    
    lastFlowSensorInput=flowSensorInput;

    metricsStop(&midControlTiming);

    DEBUG_PRINT_EVERY(100,"Control Stats: Avg us: %u\n",midControlTiming.average);

    DEBUG_PLOT((int32_t)flowSensorInput, (int32_t)targetAirFlow, (int32_t)controlOutLimited, (int32_t)(Kp*controlP), (int32_t)controlOutputFiltered, (int32_t)(Ki*controlD));

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
  // TV - (dependent upon lung compliance and PEEP setting)

  while (true) {
    if (control.state == CONTROL_IDLE) {
      DEBUG_PRINT_EVERY(10000, "state: CONTROL_IDLE");

      // Reset any parameters
      control.breathCount = 0;
      control.ieRatioMeasured = 0;
      control.respirationRateMeasured = 0;
      
      motorHalCommand(MOTOR_DIR_STOP, 0U);

      // Wait for the parameters to enter the run state before
      PT_WAIT_UNTIL(pt, parameters.startVentilation);
      
      //call with 1 here to start with the conservative initial guess based on user parameters
      generateFlowInhalationTrajectory(1);

      control.state = CONTROL_BEGIN_INHALATION;

      
    } else if (control.state == CONTROL_BEGIN_INHALATION) {
      DEBUG_PRINT("state: CONTROL_BEGIN_INHALATION");
     
      // Initialize all 
      measuredInhalationTime = 0;
      measuredExhalationTime = 0;
      
      // Kickoff timer for monitoring breath of breathing
      timerHalBegin(&breathTimer, totalBreathTime, false);
      
      breathTimerStateStart = timerHalCurrent(&breathTimer);
      controlComplete = false;

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
      measuredInhalationTime = timerHalCurrent(&breathTimer);
        
      control.state = CONTROL_BEGIN_HOLD_IN;
      
    } else if (control.state == CONTROL_BEGIN_HOLD_IN) {
      DEBUG_PRINT("state: CONTROL_BEGIN_HOLD_IN");

      // Setup the hold timer
      timerHalBegin(&controlTimer, HOLD_TIME, false);
      
      motorHalCommand(MOTOR_DIR_STOP, 0U);
      
      control.state = CONTROL_HOLD_IN;
      
    } else if (control.state == CONTROL_HOLD_IN) {
      DEBUG_PRINT("state: CONTROL_HOLD_IN");

      motorHalCommand(MOTOR_DIR_STOP, 0U);

      PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
      control.state = CONTROL_BEGIN_EXHALATION;
      
    } else if (control.state == CONTROL_BEGIN_EXHALATION) {
      DEBUG_PRINT("state: CONTROL_BEGIN_EXHALATION");

      
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
        if (!controlComplete && !checkExhalationTimeout()) {
          PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
        } else {
          break;
        }
      }
      
      motorHalCommand(MOTOR_DIR_STOP, 0);

      // TODO: look into this
      
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

      motorHalCommand(MOTOR_DIR_STOP, 0U);

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
