/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/
//
// Control Module
//

#include <stdint.h>

#include <Arduino.h>

#include "../pt/pt.h"

#include "../config.h"

#include "../hal/estop.h"
#include "../hal/motor.h"
#include "../hal/timer.h"

#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/sensors.h"
#include "../modules/parameters.h"

#include "../util/alarm.h"
#include "../util/metrics.h"

#define LOG_MODULE "control"
#define LOG_LEVEL  LOG_CONTROL_MODULE
#include "../util/log.h"

#define CONTROL_LOOP_PERIOD (10 MSEC)

#define HOLD_TIME           (200 MSEC)

#define INHALATION_OVERTIME(t) ((t) * 4 / 3)

#define CONTROL_INITIAL_PHASESHIFT_MS 200L

// Uncomment the following to enable the current closed loop control
 #define CONTROL_CLOSED_LOOP

// Public Variables
struct control control = {
  .state = CONTROL_STATE_HOME
};

typedef struct __attribute__((packed)){
  long time; //millis
  uint32_t targetInhalationTime;
  uint32_t targetHoldTime;
  uint32_t targetExhalationTime;
  float targetAirFlow;
  float controlI;
  float controlOutputFiltered;
  float volumeScale; //modified start flow
  float startEndScale; //modified end flow
  float inhalationTrajectoryInitialFlow; //flow calculation based on square form
  int32_t inhalationTrajectoryPhaseShiftEstimate; //compressing the air in the bag causes a delay of flow and we therefore need to estimate the phase shift in timing to allow enough inhale time
  int32_t currentFlow;
  int32_t currentPressure;
  uint8_t controlForceHome;
  uint8_t inhalationTrajectoryInitialCycleCnt;
  uint8_t controlLimitHit;
} CONTROL_LOG_DATA;

CONTROL_LOG_DATA controlLogData;

// Private Variables
static struct pt controlThread;
static struct timer breathTimer;
static struct timer controlTimer;

static uint32_t targetVolume=500;
static uint32_t totalBreathTime;
static uint32_t targetInhalationTime;
static uint32_t targetHoldTime;
static uint32_t targetExhalationTime;

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
static int32_t exhalationTargetPosition=MOTOR_HAL_INIT_POSITION;

//this is our initial guess scaling factor to match the trajectory to flow sensor values
#define SCALING_FACTOR 2.60f

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

#define CONTROL_LIMIT_HIT_NONE 0
#define CONTROL_LIMIT_HIT_VOLUME 1
#define CONTROL_LIMIT_HIT_PRESSURE 2
#define CONTROL_LIMIT_GOT_HANDLED 3


static float inhalationTrajectoryVolumeScale=1.0f; //scale to volume to match desired volume
static float inhalationTrajectoryStartFlow=1.0f; //modified start flow
static float inhalationTrajectoryEndFlow=1.0f; //modified end flow
static float inhalationTrajectoryInitialFlow=1.0f; //flow calculation based on square form
static float inhalationTrajectoryLastVolume=0.0f; //volume delivered in last cycle
static float inhalationTrajectoryCurrentTimeInCycle=0.0f; //0..1
static int32_t inhalationTrajectoryPhaseShiftEstimate=CONTROL_INITIAL_PHASESHIFT_MS*(int32_t)1000; //compressing the air in the bag causes a delay of flow and we therefore need to estimate the phase shift in timing to allow enough inhale time
static uint8_t inhalationTrajectoryInitialCycleCnt=0;
static uint8_t controlForceHome=0;
static uint8_t controlLimitHit=CONTROL_LIMIT_HIT_NONE;
static float controlStartEndFlowScale=1.0f;
static float controlVolumeScale=1.0f;

#define INITIAL_FLOW_SAFETY_FACTOR 0.6f //start unmeasured ventilation at lower value and rather increase over time than to deliver too much
float initialFlow()
{
  float timeInS=(float)targetInhalationTime/(float)(1.0f SEC);
  return INITIAL_FLOW_SAFETY_FACTOR*(float)targetVolume/timeInS;
}

float inhalationTrajectoryVolume()
{
    return 0.5f*(inhalationTrajectoryEndFlow+inhalationTrajectoryStartFlow)*((float)targetInhalationTime)/1000000.0f;
}

#define INHALATION_TRAJECTORY_PRESSURE_DIFFERENCE_DEADZONE 50 //in 0.1mmH20
#define INHALATION_TRAJECTORY_MAX_PRESSURE_DIFFERENCE 1000 //in 0.1mmH20 //TODO: Verify these values.
#define INHALATION_TRAJECTORY_MAX_ADJUSTMENT_PER_CYCLE_AT_MAX_PRESSURE 5.0f //in %
#define INHALATION_TRAJECTORY_DOWN_ADJUSTMENT_PER_CYCLE 0.5f //in %
#define INHALATION_TRAJECTORY_MAX_START_END_SCALE 3.0f
#define INHALATION_TRAJECTORY_MAX_SCALE 5.0f //max initial Flow to current flow scale

void inhalationTrajectoryCalcStartEnd()
{
  inhalationTrajectoryStartFlow=inhalationTrajectoryInitialFlow*controlVolumeScale*controlStartEndFlowScale;
  inhalationTrajectoryEndFlow=inhalationTrajectoryInitialFlow*controlVolumeScale/controlStartEndFlowScale;
}

void inhalationTrajectoryUpdateStartAndEndFlow(void)
{
  //only modify inhalationTrajectory if we see a difference >INHALATION_TRAJECTORY_PRESSURE_DIFFERENCE_DEADZONE, otherwise drag slowly bag to normal

  
  if (sensors.peakPressure-sensors.plateauPressure>INHALATION_TRAJECTORY_PRESSURE_DIFFERENCE_DEADZONE)
  {
    float diff=sensors.peakPressure-sensors.plateauPressure;

    if (diff>INHALATION_TRAJECTORY_MAX_PRESSURE_DIFFERENCE)
      diff=INHALATION_TRAJECTORY_MAX_PRESSURE_DIFFERENCE;
    
    float oldVol=inhalationTrajectoryVolume();
    float newScale=(1.0f+(INHALATION_TRAJECTORY_MAX_ADJUSTMENT_PER_CYCLE_AT_MAX_PRESSURE/100.0f)*diff/(float)INHALATION_TRAJECTORY_MAX_PRESSURE_DIFFERENCE); 

    LOG_PRINT(INFO, "inh trj: peak: %li plateau: %li scale:%li",(int32_t)((float)sensors.peakPressure*100.0f),(int32_t)((float)sensors.plateauPressure*100.0f),(int32_t)((float)newScale*1000.0f));
 
    controlStartEndFlowScale*=newScale;

    if (controlStartEndFlowScale>INHALATION_TRAJECTORY_MAX_START_END_SCALE)
      controlStartEndFlowScale=INHALATION_TRAJECTORY_MAX_START_END_SCALE;


  }else
  {
    float newScale=(1.0f-(INHALATION_TRAJECTORY_DOWN_ADJUSTMENT_PER_CYCLE/100.0f)); 

    LOG_PRINT(INFO, "inh trj: peak: %li plateau: %li scale:%li",(int32_t)((float)sensors.peakPressure*100.0f),(int32_t)((float)sensors.plateauPressure*100.0f),(int32_t)((float)newScale*1000.0f));

    if (controlStartEndFlowScale>1.0f)
    {
      controlStartEndFlowScale*=newScale;
      
      if (controlStartEndFlowScale>INHALATION_TRAJECTORY_MAX_START_END_SCALE)
        controlStartEndFlowScale=INHALATION_TRAJECTORY_MAX_START_END_SCALE;
    }else
    {
      controlStartEndFlowScale=1.0f;
    }
    
  }

  inhalationTrajectoryCalcStartEnd();
}

#define INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE 8.0f //in %

float inhalationTrajectoryUpdateVolumeScale(void)
{
  float newScale;
  
  if (inhalationTrajectoryLastVolume==0.0f)
    newScale=1.0f;
  else
    newScale=targetVolume/inhalationTrajectoryLastVolume;

  LOG_PRINT(INFO, "inh trj: vol: %li lastVol: %li scale:%li",(int32_t)targetVolume,(int32_t)inhalationTrajectoryLastVolume,(int32_t)(newScale*1000.0f));
  //limit adaption rate
  if (newScale>(1.0f+(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f)))
    newScale=(1.0f+(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f));
 
  if (newScale<(1.0f-(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f)))
    newScale=(1.0f-(INHALATION_TRAJECTORY_MAX_VOLUME_ADJUSTMENT_PER_CYCLE/100.0f));

  controlVolumeScale*=newScale;

  if (controlVolumeScale>INHALATION_TRAJECTORY_MAX_SCALE)
    controlVolumeScale=INHALATION_TRAJECTORY_MAX_SCALE;

  inhalationTrajectoryCalcStartEnd();

}

#define STATE_INHALATION_TRAJECTORY_INIT 0
#define STATE_INHALATION_TRAJECTORY_RAISE_FLOW 1
#define STATE_INHALATION_TRAJECTORY_LOWER_FLOW 2
#define STATE_INHALATION_TRAJECTORY_END 3

#define INHALATION_TRAJECTORY_RAMP_UP_TIME 5.0f //in %
#define INHALATION_TRAJECTORY_NON_ADAPTED_CYCLES 2 //how many breathing cycles do we wait after the user started the process before we adapt to our sensor measurements
static int generateFlowInhalationTrajectory(uint8_t init)
{
  float currentTime;
  static uint8_t inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_INIT;
  static float lastAirflow=0.0f;
 
  // TODO: Actually sync up with how ie is supposed to be represented
  //       for next, fix IE at 1:1.5 (1 / 2.5 = 0x0066)
  
  //catch old UIs which are not setting IE ratio correctly
  if (!parameters.ieRatioRequested)
    parameters.ieRatioRequested = 0x0066;
  
  if (init)
  {
    inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_INIT;
    // Collect all set points from parameters
    totalBreathTime = (60 SEC) / parameters.respirationRateRequested;
    targetInhalationTime = (parameters.ieRatioRequested * totalBreathTime) >> 8; // TODO: address fixed point math
    targetInhalationTime-= HOLD_TIME; //HOLD is part of inhalation
    targetInhalationTime+=inhalationTrajectoryPhaseShiftEstimate; //allow more time to compensate for initial flow buildup in the bag as measure by the control loop

    targetHoldTime = HOLD_TIME; // fixed value
    
    targetVolume = parameters.volumeRequested;
    

    LOG_PRINT(INFO, "InH param: vol:%li respRate:%i ieRatio:%i",targetVolume,parameters.respirationRateRequested,parameters.ieRatioRequested);
  
    inhalationTrajectoryLastVolume=(float)sensors.volumeIn;
    inhalationTrajectoryInitialFlow=initialFlow();
    controlStartEndFlowScale=1.0f;
    controlVolumeScale=1.0f;
    inhalationTrajectoryStartFlow=inhalationTrajectoryInitialFlow;
    inhalationTrajectoryEndFlow=inhalationTrajectoryInitialFlow;

    LOG_PRINT(INFO, "InH Trj: Init");
  
    inhalationTrajectoryInitialCycleCnt=INHALATION_TRAJECTORY_NON_ADAPTED_CYCLES; //the first cycles, we don't adapt to measurements

    inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_INIT;
    
    targetAirFlow=0.0f;
    return inhalationTrajectoryState;
  }
  


  targetAirFlow=0.0f;

  currentTime = (float)(timerHalCurrent(&breathTimer) - breathTimerStateStart)/(float)targetInhalationTime;

  
  if (inhalationTrajectoryState==STATE_INHALATION_TRAJECTORY_INIT)
  {
    uint32_t lastTargetInhalationTime=targetInhalationTime;
    uint32_t lastVolumeRequested=targetVolume;
    
    // Collect all set points from parameters
    totalBreathTime = (60 SEC) / parameters.respirationRateRequested;
    targetInhalationTime = (parameters.ieRatioRequested * totalBreathTime) >> 8; // TODO: address fixed point math
    targetInhalationTime-= HOLD_TIME; //HOLD is part of inhalation
    targetInhalationTime+=inhalationTrajectoryPhaseShiftEstimate; //allow more time to compensate for initial flow buildup in the bag as measure by the control loop

    targetVolume = parameters.volumeRequested;
    targetHoldTime = HOLD_TIME; // fixed value
    
   
    inhalationTrajectoryLastVolume=(float)sensors.volumeIn;
#define MAX_TIME_CHANGE_BY_PARAMETERS_BEFORE_RESET 20 //in %
#define MAX_VOLUME_CHANGE_BY_PARAMETERS_BEFORE_RESET 20 //in %
    if (!inhalationTrajectoryInitialCycleCnt)
    {
      //check if volume or timing got changed during an active process
      if ((abs(100-(lastTargetInhalationTime*100/targetInhalationTime))>MAX_TIME_CHANGE_BY_PARAMETERS_BEFORE_RESET)
        || (abs(100-(lastVolumeRequested*100/targetVolume))>MAX_VOLUME_CHANGE_BY_PARAMETERS_BEFORE_RESET))
      {
        LOG_PRINT(INFO, "InH Trj: params changed too much -> reset T: %li;%li V: %li;%li",(int32_t)(targetInhalationTime),(int32_t)(lastTargetInhalationTime),(int32_t)(lastVolumeRequested),(int32_t)targetVolume);
  
        inhalationTrajectoryInitialCycleCnt=2;
        inhalationTrajectoryInitialFlow=initialFlow();
        inhalationTrajectoryStartFlow=inhalationTrajectoryInitialFlow;
        inhalationTrajectoryEndFlow=inhalationTrajectoryInitialFlow;
        controlStartEndFlowScale=1.0f;
        controlVolumeScale=1.0f;
      }else
      {
        if (controlLimitHit)
        {
          if (controlLimitHit==CONTROL_LIMIT_HIT_PRESSURE)
          {
          //we hit a limit, make a down adjustment of the current trajectory 
            controlVolumeScale*=0.95;
            inhalationTrajectoryCalcStartEnd();
            controlLimitHit=CONTROL_LIMIT_GOT_HANDLED;
          }else if (controlLimitHit==CONTROL_LIMIT_HIT_VOLUME)
          {
            controlVolumeScale*=0.99;
            inhalationTrajectoryCalcStartEnd();
            controlLimitHit=CONTROL_LIMIT_GOT_HANDLED;
          }
          
        }else
        {
            inhalationTrajectoryUpdateStartAndEndFlow();
            inhalationTrajectoryUpdateVolumeScale();
        }
      }
    }else
    {
      inhalationTrajectoryInitialFlow=initialFlow();
      inhalationTrajectoryStartFlow=inhalationTrajectoryInitialFlow;
      inhalationTrajectoryEndFlow=inhalationTrajectoryInitialFlow;
      controlStartEndFlowScale=1.0f;
      controlVolumeScale=1.0f;

      inhalationTrajectoryInitialCycleCnt--;
    }

    LOG_PRINT(INFO, "InH Trj: param: if:%li tsF:%li teF: %li tv:%li vS:%li seS:%li",(int32_t)(inhalationTrajectoryInitialFlow*10.0f),(int32_t)(inhalationTrajectoryStartFlow*10.0f),(int32_t)(inhalationTrajectoryEndFlow*10.0f),(int32_t)targetVolume,(int32_t)(controlVolumeScale*1000.0f),(int32_t)(controlStartEndFlowScale*1000.0f));
  
    inhalationTrajectoryCurrentTimeInCycle=0.0f;
    
    targetAirFlow=.0f;
    inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_RAISE_FLOW;

    if (controlLimitHit)
          controlLimitHit=CONTROL_LIMIT_HIT_NONE;


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

    if (currentTime>1.0f)
    {
      targetAirFlow=inhalationTrajectoryEndFlow;
      inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_END;
    }else
    {
      targetAirFlow=inhalationTrajectoryStartFlow-(inhalationTrajectoryStartFlow-inhalationTrajectoryEndFlow)*(currentTime-(INHALATION_TRAJECTORY_RAMP_UP_TIME/100.0f))/(1.0f-(INHALATION_TRAJECTORY_RAMP_UP_TIME/100.0f));
    }
    
     
  }else if ((inhalationTrajectoryState==STATE_INHALATION_TRAJECTORY_END) && (currentTime<1.0f))
  {
    targetAirFlow=0.0f;
    inhalationTrajectoryState=STATE_INHALATION_TRAJECTORY_INIT;
  }else if (inhalationTrajectoryState==STATE_INHALATION_TRAJECTORY_END)
  {
    targetAirFlow=0.0f;
  }

  targetAirFlow*=10.0f; 
   

  if (controlLimitHit==CONTROL_LIMIT_GOT_HANDLED)
  {
    targetAirFlow=lastAirflow;
  }
  lastAirflow=targetAirFlow;

  return inhalationTrajectoryState;

}

#define CONTROL_INITIAL_BAG_POSITION_FLOW_TRIGGER 100
// Update control commands and handle ISR/control flags
static int updateControl(void)
{

    float flowSensorInput;
    static float lastFlowSensorInput=0.0f;
    float controlP,controlD;
    float controlOut;
    float controlOutLimited;
    static uint8_t checkForFirstFlow=0;
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
    

    if (control.state == CONTROL_STATE_EXHALATION)
    {
	//deactivate controller for exhilation
        //goto a fixed movement and deactivate the controller
	      
        if (motorHalGetPosition()-exhalationTargetPosition<2000)
        {
          targetAirFlow=-(motorHalGetPosition()-exhalationTargetPosition)/2;
        }
        else
          targetAirFlow=-15000.0;
        
        if (targetAirFlow>-1000.0)
          targetAirFlow=-1000.0;
        Kf=1.0f;
        Kp=.0f;
        Ki=.0f;
        Kd=.0f;
     
      if ((motorHalGetPosition()<=exhalationTargetPosition)  || (motorHalGetStatus()==MOTOR_HAL_STATUS_LIMIT_TOP))
      {
        motorHalCommand(MOTOR_HAL_COMMAND_HOLD,0);
        LOG_PRINT(INFO, "Target Reached %li of %li",motorHalGetPosition(),exhalationTargetPosition);
        return 1;
      }
      checkForFirstFlow=1;
    }else if (control.state == CONTROL_STATE_INHALATION)
    {
      if (generateFlowInhalationTrajectory(0)==STATE_INHALATION_TRAJECTORY_END)
      {
        motorHalCommand(MOTOR_HAL_COMMAND_HOLD,0);
        LOG_PRINT(INFO, "Target Reached");
        return 1;

      }

      #define MAX_PHASE_SHIFT_COMPENSATION 400L //in ms

      if ((checkForFirstFlow)&& (sensors.currentFlow>CONTROL_INITIAL_BAG_POSITION_FLOW_TRIGGER))
      {
        inhalationTrajectoryPhaseShiftEstimate=(8*inhalationTrajectoryPhaseShiftEstimate+2*timerHalCurrent(&breathTimer))/10;

        //limit phase shift

        if (inhalationTrajectoryPhaseShiftEstimate<0)
          inhalationTrajectoryPhaseShiftEstimate=0;

        if (inhalationTrajectoryPhaseShiftEstimate>(int32_t)MAX_PHASE_SHIFT_COMPENSATION*1000L)
          inhalationTrajectoryPhaseShiftEstimate=(int32_t)MAX_PHASE_SHIFT_COMPENSATION*1000L;

        LOG_PRINT(INFO, "New phase shift: %li",inhalationTrajectoryPhaseShiftEstimate);
        checkForFirstFlow=0;
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

    if (control.state==CONTROL_STATE_INHALATION)
    {
      //scale with motor angle from 0 to 90deg with scaling factor
      uint32_t currentPos=motorHalGetPosition();
      if (currentPos>45000)
        currentPos=45000;

      float currentScale=45000.0f-(float) currentPos;

      currentScale/=45000.0f;

      currentScale*=SCALING_FACTOR;

      controlOutLimited*=(1.0f+currentScale);
    }

    //IIR filter
    if (targetAirFlow!=0.0f)
       controlOutputFiltered=0.85f*controlOutputFiltered+0.15f*controlOutLimited; 
    else
      controlOutputFiltered=controlOutLimited;


//scale down target airflow if we are above our tidal volume target
#define INHALATION_TRAJECTORY_MAX_VOLUME_OVERSHOOT 10.0f // in %
    if ((controlLimitHit==CONTROL_LIMIT_HIT_NONE)&&(control.state==CONTROL_STATE_INHALATION)&&(sensors.currentVolume>targetVolume*(1.0f+INHALATION_TRAJECTORY_MAX_VOLUME_OVERSHOOT/(100.0f))))
    {
        controlLimitHit=CONTROL_LIMIT_HIT_VOLUME;
        controlOutputFiltered=0.0f;
        LOG_PRINT(INFO, "Volume limit hit at %li with output %li",sensors.currentVolume,(int32_t)controlOutputFiltered);
    }

    //dynamically limit to max pressure
    if ((controlLimitHit==CONTROL_LIMIT_HIT_NONE)&&(control.state==CONTROL_STATE_INHALATION)&&(sensors.averagePressure>parameters.highPressureLimit))
    {
      controlOutputFiltered=0.0f;
      controlLimitHit=CONTROL_LIMIT_HIT_PRESSURE;
      LOG_PRINT(INFO, "Pressure limit hit at %li  with output %li",sensors.currentPressure,(int32_t)controlOutputFiltered);
    }

    if ((control.state==CONTROL_STATE_INHALATION)&&(controlLimitHit))
    {
      controlOutputFiltered=0.0f;
    }

    //TODO: Remove this diry fix here. It needs to be replace with a better trajectory planning.
    if (controlOutputFiltered<0.0f)
    {

      uint8_t status;
      if (controlOutputFiltered<-65535.0f)
        controlOutputFiltered=-65535.0f;
      // Send motor commands
      status=motorHalCommand(MOTOR_HAL_COMMAND_OPEN, -controlOutputFiltered);

      if ((status==MOTOR_HAL_STATUS_LIMIT_BOTTOM) || (status==MOTOR_HAL_STATUS_LIMIT_TOP))
        controlForceHome=1;
    }else if (controlOutputFiltered>0.0f)
    {
      uint8_t status;
      if (controlOutputFiltered>65535.0f)
        controlOutputFiltered=65535.0f;
      // Send motor commands
      status=motorHalCommand(MOTOR_HAL_COMMAND_CLOSE, controlOutputFiltered);
      if ((status==MOTOR_HAL_STATUS_LIMIT_BOTTOM) || (status==MOTOR_HAL_STATUS_LIMIT_TOP))
        controlForceHome=1;
    }else
    {
      uint8_t status;
      status=motorHalCommand(MOTOR_HAL_COMMAND_HOLD, controlOutputFiltered);
      if ((status==MOTOR_HAL_STATUS_LIMIT_BOTTOM) || (status==MOTOR_HAL_STATUS_LIMIT_TOP))
        controlForceHome=1;
    }
    
    
    lastFlowSensorInput=flowSensorInput;

    //prepare to log data
    static uint8_t logDividerCnt=0;

    logDividerCnt++;

    if (logDividerCnt==4)
    {
      logDividerCnt=0;
      controlLogData.time=millis(); 
      controlLogData.controlForceHome=controlForceHome;
      controlLogData.controlOutputFiltered=controlOutputFiltered;
      controlLogData.startEndScale=controlStartEndFlowScale;
      controlLogData.inhalationTrajectoryInitialCycleCnt=inhalationTrajectoryInitialCycleCnt;
      controlLogData.inhalationTrajectoryInitialFlow=inhalationTrajectoryInitialFlow;
      controlLogData.inhalationTrajectoryPhaseShiftEstimate=inhalationTrajectoryPhaseShiftEstimate;
      controlLogData.volumeScale=controlVolumeScale;
      controlLogData.targetAirFlow=targetAirFlow;
      controlLogData.targetExhalationTime=targetExhalationTime;
      controlLogData.targetHoldTime=targetHoldTime;
      controlLogData.targetInhalationTime=targetInhalationTime;
      controlLogData.controlLimitHit=controlLimitHit;
      controlLogData.controlI=controlI;
      controlLogData.currentFlow=sensors.currentFlow;
      controlLogData.currentPressure=sensors.currentPressure;
    
      LOG_DATA(INFO, LINK_PACKET_TYPE_CONTROL,(uint8_t*)&controlLogData,sizeof(controlLogData));
    }

    metricsStop(&midControlTiming);

    LOG_PRINT_EVERY(100, DEBUG,"Control Stats: Avg us: %u",midControlTiming.average);

    LOG_PLOT((int32_t)flowSensorInput, (int32_t)targetAirFlow, (int32_t)controlOutLimited, (int32_t)(Kp*controlP), (int32_t)controlOutputFiltered, (int32_t)(Ki*controlD), (int32_t)inhalationTrajectoryPhaseShiftEstimate);

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
    if (control.state == CONTROL_STATE_HOME)
    {
      LOG_PRINT(INFO, "state: CONTROL_STATE_HOME");

       motorHalCommand(MOTOR_HAL_COMMAND_OPEN, 5000);
       PT_WAIT_UNTIL(pt, motorHalGetStatus()!=MOTOR_HAL_STATUS_MOVING);
       motorHalCommand(MOTOR_HAL_COMMAND_CLOSE, 5000);
       PT_WAIT_UNTIL(pt, (motorHalGetStatus()!=MOTOR_HAL_STATUS_MOVING) || (motorHalGetPosition()>exhalationTargetPosition) );
 
       motorHalCommand(MOTOR_HAL_COMMAND_HOLD, 0);
       if (!controlForceHome)
       {
        //reuse breath time to wait 1 sec
        timerHalBegin(&breathTimer, 1 SEC, false);
        PT_WAIT_UNTIL(pt, ((timerHalRun(&breathTimer) != HAL_IN_PROGRESS)));
       }else
       {
          controlForceHome=0;
       }
       
       if (sensors.calibrated == SENSORS_ALL_CALIBRATED) {
         control.state=CONTROL_STATE_IDLE;
       } else {
         control.state = CONTROL_STATE_UNCALIBRATED;
       }

    } else if (control.state == CONTROL_STATE_UNCALIBRATED) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_UNCALIBRATED");
      
      LOG_PRINT(WARNING, "system uncalibrated, waiting for UI to calibrate...");
      
      PT_WAIT_UNTIL(pt, parameters.calibrationStep);
      control.state = CONTROL_STATE_SENSOR_CALIBRATION;

    } else if (control.state == CONTROL_STATE_SENSOR_CALIBRATION) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_SENSOR_CALIBRATION");
      
      // Wait until the sensors have all begun calibratation, then wait
      // for calibration to compelete
      PT_WAIT_UNTIL(pt, sensors.calibrated == 0);
      
      LOG_PRINT(DEBUG, "sensors uncalibrated, waiting for calibration...");
      
      PT_WAIT_UNTIL(pt, sensors.calibrated == SENSORS_ALL_CALIBRATED);
      
      control.state = CONTROL_STATE_SENSOR_CALIBRATION_DONE;

    } else if (control.state == CONTROL_STATE_SENSOR_CALIBRATION_DONE) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_SENSOR_CALIBRATION_DONE");
      
      PT_WAIT_UNTIL(pt, !parameters.calibrationStep);
      control.state = CONTROL_STATE_IDLE;
      
    } else if (control.state == CONTROL_STATE_IDLE) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_IDLE");

      // Reset any parameters
      control.breathCount = 0;
      control.ieRatioMeasured = 0;
      control.respirationRateMeasured = 0;
      controlI=0.0f;
      exhalationTargetPosition=MOTOR_HAL_INIT_POSITION;
      inhalationTrajectoryInitialFlow=0.0f;
      controlOutputFiltered=0.0f;
      inhalationTrajectoryPhaseShiftEstimate=CONTROL_INITIAL_PHASESHIFT_MS*(int32_t)1000;

      motorHalCommand(MOTOR_HAL_COMMAND_OFF, 0U);

      // Wait for the parameters to enter the run state before
      PT_WAIT_UNTIL(pt, (parameters.startVentilation || parameters.calibrationStep) && !estopHalAsserted());
      
      if (parameters.startVentilation) {
        //call with 1 here to start with the conservative initial guess based on user parameters
        generateFlowInhalationTrajectory(1);

        control.state = CONTROL_STATE_BEGIN_INHALATION;
      } else if (parameters.calibrationStep) {
        control.state = CONTROL_STATE_SENSOR_CALIBRATION;
      }
      
    } else if (control.state == CONTROL_STATE_BEGIN_INHALATION) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_BEGIN_INHALATION");
     
      // Initialize all 
      measuredInhalationTime = 0;
      measuredExhalationTime = 0;
      
      // Kickoff timer for monitoring breath of breathing
      timerHalBegin(&breathTimer, totalBreathTime, false);
      
      breathTimerStateStart = timerHalCurrent(&breathTimer);
      controlComplete = false;

      controlI=0.0f; //reset I-part
      controlOutputFiltered=0.0f;
      control.state = CONTROL_STATE_INHALATION;
      
    } else if (control.state == CONTROL_STATE_INHALATION) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_INHALATION");
      
      // Begin control loop timer when control starts
      timerHalBegin(&controlTimer, CONTROL_LOOP_PERIOD, true);

      while (1) {
        // TODO: Different modes?
        controlComplete = updateControl();
      
        // If the control still hasnt reached its destination and the timeout
        // condition hasnt been met, continue the control loop, waiting for the
        // next control cycle; otherwise exit this state
        if (!controlComplete && !checkInhalationTimeout() && !estopHalAsserted()) {
          PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
        } else {
          break;
        }
      }


      // Update some things on state exit
      measuredInhalationTime = timerHalCurrent(&breathTimer);
        
      if (estopHalAsserted()) {
        control.state = CONTROL_STATE_HALT;
      } else {
        control.state = CONTROL_STATE_BEGIN_HOLD_IN;
      }
      
    } else if (control.state == CONTROL_STATE_BEGIN_HOLD_IN) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_BEGIN_HOLD_IN");

      // Setup the hold timer
      timerHalBegin(&controlTimer, targetHoldTime, false);
      
      motorHalCommand(MOTOR_HAL_COMMAND_HOLD, 0U);
      
      controlOutputFiltered=0.0f;
           
      control.state = CONTROL_STATE_HOLD_IN;
      
    } else if (control.state == CONTROL_STATE_HOLD_IN) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_HOLD_IN");

      motorHalCommand(MOTOR_HAL_COMMAND_HOLD, 0U);
      controlOutputFiltered=0.0f;

      PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
      control.state = CONTROL_STATE_BEGIN_EXHALATION;
      
    } else if (control.state == CONTROL_STATE_BEGIN_EXHALATION) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_BEGIN_EXHALATION");

      
      breathTimerStateStart = timerHalCurrent(&breathTimer);
      controlComplete = false;
      
      controlI=0.0f;
      motorHalCommand(MOTOR_HAL_COMMAND_HOLD, 0U);

      control.state = CONTROL_STATE_EXHALATION;

    } else if (control.state == CONTROL_STATE_EXHALATION) {
      LOG_PRINT(INFO, "state: CONTROL_STATE_EXHALATION");
      
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
        if (!controlComplete && !checkExhalationTimeout() && !estopHalAsserted()) {
          PT_WAIT_UNTIL(pt, timerHalRun(&controlTimer) != HAL_IN_PROGRESS);
        } else {
          break;
        }
      }
      
      motorHalCommand(MOTOR_HAL_COMMAND_HOLD, 0);

      // TODO: look into this
      
      // At this point, either wait for the breath timer to expire or find a new
      // breath to sync with
      if (!estopHalAsserted()) {
        PT_WAIT_UNTIL(pt, ((timerHalRun(&breathTimer) != HAL_IN_PROGRESS)));
      }
      
      // Calculate and update the measured times
      uint32_t currentBreathTime = timerHalCurrent(&breathTimer);
      measuredExhalationTime = currentBreathTime - measuredInhalationTime;
      control.ieRatioMeasured = (measuredInhalationTime << 8) / currentBreathTime; // TODO: address fixed point math
      control.respirationRateMeasured = ((60 SEC) + ((currentBreathTime >> 1) USEC)) / (currentBreathTime USEC);
      control.breathCount++;

      LOG_PRINT(INFO, "Timing report [ms]: It: %li Ps: %li Et: %li I:E [1=1000]: %li RR:%li",(uint32_t)measuredInhalationTime/(uint32_t)1000, (uint32_t)inhalationTrajectoryPhaseShiftEstimate/(uint32_t)1000, (uint32_t)measuredExhalationTime/1000, (uint32_t)measuredInhalationTime*(uint32_t)1000/(uint32_t)measuredExhalationTime, (uint32_t)control.respirationRateMeasured);
      // Check if we need to continue onto another breath or if ventilation has stopped

      if (estopHalAsserted()) {
        control.state = CONTROL_STATE_HALT;
      } else if (controlForceHome) {
        control.state = CONTROL_STATE_HOME;
      } else if (parameters.startVentilation) {
        control.state = CONTROL_STATE_BEGIN_INHALATION;
      } else {
        control.state = CONTROL_STATE_HOME;
      }
    
    } else if (control.state = CONTROL_STATE_HALT) {
      LOG_PRINT(ERROR, "state: CONTROL_STATE_HALT - ESTOP ASSERTED");
      
      motorHalCommand(MOTOR_HAL_COMMAND_OFF, 0U);
      
      PT_WAIT_UNTIL(pt, !estopHalAsserted());
      
      LOG_PRINT(WARNING, "estop deasserted, homing motor...");
      
      control.state = CONTROL_STATE_HOME;
      
    } else {
      LOG_PRINT(ERROR, "state: (unknown)");

      motorHalCommand(MOTOR_HAL_COMMAND_OFF, 0U);

      // TODO: Error, unknown control state!!!
      control.state = CONTROL_STATE_IDLE;
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
