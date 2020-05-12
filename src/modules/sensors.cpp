//
// Sensor Module
//

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "../pt/pt.h"

#include "../config.h"

#include "../hal/timer.h"
#include "../hal/sensor/battery.h"
#include "../hal/sensor/airflow.h"
#include "../hal/sensor/pressure.h"

#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/sensors.h"
#include "../modules/parameters.h"

#include "../util/utils.h"

#ifdef DEBUG_SENSORS_MODULE
#define DEBUG_MODULE "sensors"
#include "../util/debug.h"
#endif

// 
// Pressure Sensor Parameters
//
#define PRESSURE_SAMPLING_PERIOD                 (10 MSEC)

#define PRESSURE_WINDOW                           (10)

#define INHALATION_TIMEOUT                      (150 MSEC)

//
// Airflow Sensor Parameters
//
#define AIRFLOW_SAMPLING_PERIOD                  (5 MSEC)

#define AIRFLOW_BIAS_SAMPLES                       32
#define PRESSURE_BIAS_SAMPLES                      32

#define VOLUME_MINUTE_PERIOD                      (5 SEC)
#define VOLUME_MINUTE_WINDOW                     (60 SEC) / VOLUME_MINUTE_PERIOD

// 2.0 [SLM] inhalation flow threshold
#define INHALATION_DETECTION_FLOW_ABSOLUTE_THRESHOLD (200)
// 0.9 [SLM] exhalation flow threshold
#define EXHALATION_DETECTION_FLOW_ABSOLUTE_THRESHOLD ( 90)
// Delta of 0.4 [cmH2O] to determine if pressures are roughly equal
#define PRESSURE_EQUAL (40)

// Patient States
#define PATIENT_UNKNOWN    0
#define PATIENT_HOLD_OUT   1
#define PATIENT_INHALATION 2
#define PATIENT_HOLD_IN    3
#define PATIENT_EXHALATION 4

// Public variables
struct sensors sensors;

// Private Variables
static struct pt sensorsThread;
static struct pt sensorsPressureThread;
static struct pt sensorsAirFlowThread;
static struct pt sensorsBatteryThread;
static struct pt sensorsPatientStateThread;

static struct timer pressureTimer;
static struct timer airflowTimer;
static struct timer breathTimer;

#define SENSORS_PEEP_AVG_CNT 8
static uint8_t peepPressureSumCnt;
static uint8_t peepPressureSumPos;
static int32_t peepPressureHistoryBuffer[SENSORS_PEEP_AVG_CNT];

static uint8_t patientState = PATIENT_UNKNOWN;

// In order to properly update measurements and synchronize with the patient,
// model what the patient's current behavior is as a state machine
static PT_THREAD(sensorsPatientStateThreadMain(struct pt* pt))
{
  static uint32_t inhalationTime = 0;
  static uint8_t previousPatientState = PATIENT_UNKNOWN;
  PT_BEGIN(pt);
  
  while (1) {
    if (patientState == PATIENT_UNKNOWN) {
      DEBUG_PRINT("Patient State: UNKNOWN");
      PT_WAIT_UNTIL(pt, parameters.startVentilation);
      
      // Wait for the flow to pick up to move to inhalation
      PT_WAIT_UNTIL(pt, (sensors.currentFlow > INHALATION_DETECTION_FLOW_ABSOLUTE_THRESHOLD));
      patientState = PATIENT_INHALATION;
      
    } else if (patientState == PATIENT_INHALATION) {
      // Calculate the respiratory parameters
      if (previousPatientState != PATIENT_UNKNOWN) {
        // Calculate and update the measured times
        uint32_t currentBreathTime = timerHalCurrent(&breathTimer);
        uint32_t exhalationTime = currentBreathTime - inhalationTime;
        sensors.ieRatioMeasured = (exhalationTime << 8) / inhalationTime; // TODO: address fixed point math
        sensors.respirationRateMeasured = ((60 SEC) + ((currentBreathTime >> 1) USEC)) / (currentBreathTime USEC);
      }
      DEBUG_PRINT("Patient State: INHALATION");
      // Since we just entered INHALATION, kick off the breath timer and trigger
      // an inhalation detection pulse
      timerHalBegin(&breathTimer, INHALATION_TIMEOUT, false);
      sensors.inhalationDetected = true;
      // Note, we expect no breath to be shorter than the synchronization window
      PT_WAIT_UNTIL(pt, timerHalRun(&breathTimer) != HAL_IN_PROGRESS);
      sensors.inhalationDetected = false;
      // Wait on state transition conditions, moving to either hold in if the
      // control FSM moves to hold in or to exhalation if the flow decreases below
      // a threshold and the control FSM is not attempting to deliver a breath
      // TODO: determine if we can get to hold in on every breath?
      PT_WAIT_UNTIL(pt, ((control.state == CONTROL_HOLD_IN)) ||
                        ((sensors.currentFlow < EXHALATION_DETECTION_FLOW_ABSOLUTE_THRESHOLD) &&
                         (control.state != CONTROL_INHALATION) &&
                         (control.state != CONTROL_BEGIN_HOLD_IN)));
      
      patientState = (control.state == CONTROL_HOLD_IN) ? PATIENT_HOLD_IN : PATIENT_EXHALATION;
      if (patientState == PATIENT_EXHALATION) {
        inhalationTime = timerHalCurrent(&breathTimer);
      }
      
    } else if (patientState == PATIENT_HOLD_IN) {
      DEBUG_PRINT("Patient State: HOLD_IN");
      // Wait for control state machine to move to the next state
      // TODO: determine if we can get here on patient spontaneous breaths
      PT_WAIT_UNTIL(pt, (control.state == CONTROL_EXHALATION) ||
                        (control.state == CONTROL_BEGIN_EXHALATION));
      patientState = PATIENT_EXHALATION;
      inhalationTime = timerHalCurrent(&breathTimer);

    } else if (patientState == PATIENT_EXHALATION) {
      // Record the inhalation time from either transition from HOLD_IN or INHALATION
      inhalationTime = timerHalCurrent(&breathTimer);
      
      DEBUG_PRINT("Patient State: EXHALATION");
      
      // Wait for the flow to pick up to move to inhalation or wait for the
      // pressure to flat out to indicate we are at rest
      // TODO: fix transition to HOLD_OUT, the current method does not work
      // TODO: control FSM too?
      PT_WAIT_UNTIL(pt, (sensors.currentFlow > INHALATION_DETECTION_FLOW_ABSOLUTE_THRESHOLD) ||
                        (abs(sensors.currentPressure - sensors.peepPressure) < PRESSURE_EQUAL));
                        
      patientState = (sensors.currentFlow > INHALATION_DETECTION_FLOW_ABSOLUTE_THRESHOLD) ? PATIENT_INHALATION : PATIENT_HOLD_OUT;
      
    } else if (patientState == PATIENT_HOLD_OUT) {
      DEBUG_PRINT("Patient State: HOLD_OUT");
      
      // Wait for the flow to pick up to move to inhalation
      PT_WAIT_UNTIL(pt, (sensors.currentFlow > INHALATION_DETECTION_FLOW_ABSOLUTE_THRESHOLD));
      patientState = PATIENT_INHALATION;
    }
    
    // Update previous state
    previousPatientState = patientState;
    
    PT_YIELD(pt);
  }
  
  // Should never reach here
  PT_END(pt);
}

static PT_THREAD(sensorsPressureThreadMain(struct pt* pt))
{
  static int16_t currentMaxPressure = INT16_MIN;
  static int32_t plateauPressureSum = 0;
  static int8_t plateauPressureSampleCount = 0;
  static int16_t previousPressure[PRESSURE_WINDOW];
  static uint8_t previousPatientState = PATIENT_UNKNOWN;
#ifdef PRESSURE_SENSOR_CALIBRATION_AT_STARTUP
  static int16_t pressureBias = 0;
  static int pressureBiasCounter = PRESSURE_BIAS_SAMPLES;
  static int32_t pressureSum=0;
#endif
  PT_BEGIN(pt);

#ifdef PRESSURE_SENSOR_CALIBRATION_AT_STARTUP
    // Find the bias of the sensor
  while (pressureBiasCounter--) {
    PT_WAIT_UNTIL(pt, pressureSensorHalFetch() != HAL_IN_PROGRESS);
    int16_t pressure;
    pressureSensorHalGetValue(&pressure);
    pressureSum += pressure;
  }
  
  pressureBias = pressureSum / PRESSURE_BIAS_SAMPLES;
  pressureSum = 0;
  
  DEBUG_PRINT("Pressure bias = %c%u.%02u cmH20",
              (pressureBias < 0) ? '-' : ' ', abs(pressureBias)/100, abs(pressureBias)%100);
#endif
  // Kick off sampling timer
  timerHalBegin(&pressureTimer, PRESSURE_SAMPLING_PERIOD, true);
  
  while (1) {
    // Fetch the latest sample from the sensor
    PT_WAIT_UNTIL(pt, pressureSensorHalFetch() != HAL_IN_PROGRESS);
    
    int16_t pressure;
    pressureSensorHalGetValue(&pressure); // get pressure, in [0.1mmH2O]
#ifdef PRESSURE_SENSOR_CALIBRATION_AT_STARTUP
    pressure-=pressureBias;
#endif
    // Update public interface with the pressure value
    sensors.currentPressure = pressure;

    // Shift in new pressure
    for (int i = PRESSURE_WINDOW - 1; i > 0; i--) {
      previousPressure[i] = previousPressure[i - 1];
    }
    previousPressure[0] = pressure;
    
    // Derive Peak Pressure from pressure readings; updating the public value upon
    // patient exhalation
    if ((previousPatientState == PATIENT_HOLD_IN) && (patientState == PATIENT_EXHALATION)) {
      sensors.peakPressure = currentMaxPressure;
      currentMaxPressure = INT16_MIN;
    } else {
      currentMaxPressure = max(currentMaxPressure, pressure);
    }
    
    // Derive Plateau Pressure from pressure readings during hold in; updating
    // public value upon entry into CONTROL_EXHALATION state
    if (patientState == PATIENT_HOLD_IN) {
      plateauPressureSum += pressure;
      plateauPressureSampleCount++;
    } else if ((patientState == PATIENT_EXHALATION) &&
               (plateauPressureSampleCount > 0)) {
      sensors.plateauPressure = plateauPressureSum / plateauPressureSampleCount;
      plateauPressureSum = 0;
      plateauPressureSampleCount = 0;
    }
    
    // Derive PEEP Pressure from an average of the last 7 pressure readings 
    // while the patient is in the hold out state as during this time, no 
    // pressure fluxations should be happening
    if ((patientState == PATIENT_INHALATION) && (previousPatientState == PATIENT_HOLD_OUT)) {
      int32_t sum = 0;
      for (int i = 2; i < PRESSURE_WINDOW; i++) {
        sum += previousPressure[i];
      }
      
      sensors.peepPressure = sum / (PRESSURE_WINDOW - 2);
    }
    
    // Save patient state for next loop
    previousPatientState = patientState;
    
    DEBUG_PRINT_EVERY(100, "Pressure  = %c%u.%01u mmH2O",
                      (pressure < 0) ? '-' : ' ', abs(pressure)/10, abs(pressure)%10);

    // Ensure this threads cannot block if it somehow elapses the timer too fast
    PT_YIELD(pt);

    PT_WAIT_UNTIL(pt, timerHalRun(&pressureTimer) != HAL_IN_PROGRESS);
  }
  
  // Should never reach here
  PT_END(pt);
}

static PT_THREAD(sensorsAirFlowThreadMain(struct pt* pt))
{
  static int32_t airflowSum = 0;
  static uint32_t previousSampleTime = 0;
  static int16_t volumeMinuteWindow[VOLUME_MINUTE_WINDOW] = {0};
  static uint8_t volumeMinuteTimeout = 0;
  static int16_t maxVolume = INT16_MIN;
  static int16_t airflowBias = 0;
  static uint8_t inhalationTimeout = 0;
  static int airflowBiasCounter = AIRFLOW_BIAS_SAMPLES;
  static uint8_t previousPatientState = PATIENT_UNKNOWN;

  PT_BEGIN(pt);
  
  // Find the bias of the sensor
  while (airflowBiasCounter--) {
    PT_WAIT_UNTIL(pt, airflowSensorHalFetch() != HAL_IN_PROGRESS);
    int16_t airflow;
    airflowSensorHalGetValue(&airflow);
    airflowSum += airflow;
  }
  
  airflowBias = airflowSum / AIRFLOW_BIAS_SAMPLES;
  airflowSum = 0;
  
  DEBUG_PRINT("Airflow bias = %c%u.%02u SLM",
              (airflowBias < 0) ? '-' : ' ', abs(airflowBias)/100, abs(airflowBias)%100);

  // Kick off sampling timer
  timerHalBegin(&airflowTimer, AIRFLOW_SAMPLING_PERIOD, true);
  
  while (1) {
    // Fetch the latest sample from the sensor
    PT_WAIT_UNTIL(pt, airflowSensorHalFetch() != HAL_IN_PROGRESS);
    
    // Calculate dt from the current time from the ideal sample time against
    // the previous amount off of the ideal sample time
    uint32_t currentSampleTime;
    uint32_t dt;
    
    currentSampleTime = timerHalCurrent(&airflowTimer);
    dt = AIRFLOW_SAMPLING_PERIOD + currentSampleTime - previousSampleTime;
    previousSampleTime = currentSampleTime;
    
    int16_t airflow;
    int16_t airvolume;
    airflowSensorHalGetValue(&airflow); // get airflow, in [0.1SLM]
    
    // Apply the bias to the flow reading
    airflow -= airflowBias;

    // Integrate the airflow to get the volume
    // In order to preserve precision of the volume, the acculumator should be in
    // units on the order of 0.001mL, since the longest breath cycle is 6 seconds 
    // (5[bpm], 1:1I:E = 12[sec/breath] / 2 = 6[sec/breath] for inhalation/exhalation stages)
    // Over 6 seconds, a sampling rate of 200[Hz] means 1200 samples and if the calculation
    // error is about 1 unit off every time, units of 0.001[mL] means a difference of only
    // 1.2[mL] over the course of a breath (1200[samples] * 0.001[mL] = 1.2[mL]).
    // From 0.01[SLM] flow to 0.001[mL] volume:
    //  Vol = Flow * dt
    //      = (Flow[SLM] / (60[sec/min]) * (10000[0.001mL/0.01L])) * ((dt[usec]) / (1000000[usec/sec]))
    //      = Flow * dt * 10000 / 1000000 / 60
    //      = Flow * dt / 6000
    airflowSum += ((int32_t) airflow) * ((int32_t) dt) / 6000L; // airflowSum in [0.001mL]
    airvolume = airflowSum / 1000L; // airflow in [mL]
    
    // Update public interface with the flow and volume values
    sensors.currentFlow = airflow;
    sensors.currentVolume = airvolume;
    
    // Derive Volume IN from current volume; updating the public value upon entry
    // into patient exhalation
    if ((patientState == CONTROL_EXHALATION) && 
        ((previousPatientState == CONTROL_HOLD_IN) ||
         (previousPatientState == CONTROL_INHALATION))) {
      sensors.volumeIn = airvolume;
    }
    
    // Volume OUT cannot be derived from flow sensor due to position and direction
    
    // Derive Volume/min from current volume by remembering the max volume over
    // last minute volume period and shifting it into the window, updating the
    // minute volume by subtracting out the old value and adding in the new one
    maxVolume = max(maxVolume, airvolume);
    if (volumeMinuteTimeout++ == (uint8_t) (VOLUME_MINUTE_PERIOD / AIRFLOW_SAMPLING_PERIOD)) {
      sensors.volumePerMinute -= volumeMinuteWindow[VOLUME_MINUTE_WINDOW - 1];
      sensors.volumePerMinute += maxVolume;
      for (int i = VOLUME_MINUTE_WINDOW - 1; i > 0; i--) {
        volumeMinuteWindow[i] = volumeMinuteWindow[i - 1];
      }
      volumeMinuteWindow[0] = maxVolume;
      maxVolume = INT16_MIN;
    }

    // Determine if its time to reset the volume integrator, do it upon entry 
    // into patient exhalation
    if ((patientState == PATIENT_EXHALATION) && 
        ((previousPatientState == PATIENT_HOLD_IN) ||
         (previousPatientState == PATIENT_INHALATION))) {
      DEBUG_PRINT("*** Reset Volume ***");
      airflowSum = 0;
    }
    
    // Trigger inhalation detection here when transitioning from patient 
    // exhalation or hold out to inhalation; do so here as its typically related
    // to airflow
    // TODO: maybe find a better home for this
    if ((patientState == PATIENT_INHALATION) && 
        ((previousPatientState == PATIENT_HOLD_OUT) ||
         (previousPatientState == PATIENT_EXHALATION))) {
      DEBUG_PRINT("+++ Breath Detected!");
      sensors.inhalationDetected = true;
    }
    
    // Timeout mechanism for inhalation detector, only set for a brief window
    // during which synchronization is possible
    if (sensors.inhalationDetected &&
        (inhalationTimeout++ == (uint8_t) (INHALATION_TIMEOUT / PRESSURE_SAMPLING_PERIOD))) {
      sensors.inhalationDetected = false;
    }
    
    // Measure Inhalation
    
    // Save patient state for next loop
    previousPatientState = patientState;
    
    DEBUG_PRINT_EVERY(200, "Airflow   = %c%u.%02u SLM",
                      (airflow < 0) ? '-' : ' ', abs(airflow)/100, abs(airflow)%100);
    DEBUG_PRINT_EVERY(200, "AirVolume = %d mL", airvolume);
    
    // Ensure this threads cannot block if it somehow elapses the timer too fast
    PT_YIELD(pt);
    
    PT_WAIT_UNTIL(pt, timerHalRun(&airflowTimer) != HAL_IN_PROGRESS);
  }

  // Should never reach here
  PT_END(pt);
}

static PT_THREAD(sensorsBatteryThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  // TODO: battery sensor
  
  PT_RESTART(pt);
  PT_END(pt);
}

PT_THREAD(sensorsThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  if (!PT_SCHEDULE(sensorsPressureThreadMain(&sensorsPressureThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(sensorsAirFlowThreadMain(&sensorsAirFlowThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(sensorsBatteryThreadMain(&sensorsBatteryThread))) {
    PT_EXIT(pt);
  }
  
  if (!PT_SCHEDULE(sensorsPatientStateThreadMain(&sensorsPatientStateThread))) {
    PT_EXIT(pt);
  }
  
  // TODO: Mess with the units to make the graph scale nicely?
  DEBUG_PLOT(sensors.currentFlow, sensors.currentVolume, sensors.currentPressure);

  PT_RESTART(pt);
  PT_END(pt);
}

int sensorsModuleInit(void)
{
  // TODO: Improve error propagation for all hal init failures
  if (pressureSensorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  if (airflowSensorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  if (batterySensorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }

  PT_INIT(&sensorsThread);
  PT_INIT(&sensorsPressureThread);
  PT_INIT(&sensorsAirFlowThread);
  PT_INIT(&sensorsBatteryThread);
  PT_INIT(&sensorsPatientStateThread);
}

int sensorsModuleRun(void)
{
  return (PT_SCHEDULE(sensorsThreadMain(&sensorsThread))) ? MODULE_OK : MODULE_FAIL;
}
