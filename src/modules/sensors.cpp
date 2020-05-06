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

#include "../util/alarm.h"
#include "../util/utils.h"

#ifdef DEBUG_SENSORS_MODULE
#define DEBUG_MODULE "sensors"
#include "../util/debug.h"
#endif

// 
// Pressure Sensor Parameters
//
#define PRESSURE_SAMPLING_PERIOD                 (10 MSEC)

#define PRESSURE_WINDOW                           (4)

// TODO: Define these thresholds accurately
#define PEEP_PRESSURE_FLAT_THRESHOLD             (20)

#define INHALATION_DETECTION_ABSOLUTE_THRESHOLD   (0)
#define INHALATION_DETECTION_PEEP_THRESHOLD     (800)
#define INHALATION_TIMEOUT                      (400 MSEC)

//
// Airflow Sensor Parameters
//
#define AIRFLOW_SAMPLING_PERIOD                  (5 MSEC)

#define AIRFLOW_BIAS_SAMPLES                      32
#define PRESSURE_BIAS_SAMPLES                      32

#define VOLUME_MINUTE_PERIOD                      (5 SEC)
#define VOLUME_MINUTE_WINDOW                     (60 SEC) / VOLUME_MINUTE_PERIOD

// Public variables
struct sensors sensors;

// Private Variables
static struct pt sensorsThread;
static struct pt sensorsPressureThread;
static struct pt sensorsAirFlowThread;
static struct pt sensorsBatteryThread;

static struct timer pressureTimer;
static struct timer airflowTimer;

#define SENSORS_PEEP_AVG_CNT 8
static uint8_t peepPressureSumCnt;
static uint8_t peepPressureSumPos;
static int32_t peepPressureHistoryBuffer[SENSORS_PEEP_AVG_CNT];

static struct alarmProperties onBatteryAlarmProperties = {
  .priority = ALARM_PRIORITY_MODERATE,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};
static struct alarmProperties lowBatteryAlarmProperties = {
  .priority = ALARM_PRIORITY_MODERATE,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};
static struct alarmProperties badPressureSensorAlarmProperties = {
  .priority = ALARM_PRIORITY_SEVERE,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};
static struct alarmProperties badAirflowSensorAlarmProperties = {
  .priority = ALARM_PRIORITY_SEVERE,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};
static struct alarmProperties highPressureAlarmProperties = {
  .priority = ALARM_PRIORITY_HIGH,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};
static struct alarmProperties lowPressureAlarmProperties = {
  .priority = ALARM_PRIORITY_HIGH,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};
static struct alarmProperties highVolumeAlarmProperties = {
  .priority = ALARM_PRIORITY_HIGH,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};
static struct alarmProperties lowVolumeAlarmProperties = {
  .priority = ALARM_PRIORITY_HIGH,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};
static struct alarmProperties highRespiratoryRateAlarmProperties = {
  .priority = ALARM_PRIORITY_HIGH,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};
static struct alarmProperties lowRespiratoryRateAlarmProperties = {
  .priority = ALARM_PRIORITY_HIGH,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};

static PT_THREAD(sensorsPressureThreadMain(struct pt* pt))
{
  static int16_t currentMaxPressure = INT16_MIN;
  static bool setPeakPressure = false;
  static int32_t plateauPressureSum = 0;
  static int8_t plateauPressureSampleCount = 0;
  static int16_t previousPressure[PRESSURE_WINDOW];
  static uint8_t inhalationTimeout = 0;
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
    // entry into CONTROL_HOLD_IN state
    if ((control.state == CONTROL_HOLD_IN) && !setPeakPressure) {
      sensors.peakPressure = currentMaxPressure;
      currentMaxPressure = INT16_MIN;
      setPeakPressure = true;
    } else {
      currentMaxPressure = max(currentMaxPressure, pressure);
      if (control.state == CONTROL_EXHALATION) {
        setPeakPressure = false;
      }
    }
    
    // Derive Plateau Pressure from pressure readings during hold in; updating
    // public value upon entry into CONTROL_EXHALATION state
    if (control.state == CONTROL_HOLD_IN) {
      plateauPressureSum += pressure;
      plateauPressureSampleCount++;
    } else if ((control.state == CONTROL_EXHALATION) &&
               (plateauPressureSampleCount > 0)) {
      sensors.plateauPressure = plateauPressureSum / (int32_t)plateauPressureSampleCount;
      plateauPressureSum = 0;
      plateauPressureSampleCount = 0;
    }
    

    // Take the last 8 pressure values BEFORE we do the next breathing cycle. This should be a good average of our PEEP pressure.

    if (control.state == CONTROL_INHALATION)
    {
      //reset values
      if (peepPressureSumCnt)
      {
        static int32_t peepPressureSum; 
        //calculate values and reset

        peepPressureSum=0;
        
        for (uint8_t i=0;i<SENSORS_PEEP_AVG_CNT;i++)
        {
          peepPressureSum+=peepPressureHistoryBuffer[i];
        }
        
        sensors.peepPressure = peepPressureSum/peepPressureSumCnt;

        for (uint8_t i=0;i<SENSORS_PEEP_AVG_CNT;i++)
          peepPressureHistoryBuffer[i]=0;

        peepPressureSumCnt=0;
        peepPressureSum=0;
        peepPressureSumPos=0;
      }
    }else if (control.state == CONTROL_EXHALATION)
    {
      peepPressureHistoryBuffer[peepPressureSumPos]=pressure;

      peepPressureSumPos++;
      peepPressureSumPos%=SENSORS_PEEP_AVG_CNT;

      if (peepPressureSumCnt<SENSORS_PEEP_AVG_CNT)
        peepPressureSumCnt++;
    }

    // Derive PEEP Pressure from pressure readings during exhalation after values
    // have "stabilized" (ie, the difference of any one point from their average
    // is less than some threshold)
   /* if (control.state == CONTROL_EXHALATION) {
      int32_t sum = 0;
      for (int i = 0; i < PRESSURE_WINDOW; i++) {
        sum += previousPressure[i];
      }
      
      int16_t average = sum / PRESSURE_WINDOW;
      
      bool flat = true;
      for (int i = 0; i < PRESSURE_WINDOW; i++) {
        if (abs(average - previousPressure[i]) > PEEP_PRESSURE_FLAT_THRESHOLD) {
          flat = false;
        }
      }
      
      // TODO: determine if we want this reading to update so long as it can, or
      // measure once per breath, like other derived pressures
      if (flat) {
        sensors.peepPressure = pressure;
      }
    }
   */ 
    // Derive inhalation detection from pressure readings during exhalation by
    // looking for a dip in pressure below the PEEP threshold (or below an
    // absolute pressure threshold)
    if ((control.state == CONTROL_EXHALATION) && !sensors.inhalationDetected) {
      if ((pressure < INHALATION_DETECTION_ABSOLUTE_THRESHOLD) ||
          (sensors.peepPressure - pressure > INHALATION_DETECTION_PEEP_THRESHOLD)) {
        sensors.inhalationDetected = true;
        inhalationTimeout = 0;
      }
    }
    if (sensors.inhalationDetected &&
        (inhalationTimeout++ == (uint8_t) (INHALATION_TIMEOUT / PRESSURE_SAMPLING_PERIOD))) {
      sensors.inhalationDetected = false;
    }
    
    // Alarms
    if (pressure > parameters.highPressureLimit) {
      alarmSet(&sensors.highPressureAlarm);
    }
    if (pressure < parameters.lowPressureLimit) {
      alarmSet(&sensors.lowPressureAlarm);
    }
    
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
  static bool setVolumeIn = false;
  static int16_t volumeMinuteWindow[VOLUME_MINUTE_WINDOW] = {0};
  static uint8_t volumeMinuteTimeout = 0;
  static int16_t maxVolume = INT16_MIN;
  static bool volumeReset = false;
  static int16_t airflowBias = 0;
  static int airflowBiasCounter = AIRFLOW_BIAS_SAMPLES;

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
    // into CONTROL_HOLD_IN state
    // if ((control.state == CONTROL_HOLD_IN) && !setVolumeIn) {
    if ((control.state == CONTROL_EXHALATION) && !setVolumeIn) {
      sensors.volumeIn = airvolume;
      setVolumeIn = true;
    // } else if (control.state == CONTROL_EXHALATION) {
    } else if (control.state == CONTROL_INHALATION) {
      setVolumeIn = false;
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

    // Determine if its time to reset the volume integrator, do it once in exhalation
    if ((control.state == CONTROL_EXHALATION) && !volumeReset) {
      DEBUG_PRINT("*** Reset Volume ***");
      airflowSum = 0;
      volumeReset = true;
    } else if (control.state == CONTROL_INHALATION) {
      volumeReset = false;
    }
    
    // Alarms
    if (airvolume > parameters.highVolumeLimit) {
      alarmSet(&sensors.highVolumeAlarm);
    }
    if (airvolume < parameters.lowVolumeLimit) {
      alarmSet(&sensors.lowVolumeAlarm);
    }
    
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
  
  // Initialize Alarms
  alarmInit(&sensors.onBatteryAlarm, &onBatteryAlarmProperties);
  alarmInit(&sensors.lowBatteryAlarm, &lowBatteryAlarmProperties);
  alarmInit(&sensors.badPressureSensorAlarm, &badPressureSensorAlarmProperties);
  alarmInit(&sensors.badAirflowSensorAlarm, &badAirflowSensorAlarmProperties);
  alarmInit(&sensors.highPressureAlarm, &highPressureAlarmProperties);
  alarmInit(&sensors.lowPressureAlarm, &lowPressureAlarmProperties);
  alarmInit(&sensors.highVolumeAlarm, &highVolumeAlarmProperties);
  alarmInit(&sensors.lowVolumeAlarm, &lowVolumeAlarmProperties);
  alarmInit(&sensors.highRespiratoryRateAlarm, &highRespiratoryRateAlarmProperties);
  alarmInit(&sensors.lowRespiratoryRateAlarm, &lowRespiratoryRateAlarmProperties);

  PT_INIT(&sensorsThread);
  PT_INIT(&sensorsPressureThread);
  PT_INIT(&sensorsAirFlowThread);
  PT_INIT(&sensorsBatteryThread);
}

int sensorsModuleRun(void)
{
  return (PT_SCHEDULE(sensorsThreadMain(&sensorsThread))) ? MODULE_OK : MODULE_FAIL;
}
