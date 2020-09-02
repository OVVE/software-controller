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

#define LOG_MODULE "sensors"
#define LOG_LEVEL  LOG_SENSORS_MODULE
#include "../util/log.h"

// 
// Pressure Sensor Parameters
//
#define PRESSURE_SAMPLING_PERIOD                 (10 MSEC)

#define PRESSURE_WINDOW                           (20)
#define PRESSURE_ALARM_WINDOW                    ((150 MSEC) / PRESSURE_SAMPLING_PERIOD)

#define PRESSURE_RESPONSE_THRESHOLD               (500)
#define PRESSURE_MINIMUM_RESPONSE_CYCLES          (20)
#define PRESSURE_MINIMUM_BREATH_COUNT             (5)

// TODO: Define these thresholds accurately
#define PEEP_PRESSURE_FLAT_THRESHOLD             (20)

#define INHALATION_DETECTION_ABSOLUTE_THRESHOLD   (0)
#define INHALATION_DETECTION_PEEP_THRESHOLD     (800)
#define INHALATION_TIMEOUT                      (400 MSEC)

#define CONTINUOUS_PRESSURE_MIN_BREATH           (0)
#define CONTINUOUS_PRESSURE_PERIOD               (500 MSEC)
#define CONTINUOUS_PRESSURE_WINDOW               ((15 SEC) / CONTINUOUS_PRESSURE_PERIOD)
#define CONTINUOUS_PRESSURE_THRESHOLD            (1000) // 10cmH2O

//
// Airflow Sensor Parameters
//
#define AIRFLOW_SAMPLING_PERIOD                  (5 MSEC)

#define AIRFLOW_WINDOW                             (20)

#define AIRFLOW_RESPONSE_THRESHOLD                (500)

#define AIRFLOW_MINIMUM_RESPONSE_CYCLES           (20)

#define AIRFLOW_BIAS_SAMPLES                      32
#define PRESSURE_BIAS_SAMPLES                      32

#define MINUTE_VOLUME_PERIOD                      (4 SEC)
#define MINUTE_VOLUME_WINDOW                    ((60 SEC) / MINUTE_VOLUME_PERIOD)

//
// Battery Sampling Parameters
//
#define BATTERY_SAMPLING_PERIOD                (500 MSEC)
#define BATTERY_LOW_LIMIT                        20  // 20%

// Public variables
struct sensors sensors;

// Private Variables
static struct pt sensorsThread;
static struct pt sensorsPressureThread;
static struct pt sensorsAirFlowThread;
static struct pt sensorsBatteryThread;

static struct timer pressureTimer;
static struct timer airflowTimer;
static struct timer batteryTimer;

#define SENSORS_PEEP_AVG_CNT 8
static uint8_t peepPressureSumCnt;
static uint8_t peepPressureSumPos;
static int32_t peepPressureHistoryBuffer[SENSORS_PEEP_AVG_CNT];

static struct alarmProperties onBatteryAlarmProperties = {
  .priority = ALARM_PRIORITY_MODERATE,
  .preventWatchdog = false,
  .suppressionTimeout = (1200 SEC),
};
static struct alarmProperties lowBatteryAlarmProperties = {
  .priority = ALARM_PRIORITY_HIGH,
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
static struct alarmProperties continuousPressureAlarmProperties = {
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
  static int32_t pressureResponseCount = 0;
  static int32_t pressureAlarmSum = 0;
  static int16_t previousPressureAlarm[PRESSURE_ALARM_WINDOW] = {0};
  static int16_t continuousPressureAlarmMin[CONTINUOUS_PRESSURE_WINDOW] = {0};
  static int16_t continuousPressureAlarmMax[CONTINUOUS_PRESSURE_WINDOW] = {0};
  static uint8_t continuousPressureAlarmTimeout = 0;
  static int16_t continuousPressureAlarmCurrentMin = INT16_MAX;
  static int16_t continuousPressureAlarmCurrentMax = INT16_MIN;
  static int32_t pressureBiasSum = 0;
  static int16_t pressureBias = 0;
  static int pressureBiasCounter = PRESSURE_BIAS_SAMPLES;

  PT_BEGIN(pt);

  // Kick off sampling timer
  timerHalBegin(&pressureTimer, PRESSURE_SAMPLING_PERIOD, true);
  
  while (1) {
    // Fetch the latest sample from the sensor
    PT_WAIT_UNTIL(pt, pressureSensorHalFetch() != HAL_IN_PROGRESS);
    
    int16_t pressure;
    pressureSensorHalGetValue(&pressure); // get pressure, in [0.1mmH2O]

    pressure-=pressureBias;

    // Update public interface with the pressure value
    sensors.currentPressure = pressure;

    // Shift in new pressure
    for (int i = PRESSURE_WINDOW - 1; i > 0; i--) {
      previousPressure[i] = previousPressure[i - 1];
    }
    previousPressure[0] = pressure;

    pressureAlarmSum = 0;
    for (int i = PRESSURE_ALARM_WINDOW - 1; i > 0; i--) {
      previousPressureAlarm[i] = previousPressureAlarm[i - 1];
      pressureAlarmSum += previousPressureAlarm[i];
    }
    previousPressureAlarm[0] = pressure;
    sensors.averagePressure = (pressureAlarmSum / ((int32_t) PRESSURE_ALARM_WINDOW));
    //LOG_PRINT_EVERY(15, DEBUG, "Current Pressure: %i ; Pressure Avg: %i", pressure, (int16_t) sensors.averagePressure);
    
    // Derive Peak Pressure from pressure readings; updating the public value upon
    // entry into CONTROL_STATE_HOLD_IN state
    if ((control.state == CONTROL_STATE_HOLD_IN) && !setPeakPressure) {
      sensors.peakPressure = currentMaxPressure;
      currentMaxPressure = INT16_MIN;
      setPeakPressure = true;
    } else {
      currentMaxPressure = max(currentMaxPressure, pressure);
      if (control.state == CONTROL_STATE_EXHALATION) {
        setPeakPressure = false;
      }
    }
    
    // Derive Plateau Pressure from pressure readings during hold in; updating
    // public value upon entry into CONTROL_STATE_EXHALATION state
    if (control.state == CONTROL_STATE_HOLD_IN) {
      plateauPressureSum += pressure;
      plateauPressureSampleCount++;
    } else if ((control.state == CONTROL_STATE_EXHALATION) &&
               (plateauPressureSampleCount > 0)) {
      sensors.plateauPressure = plateauPressureSum / (int32_t)plateauPressureSampleCount;
      plateauPressureSum = 0;
      plateauPressureSampleCount = 0;
    }
    

    // Take the last 8 pressure values BEFORE we do the next breathing cycle. This should be a good average of our PEEP pressure.

    if (control.state == CONTROL_STATE_INHALATION)
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
        
        sensors.peepPressure = peepPressureSum/(int32_t)peepPressureSumCnt;

        for (uint8_t i=0;i<SENSORS_PEEP_AVG_CNT;i++)
          peepPressureHistoryBuffer[i]=0;

        peepPressureSumCnt=0;
        peepPressureSum=0;
        peepPressureSumPos=0;
      }
    }else if (control.state == CONTROL_STATE_EXHALATION)
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
   /* if (control.state == CONTROL_STATE_EXHALATION) {
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
    if ((control.state == CONTROL_STATE_EXHALATION) && !sensors.inhalationDetected) {
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
    
    // Calibration
    if (control.state == CONTROL_STATE_SENSOR_CALIBRATION) {
      if (pressureBiasCounter == PRESSURE_BIAS_SAMPLES) {
        pressureBiasSum = pressure;
        sensors.calibrated &= ~SENSORS_PRESSURE_CALIBRATED;
        pressureBiasCounter--;
      } else if (pressureBiasCounter > 0) {
        pressureBiasSum += pressure;
        pressureBiasCounter--;
      } else {
        if (!(sensors.calibrated & SENSORS_PRESSURE_CALIBRATED)) {
          pressureBias = pressureBiasSum / PRESSURE_BIAS_SAMPLES;
          LOG_PRINT(INFO, "Pressure bias = %c%u.%02u cmH20",
                    (pressureBias < 0) ? '-' : ' ', abs(pressureBias)/100, abs(pressureBias)%100);
          sensors.calibrated |= SENSORS_PRESSURE_CALIBRATED;
        }
      }
    } else if (pressureBiasCounter == 0) {
      pressureBiasCounter = PRESSURE_BIAS_SAMPLES;
    }
    
    // Alarms
    if ((control.state != CONTROL_STATE_HOME) && (control.state != CONTROL_STATE_IDLE)
	&& (sensors.calibrated & SENSORS_PRESSURE_CALIBRATED)) {
      if (sensors.averagePressure > parameters.highPressureLimit) {
        LOG_PRINT_EVERY(100, INFO, "High Pressure Alarm! Measured: %i ; Limit: %i", (int16_t) sensors.averagePressure, parameters.highPressureLimit);
        alarmSet(&sensors.highPressureAlarm);
      }
      if (sensors.averagePressure < parameters.lowPressureLimit) {
        LOG_PRINT_EVERY(1,INFO,  "Low Pressure Alarm! Measured: %i ; Limit: %i", (int16_t) sensors.averagePressure, parameters.lowPressureLimit);
        alarmSet(&sensors.lowPressureAlarm);
      }
      // Continuous pressure alarm fires if the  pressure does not have a delta 
      // of at least 10cmH2O between min and max values over the past 15 [sec].
      // Instead of storing all the data, just store the mins and maxes for 500
      // [msec] blocks and use these are proxies for their periods, shifting
      // them out every 500 [msec].
      if (continuousPressureAlarmTimeout++ == (uint8_t) (CONTINUOUS_PRESSURE_PERIOD / PRESSURE_SAMPLING_PERIOD)) {
        int16_t currentMin = INT16_MAX;
        int16_t currentMax = INT16_MIN;
        
        for (int i = CONTINUOUS_PRESSURE_WINDOW - 1; i > 0; i--) {
          continuousPressureAlarmMin[i] = continuousPressureAlarmMin[i - 1];
          continuousPressureAlarmMax[i] = continuousPressureAlarmMax[i - 1];
          currentMin = min(currentMin, continuousPressureAlarmMin[i]);
          currentMax = max(currentMax, continuousPressureAlarmMax[i]);
        }
        continuousPressureAlarmMin[0] = continuousPressureAlarmCurrentMin;
        continuousPressureAlarmMax[0] = continuousPressureAlarmCurrentMax;
        currentMin = min(currentMin, continuousPressureAlarmMin[0]);
        currentMax = max(currentMax, continuousPressureAlarmMax[0]);
        
        if ((control.breathCount > CONTINUOUS_PRESSURE_MIN_BREATH) &&
            ((currentMax - currentMin) < CONTINUOUS_PRESSURE_THRESHOLD)) {
          LOG_PRINT_EVERY(2, ERROR, "Continuous Pressure Alarm! (%d - %d)", currentMin, currentMax);
          alarmSet(&sensors.continuousPressureAlarm);
        } else {
          LOG_PRINT(DEBUG, "CP Window: %d - %d", currentMin, currentMax);
        }
        
        continuousPressureAlarmCurrentMin = INT16_MAX;
        continuousPressureAlarmCurrentMax = INT16_MIN;
        continuousPressureAlarmTimeout = 0;
      } else {
        continuousPressureAlarmCurrentMin = min(continuousPressureAlarmCurrentMin, pressure);
        continuousPressureAlarmCurrentMax = max(continuousPressureAlarmCurrentMax, pressure);
      }
    }

    // Check pressure sensor for nominal response
    if (control.state==CONTROL_STATE_IDLE)
    {
      pressureResponseCount=0;
    }else if (control.state == CONTROL_STATE_INHALATION) {
      if (pressureResponseCount==-1)
          pressureResponseCount=0;
      if (sensors.averagePressure > PRESSURE_RESPONSE_THRESHOLD ) {
          pressureResponseCount += 1;
      }
    }else if(control.state==CONTROL_STATE_EXHALATION)
    {
      if (pressureResponseCount!=-1)
        LOG_PRINT(INFO,"PressureResponseCnt: %li",pressureResponseCount);
      //check if we got any response on the pressure sensor
      if ((pressureResponseCount<PRESSURE_MINIMUM_RESPONSE_CYCLES) && (pressureResponseCount!=-1) && (control.breathCount>PRESSURE_MINIMUM_BREATH_COUNT)) //pressure might need to build up, so ignore the first couple breaths after a start
      {
        LOG_PRINT_EVERY(1, INFO, "Pressure Unresponsive Alarm!");
        alarmSet(&sensors.badPressureSensorAlarm);
      }
      pressureResponseCount=-1;
    }

     LOG_PRINT_EVERY(100, INFO,"Pressure  = %c%u.%02u cmH2O",
                       (pressure < 0) ? '-' : ' ', abs(pressure)/100, abs(pressure)%100);

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
  static int32_t airflowBiasSum = 0;
  static uint32_t previousSampleTime = 0;
  static bool setVolumeIn = false;
  static int32_t minuteVolumeWindow[MINUTE_VOLUME_WINDOW] = {0};
  static uint16_t minuteVolumeTimeout = 0;
  static int32_t minuteVolumeSum;
  static bool volumeReset = false;
  static int16_t airflowBias = 0;
  static int airflowBiasCounter = AIRFLOW_BIAS_SAMPLES;
  static int16_t previousAirflow[AIRFLOW_WINDOW];
  static int16_t airflowResponseCount = 0;
  
  PT_BEGIN(pt);

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

    // Shift in new airflow
    for (int i = AIRFLOW_WINDOW - 1; i > 0; i--) {
      previousAirflow[i] = previousAirflow[i - 1];
    }
    previousAirflow[0] = airflow;
    
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
    // into CONTROL_STATE_HOLD_IN state
    // if ((control.state == CONTROL_STATE_HOLD_IN) && !setVolumeIn) {
    if ((control.state == CONTROL_STATE_EXHALATION) && !setVolumeIn) {
      sensors.volumeIn = airvolume;
      setVolumeIn = true;
    // } else if (control.state == CONTROL_STATE_EXHALATION) {
    } else if (control.state == CONTROL_STATE_INHALATION) {
      setVolumeIn = false;
    }
    
    // Volume OUT cannot be derived from flow sensor due to position and direction
    
    // Derive Minute Volume by running a parallel acculumator over
    if (minuteVolumeTimeout++ == (uint16_t) (MINUTE_VOLUME_PERIOD / AIRFLOW_SAMPLING_PERIOD)) {
      sensors.minuteVolume -= minuteVolumeWindow[MINUTE_VOLUME_WINDOW - 1];
      sensors.minuteVolume += minuteVolumeSum / 1000L;
      for (int i = MINUTE_VOLUME_WINDOW - 1; i > 0; i--) {
        minuteVolumeWindow[i] = minuteVolumeWindow[i - 1];
      }
      minuteVolumeWindow[0] = minuteVolumeSum / 1000L;
      minuteVolumeSum = 0;
      minuteVolumeTimeout = 0;
      LOG_PRINT(DEBUG, "Minute Volume = %ld mL", sensors.minuteVolume);
    } else {
      // Same math as the airflowSum; see above
      minuteVolumeSum += ((int32_t) airflow) * ((int32_t) dt) / 6000L;
    }

    // Determine if its time to reset the volume integrator, do it in idle and
    // once in exhalation
    if (control.state==CONTROL_STATE_IDLE) {
      airflowSum = 0;
      minuteVolumeSum = 0;
      sensors.volumeIn = 0;
    } else if ((control.state == CONTROL_STATE_EXHALATION) && !volumeReset) {
      LOG_PRINT(INFO, "*** Reset Volume ***");
      airflowSum = 0;
      volumeReset = true;
    } else if (control.state == CONTROL_STATE_INHALATION) {
      volumeReset = false;
    }
    
    // Calibration
    if (control.state == CONTROL_STATE_SENSOR_CALIBRATION) {
      if (airflowBiasCounter == AIRFLOW_BIAS_SAMPLES) {
        airflowBiasSum = airflow;
        sensors.calibrated &= ~SENSORS_AIRFLOW_CALIBRATED;
        airflowBiasCounter--;
      } else if (airflowBiasCounter > 0) {
        airflowBiasSum += airflow;
        airflowBiasCounter--;
      } else {
        if (!(sensors.calibrated & SENSORS_AIRFLOW_CALIBRATED)) {
          airflowBias = airflowBiasSum / AIRFLOW_BIAS_SAMPLES;
          LOG_PRINT(INFO, "Airflow bias = %c%u.%02u SLM",
                    (airflowBias < 0) ? '-' : ' ', abs(airflowBias)/100, abs(airflowBias)%100);
          sensors.calibrated |= SENSORS_AIRFLOW_CALIBRATED;
        }
      }
    } else if (airflowBiasCounter == 0) {
      airflowBiasCounter = AIRFLOW_BIAS_SAMPLES;
    }
    
    // Alarms
    if (control.state == CONTROL_STATE_HOLD_IN) {
      if (airvolume > parameters.highVolumeLimit) {
        LOG_PRINT_EVERY(10, INFO, "High Volume Alarm! Measured: %i ; Limit: %i", airvolume, parameters.highVolumeLimit);
        alarmSet(&sensors.highVolumeAlarm);
      }
      if (airvolume < parameters.lowVolumeLimit) {
        LOG_PRINT_EVERY(10, INFO, "Low Volume Alarm! Measured: %i ; Limit: %i", airvolume, parameters.lowVolumeLimit);
        alarmSet(&sensors.lowVolumeAlarm);
      }
    }

// Check pressure sensor for nominal response
    if (control.state==CONTROL_STATE_IDLE)
    {
      airflowResponseCount=0;
    }else if (control.state == CONTROL_STATE_INHALATION) {
      if (airflowResponseCount==-1)
          airflowResponseCount=0;
      if (airflow > AIRFLOW_RESPONSE_THRESHOLD ) {
        airflowResponseCount += 1;
      }
    }else if(control.state==CONTROL_STATE_EXHALATION)
    {
      if (airflowResponseCount!=-1)
        LOG_PRINT(INFO,"AirflowResponseCnt: %i",airflowResponseCount);
      //check if we got any response on the pressure sensor
      if ((airflowResponseCount<AIRFLOW_MINIMUM_RESPONSE_CYCLES) && (airflowResponseCount!=-1))
      {
        LOG_PRINT_EVERY(1, INFO, "Airflow Unresponsive Alarm!");
        alarmSet(&sensors.badAirflowSensorAlarm);
      }
      airflowResponseCount=-1;
    }

    LOG_PRINT_EVERY(200, INFO, "Airflow   = %c%u.%02u SLM",
                      (airflow < 0) ? '-' : ' ', abs(airflow)/100, abs(airflow)%100);
    LOG_PRINT_EVERY(200, INFO, "AirVolume = %d mL", airvolume);
    
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
  
  // Kick off sampling timer
  timerHalBegin(&batteryTimer, BATTERY_SAMPLING_PERIOD, true);
  
  while (1) {
    // Fetch the latest sample from the sensor
    PT_WAIT_UNTIL(pt, batterySensorHalFetch() != HAL_IN_PROGRESS);
    
    batterySensorHalGetValue(&sensors.batteryPercent, &sensors.batteryCharging);
    
    if (!sensors.batteryCharging) {
      if (!alarmGet(&sensors.onBatteryAlarm)) {
        LOG_PRINT(WARNING, "AC power lost!");
      }
      alarmSet(&sensors.onBatteryAlarm);
    }
    
    if (sensors.batteryPercent <= BATTERY_LOW_LIMIT) {
      LOG_PRINT_EVERY(2, WARNING, "Battery low! (%u%%)", sensors.batteryPercent);
      alarmSet(&sensors.lowBatteryAlarm);
    }
    
    LOG_PRINT_EVERY(4, INFO, "battery: charging (%d), %u%%", sensors.batteryCharging, sensors.batteryPercent);
  
    PT_WAIT_UNTIL(pt, timerHalRun(&batteryTimer) != HAL_IN_PROGRESS);
  }

  // Should never reach here
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
  LOG_PLOT(sensors.currentFlow, sensors.currentVolume, sensors.currentPressure);

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
  alarmInit(&sensors.continuousPressureAlarm, &continuousPressureAlarmProperties);
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
