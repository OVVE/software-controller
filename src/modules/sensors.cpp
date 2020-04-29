//
// Sensor Module
//

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "../pt/pt.h"

#include "../hal/sensor/battery.h"
#include "../hal/sensor/sensor.h"
#include "../hal/timer.h"

#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/sensors.h"

#include "../util/utils.h"

#define DEBUG
#define DEBUG_MODULE "sensor"
#include "../util/debug.h"

// 
// Pressure Sensor Parameters
//
#define PRESSURE_SAMPLING_PERIOD                 (10 MSEC)

#define PRESSURE_WINDOW                           (4)

// TODO: Define these thresholds accurately
#define PEEP_PRESSURE_FLAT_THRESHOLD             (20)

#define INHALATION_DETECTION_ABSOLUTE_THRESHOLD   (0)
#define INHALATION_DETECTION_PEEP_THRESHOLD     (100)
#define INHALATION_TIMEOUT                      (400 MSEC)

//
// Airflow Sensor Parameters
//
#define AIRFLOW_SAMPLING_PERIOD                  (10 MSEC)

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

static PT_THREAD(sensorsPressureThreadMain(struct pt* pt))
{
  static int16_t currentMaxPressure = INT16_MIN;
  static bool setPeakPressure = false;
  static int16_t plateauPressureSum = 0;
  static int8_t plateauPressureSampleCount = 0;
  static int16_t previousPressure[PRESSURE_WINDOW];
  static uint8_t inhalationTimeout = 0;

  PT_BEGIN(pt);

  // Kick off sampling timer
  // TODO: refactor to use periodic timer
  timerHalBegin(&pressureTimer, PRESSURE_SAMPLING_PERIOD, false);
  
  int16_t pressure;
  pressureSensorHalGetValue(&pressure);	// get pressure, in [0.1mmH2O]
  
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
    sensors.plateauPressure = plateauPressureSum / plateauPressureSampleCount;
    plateauPressureSum = 0;
    plateauPressureSampleCount = 0;
  }
  
  // Derive PEEP Pressure from pressure readings during exhalation after values
  // have "stabilized" (ie, the difference of any one point from their average
  // is less than some threshold)
  if (control.state == CONTROL_EXHALATION) {
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
  
  DEBUG_PRINT_EVERY(200, "Pressure  = %c%u.%01u mmH2O",
                    (pressure < 0) ? '-' : ' ', abs(pressure)/10, abs(pressure)%10);

  PT_WAIT_UNTIL(pt, timerHalRun(&pressureTimer) != HAL_IN_PROGRESS);
  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(sensorsAirFlowThreadMain(struct pt* pt))
{
  static bool setVolumeIn = false;
  static int16_t volumeMinuteWindow[VOLUME_MINUTE_WINDOW] = {0};
  static uint8_t volumeMinuteTimeout = 0;
  static int16_t maxVolume = INT16_MIN;
  static bool volumeReset = false;
  
  PT_BEGIN(pt);
  
  // Kick off sampling timer
  // TODO: refactor to use periodic timer
  timerHalBegin(&airflowTimer, AIRFLOW_SAMPLING_PERIOD, false);
  
  int16_t airflow;
  int16_t airvolume;
  airflowSensorHalGetFlow(&airflow); // get airflow, in [0.1SLM]
  airflowSensorHalGetVolume(&airvolume); // get airvolume, in [mL] (derived from volume)
  
  // Update public interface with the flow and volume values
  sensors.currentFlow = airflow;
  sensors.currentVolume = airvolume;
  
  // Derive Volume IN from current volume; updating the public value upon entry
  // into CONTROL_HOLD_IN state
  if ((control.state == CONTROL_HOLD_IN) && !setVolumeIn) {
    sensors.volumeIn = airvolume;
    setVolumeIn = true;
  } else if (control.state == CONTROL_EXHALATION) {
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
    airflowSensorHalResetVolume();
    volumeReset = true;
  } else if (control.state == CONTROL_INHALATION) {
    volumeReset = false;
  }
  
  DEBUG_PRINT_EVERY(200, "Airflow   = %c%u.%02u SLM",
                    (airflow < 0) ? '-' : ' ', abs(airflow)/100, abs(airflow)%100);
  DEBUG_PRINT_EVERY(200, "AirVolume = %d mL", airvolume);
  
  PT_WAIT_UNTIL(pt, timerHalRun(&airflowTimer) != HAL_IN_PROGRESS);
  PT_RESTART(pt);
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

  PT_RESTART(pt);
  PT_END(pt);
}

int sensorsModuleInit(void)
{
  // TODO: Improve error propagation for all hal init failures
  if (sensorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  if (batterySensorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }

  PT_INIT(&sensorsThread);
  PT_INIT(&sensorsPressureThread);
  PT_INIT(&sensorsAirFlowThread);
  PT_INIT(&sensorsBatteryThread);
}

int sensorsModuleRun(void)
{
  return (PT_SCHEDULE(sensorsThreadMain(&sensorsThread))) ? MODULE_OK : MODULE_FAIL;
}
