//
// Sensor Module
//

#include "../pt/pt.h"

#include "../hal/sensor/airflow.h"
#include "../hal/sensor/battery.h"
#include "../hal/sensor/pressure.h"

#include "../modules/module.h"
#include "../modules/sensors.h"

// Public variables
int currentPressure;
int peakPressure;
int plateauPressure; 

// Private Variables
static struct pt sensorsThread;
static struct pt sensorsPressureThread;
static struct pt sensorsAirFlowThread;
static struct pt sensorsBatteryThread;

static PT_THREAD(sensorsPressureThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, pressureSensorHalFetch() != HAL_IN_PROGRESS);

  int rawPressure;
  pressureSensorHalGetValue(&rawPressure);
  
  // TODO: Process pressure reading, store in public variable
  currentPressure = 0;
  peakPressure = 0;
  plateauPressure = 0;
  
  // TODO: add sampling rate delay

  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(sensorsAirFlowThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  // TODO: airflow sensor

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
}

int sensorsModuleRun(void)
{
  return (PT_SCHEDULE(sensorsThreadMain(&sensorsThread))) ? MODULE_OK : MODULE_FAIL;
}
