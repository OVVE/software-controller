//
// Sensor Module
//

#include "../pt/pt.h"
#include <arduino.h>

#include "../hal/sensor/battery.h"
#include "../hal/sensor/sensor.h"
#include "../hal/timer.h"

#include "../modules/module.h"
#include "../modules/sensors.h"

// Public variables
struct sensors sensors;

// Private Variables
static struct pt sensorsThread;
static struct pt sensorsPressureThread;
static struct pt sensorsAirFlowThread;
static struct pt sensorsAirVolumeThread;
static struct pt sensorsBatteryThread;
static struct timer pressureTimer;
static struct timer airflowTimer;
static struct timer airVolumeTimer;

uint32_t last_debug_time;

#define PRESSURE_SAMPLING_RATE 20000
static PT_THREAD(sensorsPressureThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  timerHalBegin(&pressureTimer, PRESSURE_SAMPLING_RATE);  // set to 50ms to not overflow timer. Can be slowed down once timer hal is updated.
  PT_WAIT_UNTIL(pt, timerHalRun(&pressureTimer)!=HAL_IN_PROGRESS);
  int16_t rwPressure;						// pressure in real world units (tenths of mms of H2O)
  pressureSensorHalGetValue(&rwPressure);	// get pressure
  sensors.currentPressure = rwPressure;		// store in public sensor structure
  PT_RESTART(pt);
  PT_END(pt);
}

#define AIRFLOW_SAMPLING_RATE 20000
static PT_THREAD(sensorsAirFlowThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  timerHalBegin(&airflowTimer, AIRFLOW_SAMPLING_RATE);  // set to 50ms to not overflow timer. Can be slowed down once timer hal is updated.
  PT_WAIT_UNTIL(pt, timerHalRun(&airflowTimer)!=HAL_IN_PROGRESS);
  int16_t rwAirflow;						// airflow in real world units (0.01 SLM) 
  airflowSensorHalGetValue(&rwAirflow);		// get airflow
  sensors.currentFlow = rwAirflow;			// store in public sensor structure
  PT_RESTART(pt);
  PT_END(pt);
}

#define AIRVOLUME_SAMPLING_RATE 20000
static PT_THREAD(sensorsAirVolumeThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  timerHalBegin(&airVolumeTimer, AIRVOLUME_SAMPLING_RATE);  // set to 50ms to not overflow timer. Can be slowed down once timer hal is updated.
  PT_WAIT_UNTIL(pt, timerHalRun(&airVolumeTimer)!=HAL_IN_PROGRESS);
  int16_t rwAirVolume;						// air volume in real world units (ml)
  airVolumeSensorHalGetValue(&rwAirVolume);	// get air volume
  sensors.currentVolume = rwAirVolume;		// store in public sensor structure
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

  PT_WAIT_UNTIL(pt, (millis()-last_debug_time)>100);

  last_debug_time=millis();

  if (!PT_SCHEDULE(sensorsPressureThreadMain(&sensorsPressureThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(sensorsAirFlowThreadMain(&sensorsAirFlowThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(sensorsAirVolumeThreadMain(&sensorsAirVolumeThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(sensorsBatteryThreadMain(&sensorsBatteryThread))) {
    PT_EXIT(pt);
  }

  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN)^1);
  Serial.print(sensors.currentPressure);
  Serial.print("    ");
  Serial.print(sensors.currentFlow);
  Serial.print("    ");
  Serial.print(sensors.currentVolume);
  Serial.println();


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

  // Built-in LED initialization
  pinMode(LED_BUILTIN, OUTPUT);
  // Serial debug port
  Serial.begin(38400);
  Serial.print("Start");

  last_debug_time = millis();

  PT_INIT(&sensorsThread);
  PT_INIT(&sensorsPressureThread);
  PT_INIT(&sensorsAirFlowThread);
  PT_INIT(&sensorsBatteryThread);
}

int sensorsModuleRun(void)
{
  return (PT_SCHEDULE(sensorsThreadMain(&sensorsThread))) ? MODULE_OK : MODULE_FAIL;
}
