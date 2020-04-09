//
// Sensor Module
//

#include "../pt/pt.h"

#include "../hal/sensor/battery.h"
#include "../hal/sensor/sensor.h"

#include "../modules/module.h"
#include "../modules/sensors.h"

// Public variables
struct sensors sensors;

// Private Variables
static struct pt sensorsThread;
static struct pt sensorsPressureThread;
static struct pt sensorsAirFlowThread;
static struct pt sensorsBatteryThread;

unsigned long last_debug_time;

static PT_THREAD(sensorsPressureThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  int rawPressure;
  pressureSensorHalGetValue(&rawPressure);
  long temp = rawPressure;

  long pascals = ((temp * 217L) - 110995L)>>2;	// convert to Pascals
  long u100umH2O = (pascals * 4177L)>>12;				// convert Pascals to 0.1mmH2O
  sensors.currentPressure = u100umH2O;					// set in public sensor structure

//  sensors.currentPressure = pascals;					// set in public sensor structure


  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(sensorsAirFlowThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  int rawAirflow;
  airflowSensorHalGetValue(&rawAirflow);
  sensors.currentFlow = rawAirflow;

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

  PT_WAIT_UNTIL(pt, (millis()-last_debug_time)>250);

  last_debug_time=millis();

  if (!PT_SCHEDULE(sensorsPressureThreadMain(&sensorsPressureThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(sensorsAirFlowThreadMain(&sensorsAirFlowThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(sensorsBatteryThreadMain(&sensorsBatteryThread))) {
    PT_EXIT(pt);
  }

  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN)^1);
  Serial.print(sensors.currentPressure);
  Serial.print("    ");
  Serial.print(sensors.currentFlow);
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
