
#ifndef __SENSORS_MODULE_H__
#define __SENSORS_MODULE_H__

#include <stdint.h>
#include <stdbool.h>

#include "../util/alarm.h"

// TODO: Units?
struct sensors {
  // Variables
  int32_t currentFlow;
  int32_t currentVolume;
  int32_t volumeIn;
  int32_t volumeOut;
  int32_t volumePerMinute;
  int32_t currentPressure;
  int32_t peakPressure;
  int32_t plateauPressure;
  int32_t peepPressure;
  int32_t virtualInhalationSensor;
  bool    inhalationDetected;
  bool    exhalationDetected;
  
  // Alarms
  struct alarm onBatteryAlarm;
  struct alarm lowBatteryAlarm;
  struct alarm badPressureSensorAlarm;
  struct alarm badAirflowSensorAlarm;
  struct alarm highPressureAlarm;
  struct alarm lowPressureAlarm;
  struct alarm highVolumeAlarm;
  struct alarm lowVolumeAlarm;
  struct alarm highRespiratoryRateAlarm;
  struct alarm lowRespiratoryRateAlarm;
};
// Public Variables
extern struct sensors sensors;

// TODO: Doc
int sensorsModuleInit(void);

// TODO: Doc
int sensorsModuleRun(void);

#endif /* __SENSORS_MODULE_H__ */