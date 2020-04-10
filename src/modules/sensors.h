
#ifndef __SENSORS_MODULE_H__
#define __SENSORS_MODULE_H__

#include <stdint.h>
#include <stdbool.h>

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
  bool    inhalationDetected;
  bool    exhalationDetected;
  
  // Alarms
  int8_t  onBatteryAlarm;
  int8_t  lowBatteryAlarm;
  int8_t  highPressureAlarm;
  int8_t  lowPressureAlarm;
  int8_t  lowVolumeAlarm;
  int8_t  apneaAlarm;
};
// Public Variables
extern struct sensors sensors;

// TODO: Doc
int16_t sensorsModuleInit(void);

// TODO: Doc
int16_t sensorsModuleRun(void);

#endif /* __SENSORS_MODULE_H__ */