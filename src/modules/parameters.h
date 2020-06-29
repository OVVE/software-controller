
#ifndef __PARAMETERS_MODULE_H__
#define __PARAMETERS_MODULE_H__

#include <stdbool.h>

#include "../util/alarm.h"

#define VENTILATOR_MODE_VC   0x01
#define VENTILATOR_MODE_AC   0x02
#define VENTILATOR_MODE_SIMV 0x03

struct parameters {
  // Variables
  uint8_t  startVentilation;
  uint8_t  calibrationStep;
  uint8_t  ventilationMode;
  int16_t  volumeRequested;
  int16_t  pressureRequested;
  uint16_t respirationRateRequested;
  uint16_t ieRatioRequested;
  int16_t  highVolumeLimit;
  int16_t  lowVolumeLimit;
  int16_t  highPressureLimit;
  int16_t  lowPressureLimit;
  uint16_t highRespiratoryRateLimit;
  uint16_t lowRespiratoryRateLimit;
  
  // Alarms
  struct alarm parametersInvalidAlarm;
};

// Public Variables
extern struct parameters parameters;

// TODO: Doc
int parametersModuleInit(void);

// TODO: Doc
int parametersModuleRun(void);

#endif /* __PARAMETERS_MODULE_H__ */