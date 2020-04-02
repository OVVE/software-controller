
#ifndef __PARAMETERS_MODULE_H__
#define __PARAMETERS_MODULE_H__

#include <stdbool.h>

#define VENTILATOR_MODE_VC   0x01
#define VENTILATOR_MODE_AC   0x02
#define VENTILATOR_MODE_SIMV 0x03

struct parameters {
  // Variables
  uint8_t  startVentilation;
  uint8_t  ventilationMode;
  uint32_t volumeRequested;
  uint32_t respirationRateRequested;
  uint32_t ieRatioRequested;
  
  // Alarms
  int8_t   parametersInvalidAlarm;
};

// Public Variables
extern struct parameters parameters;

// TODO: Doc
int parametersModuleInit(void);

// TODO: Doc
int parametersModuleRun(void);

#endif /* __PARAMETERS_MODULE_H__ */