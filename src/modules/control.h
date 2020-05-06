
#ifndef __CONTROL_MODULE_H__
#define __CONTROL_MODULE_H__

#include <stdint.h>

#include "../util/alarm.h"

// Control States
// TODO: Consider different encoding?
// TODO: Hold out states? error state?

#define CONTROL_HOME              0
#define CONTROL_IDLE              1
#define CONTROL_BEGIN_INHALATION  2
#define CONTROL_INHALATION        3
#define CONTROL_BEGIN_HOLD_IN     4
#define CONTROL_HOLD_IN           5
#define CONTROL_BEGIN_EXHALATION  6
#define CONTROL_EXHALATION        7


struct control {
  // Variables
  uint8_t  state;
  uint32_t respirationRateMeasured;
  uint32_t ieRatioMeasured;
  uint32_t breathCount;
  
  // Alarms
  struct alarm unknownStateAlarm;
};

// Public Variables
extern struct control control;

// TODO: Doc
int controlModuleInit(void);

// TODO: Doc
int controlModuleRun(void);

#endif /* __CONTROL_MODULE_H__ */