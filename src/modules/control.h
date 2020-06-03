
#ifndef __CONTROL_MODULE_H__
#define __CONTROL_MODULE_H__

#include <stdint.h>

#include "../util/alarm.h"

// Control States
// TODO: Consider different encoding?
// TODO: Hold out states? error state?

#define CONTROL_STATE_IDLE             0x00
#define CONTROL_STATE_BEGIN_INHALATION 0x01
#define CONTROL_STATE_INHALATION       0x02
#define CONTROL_STATE_BEGIN_HOLD_IN    0x03
#define CONTROL_STATE_HOLD_IN          0x04
#define CONTROL_STATE_BEGIN_EXHALATION 0x05
#define CONTROL_STATE_EXHALATION       0x06
#define CONTROL_STATE_HOME             0x07

struct control {
  // Variables
  uint8_t  state;
  uint32_t respirationRateMeasured;
  uint32_t ieRatioMeasured;
  uint32_t breathCount;
  float outputFiltered;
  // Alarms
  int8_t   breathTimeoutAlarm;
  int8_t   unknownStateAlarm;

  uint8_t allowPatientSync;
};

// Public Variables
extern struct control control;

// TODO: Doc
int controlModuleInit(void);

// TODO: Doc
int controlModuleRun(void);

#endif /* __CONTROL_MODULE_H__ */