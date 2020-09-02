/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef __CONTROL_MODULE_H__
#define __CONTROL_MODULE_H__

#include <stdint.h>

#include "../util/alarm.h"

// Control States
// TODO: Consider different encoding?
// TODO: Hold out states? error state?

#define CONTROL_STATE_UNCALIBRATED            0x00
#define CONTROL_STATE_IDLE                    0x01
#define CONTROL_STATE_BEGIN_INHALATION        0x02
#define CONTROL_STATE_INHALATION              0x03
#define CONTROL_STATE_BEGIN_HOLD_IN           0x04
#define CONTROL_STATE_HOLD_IN                 0x05
#define CONTROL_STATE_BEGIN_EXHALATION        0x06
#define CONTROL_STATE_EXHALATION              0x07
#define CONTROL_STATE_HOME                    0x08
#define CONTROL_STATE_HALT                    0x09
#define CONTROL_STATE_SENSOR_CALIBRATION      0x0a
#define CONTROL_STATE_SENSOR_CALIBRATION_DONE 0x0b

struct control {
  // Variables
  uint8_t  state;
  uint32_t respirationRateMeasured;
  uint32_t ieRatioMeasured;
  uint32_t breathCount;
  
  // Alarms
  int8_t   breathTimeoutAlarm;
  int8_t   unknownStateAlarm;
};

// Public Variables
extern struct control control;

// TODO: Doc
int controlModuleInit(void);

// TODO: Doc
int controlModuleRun(void);

#endif /* __CONTROL_MODULE_H__ */