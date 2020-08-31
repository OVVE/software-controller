/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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