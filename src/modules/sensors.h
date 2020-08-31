/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __SENSORS_MODULE_H__
#define __SENSORS_MODULE_H__

#include <stdint.h>
#include <stdbool.h>

#include "../util/alarm.h"

#define SENSORS_AIRFLOW_CALIBRATED  0x01
#define SENSORS_PRESSURE_CALIBRATED 0x02
#define SENSORS_ALL_CALIBRATED (SENSORS_AIRFLOW_CALIBRATED | SENSORS_PRESSURE_CALIBRATED)

// TODO: Units?
struct sensors {
  // Variables
  int32_t currentFlow;
  int32_t currentVolume;
  int32_t volumeIn;
  int32_t volumeOut;
  int32_t minuteVolume;
  int32_t currentPressure;
  int32_t averagePressure;
  int32_t peakPressure;
  int32_t plateauPressure;
  int32_t peepPressure;
  bool    inhalationDetected;
  bool    exhalationDetected;
  uint8_t batteryPercent;
  bool    batteryCharging;
  
  uint8_t calibrated;
  
  // Alarms
  struct alarm onBatteryAlarm;
  struct alarm lowBatteryAlarm;
  struct alarm badPressureSensorAlarm;
  struct alarm badAirflowSensorAlarm;
  struct alarm highPressureAlarm;
  struct alarm lowPressureAlarm;
  struct alarm continuousPressureAlarm;
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