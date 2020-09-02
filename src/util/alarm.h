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

#ifndef __ALARM_UTIL_H__
#define __ALARM_UTIL_H__

#include <stdint.h>
#include <stdbool.h>

#include "../hal/timer.h"

#define ALARM_PRIORITY_SEVERE   0
#define ALARM_PRIORITY_HIGH     1
#define ALARM_PRIORITY_MODERATE 2
#define ALARM_PRIORITY_LOW      3

struct alarmProperties {
  int8_t   priority : 4;
  bool     preventWatchdog : 1;
  uint32_t suppressionTimeout;
};

struct alarm {
  bool                          set : 1;
  bool                          suppressed : 1;
  const struct alarmProperties* properties;
  struct timer                  suppressionTimer;
  struct alarm*                 next;
};

// TODO: Doc
void alarmInit(struct alarm* alarm, const struct alarmProperties* properties);

// TODO: Doc
void alarmSet(struct alarm* alarm);

// TODO: Doc
bool alarmGet(struct alarm* alarm);

// TODO: Doc
void alarmSuppress(struct alarm* alarm);

// TODO: Doc
bool alarmCheckAll(struct alarmProperties* properties);

#endif /* __ALARM_UTIL_H__ */