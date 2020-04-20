
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
  int8_t   priority;
  bool     preventWatchdog;
  uint32_t suppressionTimeout;
};

struct alarm {
  bool                          set;
  bool                          suppressed;
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