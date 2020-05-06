
#ifndef __ALARM_HAL_H__
#define __ALARM_HAL_H__

#include "../hal/hal.h"

#define ALARM_PIN          3

#define ALARM_ASSERT_OFF   0
#define ALARM_ASSERT_ON    1

// Various alarm settings
#define ALARM_HAL_OFF       0
#define ALARM_HAL_0_25HZ    1
#define ALARM_HAL_0_5HZ     2
#define ALARM_HAL_1HZ       3
#define ALARM_HAL_CONSTANT  4

// TODO: Doc
int alarmHalInit(void);

// TODO: Doc
int alarmHalRing(unsigned int pattern);

#endif /* __ALARM_HAL_H__ */