
#ifndef __SYS_HAL_H__
#define __SYS_HAL_H__

#include <stdint.h>

#include <Arduino.h>

static inline uint32_t sysHalTime(void)
{
  return (uint32_t) millis();
}

#endif /* __SYS_HAL_H__ */