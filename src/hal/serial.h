// TODO: Document what this module does

#ifndef __SERIAL_HAL_H__
#define __SERIAL_HAL_H__

#include <stddef.h>
#include <stdint.h>

#include "../hal/hal.h"
#include "../hal/timer.h"

#include <HardwareSerial.h>

#define SERIAL_PORT_UI          &serialHalPortUI
#define SERIAL_PORT_UI_PORT     Serial1
#define SERIAL_PORT_UI_BAUD     (250000)

#define SERIAL_PORT_DEBUG       &serialHalPortDebug
#define SERIAL_PORT_DEBUG_PORT  Serial
#define SERIAL_PORT_DEBUG_BAUD  (250000)

struct serialHalPort {
  HardwareSerial* port;
  struct timer    readTimer;
  struct timer    writeTimer;
  size_t          readBytesLeft;
  size_t          writeBytesLeft;
};

extern struct serialHalPort serialHalPortUI;
extern struct serialHalPort serialHalPortDebug;

int serialHalInit(void);

int serialHalWrite(struct serialHalPort* port, void* data, size_t size, uint32_t timeout);

int serialHalRead(struct serialHalPort* port, void* data, size_t size, uint32_t timeout);

#endif /* __SERIAL_HAL_H__ */