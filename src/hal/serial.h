// TODO: Document what this module does

#ifndef __SERIAL_HAL_H__
#define __SERIAL_HAL_H__
#include <Arduino.h>
#include "../hal/hal.h"

#define BAUD_RATE 38400
#define WATCHDOG_MS 1000  // maximum time to receive all bytes from command packet. if time expires trash data and send next packet.
#define SEND_INTERVAL_MS 100 // minimum time between sending packets. The time could take longer if response is late.
#define SEND_MAX_TIME_MS 30  // maximum time to allow for sending data packet. This is used with sending data in parts if send buffer full.
#define USE_AVAILABLE_WRITE  // #ifdef in serial.cpp for checking the available bytes on write queue before sending to keep from blocking

#define SERIAL_UI Serial1
#define SERIAL_DEBUG Serial

int serialHalInit(void);
int serialHalGetData(void);
int serialHalSendData();

#endif /* __SERIAL_HAL_H__ */