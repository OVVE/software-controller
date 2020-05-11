
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "../hal/serial.h"
#include "../config.h"

#ifndef LOG_MODULE
#error "Cannot include log.h without defining LOG_MODULE"
#endif

#define LOG_BUFFER_SIZE 96
#ifdef ENABLE_LOGGING
  
  #define LOG_DATA(logId, logPtr, logSize)                          \
    do {                                                              \
        serialHalSendPacket(logId,logSize,logPtr);                      \
    } while (0);                                                      \

  #define LOG_PRINT(...)                                            \
    do {                                                              \
        char _buf[LOG_BUFFER_SIZE];                                     \
        snprintf(_buf, LOG_BUFFER_SIZE, "[" LOG_MODULE "] " __VA_ARGS__);                   \
        serialHalSendPacket(LOG_PACKET_TEXT,strlen(_buf),&_buf[0]);     \
    } while (0);                                                      \
  
  #define LOG_PRINT_EVERY(i, ...)                                       \
  do {                                                                  \
    static unsigned int _count = (i - __LINE__) % i;                    \
    if (i - (++_count) == 0) {                                          \
      LOG_PRINT(__VA_ARGS__);                                         \
      _count = 0;                                                       \
    }                                                                   \
  } while (0);
#else
#define LOG_DATA(...)
#define LOG_PRINT(...)
#define LOG_PRINT_EVERY(...)
#endif