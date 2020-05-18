


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include <avr/pgmspace.h>

#include "../util/utils.h"

#ifdef LOG_BACKEND_DEBUG
#include "../hal/serial.h"
#endif

#ifdef LOG_BACKEND_LINK
#include "../modules/link.h"

#include "../util/queue.h"
#endif

#ifndef LOG_MODULE
#error "Cannot include log.h without defining LOG_MODULE"
#endif

#ifndef LOG_LEVEL
#error "Cannot include log.h without defining LOG_LEVEL"
#endif

#ifndef LOG_BUFFER_SIZE
#define LOG_BUFFER_SIZE 128
#endif

// Log Levels
#define NOLOG   0
#define ERROR   1
#define WARNING 2
#define INFO    3
#define DEBUG   4
#define VERBOSE 5

#ifdef LOG_BACKEND_DEBUG
#define LOG_BACKEND_DEBUG_PORT SERIAL_PORT_DEBUG

// Handles actually printing to the backend
#define BACKEND_DEBUG(lid, buf, sz)                                            \
  do {                                                                         \
    while (serialHalWrite(LOG_BACKEND_DEBUG_PORT,                              \
                          buf, sz, HAL_FOREVER) == HAL_IN_PROGRESS);           \
  } while (0);
#else
#define BACKEND_DEBUG(...)
#endif

#ifdef LOG_BACKEND_LINK
#define BACKEND_LINK(lid, buf, sz)                                             \
  do {                                                                         \
    uint8_t _queueSize = comm.logQueue.maxSize - queueSize(&comm.logQueue);    \
    struct logMessage _msg;                                                    \
    uint8_t _tsz = sizeof(_msg.header) + sz;                                   \
    if (_queueSize <= _tsz) {                                                  \
      _msg.header.size = min(sz, MAX_LOG_DATA_SIZE);                           \
      _msg.header.id = lid;                                                    \
      for (int _i = 0; _i < sizeof(_msg.header); _i++) {                       \
        queuePush(&comm.logQueue, &(_msg.header.raw[_i]));                     \
      }                                                                        \
      for (int _i = 0; _i < _msg.header.size; _i++) {                          \
        queuePush(&comm.logQueue, &(buf[_i]));                                 \
      }                                                                        \
      comm.loggingSentMessages++;                                              \
    } else {                                                                   \
      comm.loggingDroppedMessages++;                                           \
    }                                                                          \
  } while (0);
#else
#define BACKEND_LINK(...)
#endif

#ifdef LOG
#ifdef LOG_PLOTTING

// TODO: Leave in link backend, only applies to debug backend, so clean this up

// Use LOG_PLOT to plot a set of variables, can only be used once in a module
// and requires LOG_PLOTTING to be defined and set the correct module's name,
// see above
// NOTE: Compiler seems pretty good at optimizing out the strcmp, would like a
//       better way of doing this but the rest of the mechanism is so clean, I
//       am relying on the compiler to do the optimization rather than via some
//       macros somehow...
#define LOG_PLOT(...)                                                 \
  do {                                                                  \
    static bool header = true;                                          \
    if (strcmp(LOG_PLOTTING, LOG_MODULE) == 0) {                    \
      if (header) {                                                     \
        LOG_SERIAL_PORT.println(#__VA_ARGS__);                        \
        header = false;                                                 \
      }                                                                 \
      int32_t _vars[] = { __VA_ARGS__ };                                \
      for (int i = 0; i < sizeof(_vars) / sizeof(int32_t); i++) {       \
        if (i > 0) LOG_SERIAL_PORT.print(",");                        \
        LOG_SERIAL_PORT.print(_vars[i]);                              \
      }                                                                 \
      LOG_SERIAL_PORT.println("");                                    \
    }                                                                   \
  } while (0);                                                          \

#define LOG_DATA(...)
#define LOG_PRINT(...)
#define LOG_PRINT_EVERY(...)

#else

#define LOG_PLOT(...)

// Use LOG_DATA to send raw data to the link backend only (if enabled)
#define LOG_DATA(lvl, id, buf, sz)                                             \
  do {                                                                         \
    if (lvl <= LOG_LEVEL) {                                                    \
      uint8_t* _buf = buf;                                                     \
      BACKEND_LINK(id, _buf, sz);                                              \
    }                                                                          \
  } while (0);

// Use LOG_PRINT to add print messages like printf
// NOTE: Currently uses non-portable GNU C/CPP extension ## to eliminate trailing
//       comma; other techniques might be looked at in the future
#define LOG_PRINT(lvl, fmt, ...)                                               \
  do {                                                                         \
    if (lvl <= LOG_LEVEL) {                                                    \
      char _buf[LOG_BUFFER_SIZE];                                              \
      char* PROGMEM _fmt = PSTR("[" LOG_MODULE "] " fmt "\n");                 \
      int _sz = snprintf_P(_buf, LOG_BUFFER_SIZE, _fmt, ## __VA_ARGS__);       \
      BACKEND_DEBUG(LOG_MESSAGE_ID, _buf, min(_sz, sizeof(_buf)));             \
      BACKEND_LINK(LOG_MESSAGE_ID, _buf, min(_sz, sizeof(_buf)));              \
    }                                                                          \
  } while (0);

// Use LOG_PRINT_EVERY to add print messages line printf, but it will only
// print the message every i times the line is reached (note: it costs 2 bytes
// to remember this per line)
#define LOG_PRINT_EVERY(i, lvl, ...)                                           \
  do {                                                                         \
    if (lvl <= LOG_LEVEL) {                                                    \
      static unsigned int _count = (i - __LINE__) % i;                         \
      if (i - (++_count) == 0) {                                               \
        LOG_PRINT(lvl, __VA_ARGS__);                                           \
        _count = 0;                                                            \
      }                                                                        \
    }                                                                          \
  } while (0);

#endif
#else
#define LOG_PLOT(...)
#define LOG_DATA(...)
#define LOG_PRINT(...)
#define LOG_PRINT_EVERY(...)
#endif
