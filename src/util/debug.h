
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include <HardwareSerial.h>

#ifndef DEBUG_MODULE
#error "Cannot include debug.h without defining DEBUG_MODULE"
#endif

#ifndef DEBUG_BUFFER_SIZE
#define DEBUG_BUFFER_SIZE 128
#endif

#ifndef DEBUG_SERIAL_PORT
#define DEBUG_SERIAL_PORT Serial
#endif

// In order to enable the standard Arduino Serial Plotter, all other types of
// messages must be disabled and only a single module can plot a single set
// of variables at one time. As such, to use the DEBUG_PLOT macro, define
// DEBUG_PLOTTING here with the module name (DEBUG_MODULE) to be plotted,
// example commented out below:
// #define DEBUG_PLOTTING "control"

// Initial setup of debug; only to be used once
#define DEBUG_BEGIN DEBUG_SERIAL_PORT.begin(230400)

#ifdef DEBUG
#ifdef DEBUG_PLOTTING

// Use DEBUG_PLOT to plot a set of variables, can only be used once in a module
// and requires DEBUG_PLOTTING to be defined and set the correct module's name,
// see above
// NOTE: Compiler seems pretty good at optimizing out the strcmp, would like a
//       better way of doing this but the rest of the mechanism is so clean, I
//       am relying on the compiler to do the optimization rather than via some
//       macros somehow...
#define DEBUG_PLOT(...)                                                 \
  do {                                                                  \
    static bool header = true;                                          \
    if (strcmp(DEBUG_PLOTTING, DEBUG_MODULE) == 0) {                    \
      if (header) {                                                     \
        DEBUG_SERIAL_PORT.println(#__VA_ARGS__);                        \
        header = false;                                                 \
      }                                                                 \
      int32_t _vars[] = { __VA_ARGS__ };                                \
      for (int i = 0; i < sizeof(_vars) / sizeof(int32_t); i++) {       \
        if (i > 0) DEBUG_SERIAL_PORT.print(",");                        \
        DEBUG_SERIAL_PORT.print(_vars[i]);                              \
      }                                                                 \
      DEBUG_SERIAL_PORT.print("\n");                                    \
    }                                                                   \
  } while (0);                                                          \

#define DEBUG_PRINT(...)
#define DEBUG_PRINT_EVERY(...)

#else

#define DEBUG_PLOT(...)

// Use DEBUG_PRINT to add print messages like printf
#define DEBUG_PRINT(...)                                                \
  do {                                                                  \
    char _buf[DEBUG_BUFFER_SIZE];                                       \
    snprintf(_buf, DEBUG_BUFFER_SIZE, __VA_ARGS__);                     \
    DEBUG_SERIAL_PORT.print("[" DEBUG_MODULE "] ");                     \
    DEBUG_SERIAL_PORT.println(_buf);                                    \
  } while (0);

// Use DEBUG_PRINT_EVERY to add print messages line printf, but it will only
// print the message every i times the line is reached (note: it costs 2 bytes
// to remember this per line)
#define DEBUG_PRINT_EVERY(i, ...)                                       \
  do {                                                                  \
    static unsigned int _count = (i - __LINE__) % i;                    \
    if (i - (++_count) == 0) {                                          \
      DEBUG_PRINT(__VA_ARGS__);                                         \
      _count = 0;                                                       \
    }                                                                   \
  } while (0);

#endif
#else
#define DEBUG_PLOT(...)
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_EVERY(...)
#endif