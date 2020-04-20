
#include "hal/alarm.h"
#include "hal/timer.h"
#include "hal/watchdog.h"

#include "modules/link.h"
#include "modules/module.h"
#include "modules/sensors.h"
#include "modules/control.h"
#include "modules/parameters.h"

#include "util/alarm.h"
#include "util/metrics.h"

#define DEBUG
#define DEBUG_MODULE "main"
#include "util/debug.h"

#define MAIN_METRICS

#ifdef MAIN_METRICS
static struct metrics mainLoopMetrics;
static struct metrics scheduleMetrics;
static struct metrics controlModuleMetrics;
static struct metrics sensorsModuleMetrics;
static struct metrics parametersModuleMetrics;
static struct metrics linkModuleMetrics;
#endif

void mainSetup(void)
{
  DEBUG_BEGIN;
  
  // TODO: cleanup prints
  DEBUG_PRINT("hal setup");
  
  // Initialize HAL
  // TODO: Handle failure conditions
  // TODO: refactor all HAL initiailization functions here, since more than one
  //       module might be using a single HAL (like timer)
  watchdogHalInit();

  timerHalInit();
  alarmHalInit();
  
  DEBUG_PRINT("module setup");

  // TODO: Handle failure conditions
  controlModuleInit();
  sensorsModuleInit();
  parametersModuleInit();
  linkModuleInit();

#ifdef MAIN_METRICS
  metricsReset(&mainLoopMetrics);
  metricsReset(&scheduleMetrics);
  metricsReset(&controlModuleMetrics);
  metricsReset(&sensorsModuleMetrics);
  metricsReset(&parametersModuleMetrics);
  metricsReset(&linkModuleMetrics);
#endif
  DEBUG_PRINT("setup done");
}

void mainLoop(void)
{
#ifdef MAIN_METRICS
  metricsStart(&mainLoopMetrics);
  metricsStart(&scheduleMetrics);
#endif

  // Run all modules in RR; take specified actions in the event of failure
#ifdef MAIN_METRICS
  metricsStart(&controlModuleMetrics);
#endif
  if (controlModuleRun() != MODULE_OK) {
    // TODO: control module exited, trigger severe error
  }
#ifdef MAIN_METRICS
  metricsStop(&controlModuleMetrics);
#endif
  
#ifdef MAIN_METRICS
  metricsStart(&sensorsModuleMetrics);
#endif
  if (sensorsModuleRun() != MODULE_OK) {
    // TODO: sensor module exited, trigger severe error  
  }
#ifdef MAIN_METRICS
  metricsStop(&sensorsModuleMetrics);
#endif

#ifdef MAIN_METRICS
  metricsStart(&parametersModuleMetrics);
#endif
  if (parametersModuleRun() != MODULE_OK) {
    // TODO: parameters module exited, attempt recovery
  }
#ifdef MAIN_METRICS
  metricsStop(&parametersModuleMetrics);
#endif

#ifdef MAIN_METRICS
  metricsStart(&linkModuleMetrics);
#endif
  if (linkModuleRun() != MODULE_OK) {
    // TODO: link module exited, attempt recovery
  }
#ifdef MAIN_METRICS
  metricsStop(&linkModuleMetrics);
#endif

  // Scan through all alarms, looking for any set warning that need to be addressed
  struct alarmProperties properties = {0};
  if (alarmCheckAll(&properties)) {
    // At least one alarm has fired, determine correct actions
    if (!properties.preventWatchdog) {
      watchdogHalReset();
    }
    
    switch (properties.priority) {
      case ALARM_PRIORITY_SEVERE:
      alarmHalRing(ALARM_HAL_CONSTANT);
      break;
      case ALARM_PRIORITY_HIGH:
      alarmHalRing(ALARM_HAL_1HZ);
      break;
      case ALARM_PRIORITY_MODERATE:
      alarmHalRing(ALARM_HAL_0_25HZ);
      break;
      case ALARM_PRIORITY_LOW:
      alarmHalRing(ALARM_HAL_OFF);
      break;
    }
  } else {
    watchdogHalReset();
    alarmHalRing(ALARM_HAL_OFF);
  }

#ifdef MAIN_METRICS
  metricsStop(&scheduleMetrics);
  DEBUG_PRINT_EVERY(25000, "Runtime Metrics:");
  DEBUG_PRINT_EVERY(25000, " Main loop:  %10lu us : %10lu us : %10lu us",
                    mainLoopMetrics.minimum,
                    mainLoopMetrics.average,
                    mainLoopMetrics.maximum);
  DEBUG_PRINT_EVERY(25000, " Scheduler:  %10lu us : %10lu us : %10lu us",
                    scheduleMetrics.minimum,
                    scheduleMetrics.average,
                    scheduleMetrics.maximum);
  DEBUG_PRINT_EVERY(25000, " Control:    %10lu us : %10lu us : %10lu us",
                    controlModuleMetrics.minimum,
                    controlModuleMetrics.average,
                    controlModuleMetrics.maximum);
  DEBUG_PRINT_EVERY(25000, " Sensors:    %10lu us : %10lu us : %10lu us",
                    sensorsModuleMetrics.minimum,
                    sensorsModuleMetrics.average,
                    sensorsModuleMetrics.maximum);
  DEBUG_PRINT_EVERY(25000, " Parameters: %10lu us : %10lu us : %10lu us",
                    parametersModuleMetrics.minimum,
                    parametersModuleMetrics.average,
                    parametersModuleMetrics.maximum);
  DEBUG_PRINT_EVERY(25000, " Link:       %10lu us : %10lu us : %10lu us",
                    linkModuleMetrics.minimum,
                    linkModuleMetrics.average,
                    linkModuleMetrics.maximum);
  metricsStop(&mainLoopMetrics);
#endif
}