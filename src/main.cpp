
#include "config.h"

#include "hal/alarm.h"
#include "hal/estop.h"
#include "hal/timer.h"
#include "hal/watchdog.h"

#include "modules/link.h"
#include "modules/module.h"
#include "modules/sensors.h"
#include "modules/control.h"
#include "modules/parameters.h"

#include "util/alarm.h"
#include "util/metrics.h"

#define LOG_MODULE "main"
#define LOG_LEVEL  LOG_MAIN_LOOP
#include "util/log.h"

#if LOG_MAIN_LOOP >= DEBUG
#define DEBUG_MAIN_LOOP_METRICS
#endif

#ifdef DEBUG_MAIN_LOOP_METRICS
static struct metrics mainLoopMetrics;
static struct metrics scheduleMetrics;
static struct metrics controlModuleMetrics;
static struct metrics sensorsModuleMetrics;
static struct metrics parametersModuleMetrics;
static struct metrics linkModuleMetrics;
#endif

static struct alarmProperties estopAlarmProperties = {
  .priority = ALARM_PRIORITY_SEVERE,
  .preventWatchdog = false,
  .suppressionTimeout = (100 MSEC),
};

static struct alarm estopAlarm;

void mainSetup(void)
{ 
  // Initialize HAL
  // TODO: Handle failure conditions
  // TODO: refactor all HAL initiailization functions here, since more than one
  //       module might be using a single HAL (like timer)

  timerHalInit();
  alarmHalInit();
  estopHalInit();
  serialHalInit();
  
  LOG_PRINT(INFO, "Initialization started, loading modules...");

  // TODO: Handle failure conditions
  controlModuleInit();
  sensorsModuleInit();
  parametersModuleInit();
  linkModuleInit();
  
  alarmInit(&estopAlarm, &estopAlarmProperties);

#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsReset(&mainLoopMetrics);
  metricsReset(&scheduleMetrics);
  metricsReset(&controlModuleMetrics);
  metricsReset(&sensorsModuleMetrics);
  metricsReset(&parametersModuleMetrics);
  metricsReset(&linkModuleMetrics);
#endif

  // TODO: For now, move the watchdog initialization here; this will need to go
  //       back to the top of setup to allow the watchdog to prevent HAL/module
  //       initialization from hanging, but theres a bunch of stuff that needs
  //       to be refactored to allow this right now (namely, the motor HAL)
  watchdogHalInit();
  LOG_PRINT(INFO, "Initialization complete!");
}

void mainLoop(void)
{
#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStart(&mainLoopMetrics);
  metricsStart(&scheduleMetrics);
#endif

  // Run all modules in RR; take specified actions in the event of failure
#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStart(&controlModuleMetrics);
#endif
  if (controlModuleRun() != MODULE_OK) {
    // TODO: control module exited, trigger severe error
  }
#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStop(&controlModuleMetrics);
#endif
  
#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStart(&sensorsModuleMetrics);
#endif
  if (sensorsModuleRun() != MODULE_OK) {
    // TODO: sensor module exited, trigger severe error  
  }
#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStop(&sensorsModuleMetrics);
#endif

#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStart(&parametersModuleMetrics);
#endif
  if (parametersModuleRun() != MODULE_OK) {
    // TODO: parameters module exited, attempt recovery
  }
#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStop(&parametersModuleMetrics);
#endif

#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStart(&linkModuleMetrics);
#endif
  if (linkModuleRun() != MODULE_OK) {
    // TODO: link module exited, attempt recovery
  }
#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStop(&linkModuleMetrics);
#endif

  // Check the Estop, asserting an alarm if active
  if (estopHalAsserted()) {
    alarmSet(&estopAlarm);
  } else {
    alarmSuppress(&estopAlarm);
  }

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

#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsStop(&scheduleMetrics);
  LOG_PRINT_EVERY(25000, DEBUG, "Runtime Metrics:");
  LOG_PRINT_EVERY(25000, DEBUG, " Main loop:  %10lu us : %10lu us : %10lu us",
                  mainLoopMetrics.minimum,
                  mainLoopMetrics.average,
                  mainLoopMetrics.maximum);
  LOG_PRINT_EVERY(25000, DEBUG, " Scheduler:  %10lu us : %10lu us : %10lu us",
                  scheduleMetrics.minimum,
                  scheduleMetrics.average,
                  scheduleMetrics.maximum);
  LOG_PRINT_EVERY(25000, DEBUG, " Control:    %10lu us : %10lu us : %10lu us",
                  controlModuleMetrics.minimum,
                  controlModuleMetrics.average,
                  controlModuleMetrics.maximum);
  LOG_PRINT_EVERY(25000, DEBUG, " Sensors:    %10lu us : %10lu us : %10lu us",
                  sensorsModuleMetrics.minimum,
                  sensorsModuleMetrics.average,
                  sensorsModuleMetrics.maximum);
  LOG_PRINT_EVERY(25000, DEBUG, " Parameters: %10lu us : %10lu us : %10lu us",
                  parametersModuleMetrics.minimum,
                  parametersModuleMetrics.average,
                  parametersModuleMetrics.maximum);
  LOG_PRINT_EVERY(25000, DEBUG, " Link:       %10lu us : %10lu us : %10lu us",
                  linkModuleMetrics.minimum,
                  linkModuleMetrics.average,
                  linkModuleMetrics.maximum);
  metricsStop(&mainLoopMetrics);
#endif
}