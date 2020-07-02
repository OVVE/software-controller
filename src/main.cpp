
#include <stdbool.h>

#include "config.h"

#include "hal/i2c.h"
#include "hal/sys.h"
#include "hal/alarm.h"
#include "hal/estop.h"
#include "hal/timer.h"
#include "hal/watchdog.h"
#include "hal/sensor/battery.h"

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

#define POWEROFF_TIMEOUT    (10 SEC)
#define POWEROFF_BEEP_TIME  (500 MSEC)
#define POWEROFF_DELAY_TIME (5 SEC)
#define POWERON_BEEP_TIME   (250 MSEC)

#define POWEROFF_STEP0  0
#define POWEROFF_STEP1  1
#define POWEROFF_STEP2  2
#define POWEROFF_STEP3  3
#define POWEROFF_STEP4  4

#define POWERON_STEP0  0
#define POWERON_STEP1  1
#define POWERON_STEP2  2

#ifdef DEBUG_MAIN_LOOP_METRICS
static struct metrics mainLoopMetrics;
static struct metrics scheduleMetrics;
static struct metrics controlModuleMetrics;
static struct metrics sensorsModuleMetrics;
static struct metrics parametersModuleMetrics;
static struct metrics linkModuleMetrics;
#endif

static struct timer powerTimer;
static uint8_t powerOffStep = POWEROFF_STEP0;
static uint8_t powerOnStep = POWERON_STEP1;
bool powerOff = false;

static struct alarmProperties estopAlarmProperties = {
  .priority = ALARM_PRIORITY_SEVERE,
  .preventWatchdog = false,
  .suppressionTimeout = (100 MSEC),
};
static struct alarmProperties powerOnAlarmProperties = {
  .priority = ALARM_PRIORITY_SEVERE,
  .preventWatchdog = false,
  .suppressionTimeout = (100 MSEC),
};
static struct alarmProperties powerOffAlarmProperties = {
  .priority = ALARM_PRIORITY_SEVERE,
  .preventWatchdog = false,
  .suppressionTimeout = (100 MSEC),
};

struct alarm estopAlarm;
struct alarm powerOnAlarm;
struct alarm powerOffAlarm;

void mainSetup(void)
{ 
  // Initialize HAL
  // TODO: Handle failure conditions
  // TODO: refactor all HAL initiailization functions here, since more than one
  //       module might be using a single HAL (like timer)

  watchdogHalInit();
  timerHalInit();
  alarmHalInit();
  estopHalInit();
  i2cHalInit();
  serialHalInit();

  LOG_PRINT(INFO, "Initialization started, loading modules...");

  // TODO: Handle failure conditions
  controlModuleInit();
  sensorsModuleInit();
  parametersModuleInit();
  linkModuleInit();
  
  alarmInit(&estopAlarm, &estopAlarmProperties);
  alarmInit(&powerOnAlarm, &powerOnAlarmProperties);
  alarmInit(&powerOffAlarm, &powerOffAlarmProperties);

#ifdef DEBUG_MAIN_LOOP_METRICS
  metricsReset(&mainLoopMetrics);
  metricsReset(&scheduleMetrics);
  metricsReset(&controlModuleMetrics);
  metricsReset(&sensorsModuleMetrics);
  metricsReset(&parametersModuleMetrics);
  metricsReset(&linkModuleMetrics);
#endif
  
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
    if (!alarmGet(&estopAlarm)) {
      LOG_PRINT(ERROR, "ESTOP Asserted!");
    }
    alarmSet(&estopAlarm);
  } else if (alarmGet(&estopAlarm)) {
    LOG_PRINT(INFO, "ESTOP Deasserted, suppressing alarm");
    alarmSuppress(&estopAlarm);
  }
  
  // Handle power on
  switch (powerOnStep) {
    case POWERON_STEP0:
      alarmSet(&powerOnAlarm);
      timerHalBegin(&powerTimer, POWERON_BEEP_TIME, false);
      powerOnStep = POWERON_STEP1;
    case POWERON_STEP1:
      if (timerHalRun(&powerTimer) != HAL_IN_PROGRESS) {
        alarmSuppress(&powerOnAlarm);
        powerOnStep = POWERON_STEP2;
      }
      break;
  }
  
  // Handle power off
  if (parameters.startVentilation) {
    if (powerOffStep != POWEROFF_STEP0 || comm.powerOff) {
      LOG_PRINT_EVERY(500, ERROR, "Continuing/starting to ventilate during power off!");
    } else if (sysHalPowerButtonAsserted()) {
      LOG_PRINT_EVERY(500, WARNING, "Power button hit when ventilating, ignoring...");
    }
  } else {
    switch (powerOffStep) {
      case POWEROFF_STEP0:
        if (comm.powerOff) {
          powerOffStep = POWEROFF_STEP2;
        } if (sysHalPowerButtonAsserted()) {
          LOG_PRINT(INFO, "Power button pushed, preparing to power off...");
          timerHalBegin(&powerTimer, POWEROFF_TIMEOUT, false);
          powerOffStep = POWEROFF_STEP1;
        }
        break;
      case POWEROFF_STEP1:
        if (comm.powerOff || (timerHalRun(&powerTimer) == HAL_TIMEOUT)) {
          powerOffStep = POWEROFF_STEP2;
        }
        break;
      case POWEROFF_STEP2:
        alarmSet(&powerOffAlarm);
        timerHalBegin(&powerTimer, POWEROFF_BEEP_TIME, false);
        powerOff = true;
        powerOffStep = POWEROFF_STEP3;
        break;
      case POWEROFF_STEP3:
        if (timerHalRun(&powerTimer) != HAL_IN_PROGRESS) {
          LOG_PRINT(INFO, "Waiting 5s before power off...");
          alarmSuppress(&powerOffAlarm);
          timerHalBegin(&powerTimer, POWEROFF_DELAY_TIME, false);
          powerOffStep = POWEROFF_STEP4;
        }
        break;
      case POWEROFF_STEP4:
        if (timerHalRun(&powerTimer) != HAL_IN_PROGRESS) {
          LOG_PRINT(INFO, "Powering off...");
          sysHalPowerOff();
        }
        break;
    }
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