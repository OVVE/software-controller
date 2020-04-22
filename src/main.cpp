
#include "modules/link.h"
#include "modules/module.h"
#include "modules/sensors.h"
#include "modules/control.h"
#include "modules/parameters.h"

#include "util/metrics.h"

#define DEBUG
#define DEBUG_MODULE "main"
#include "util/debug.h"

static struct metrics scheduleMetrics;
static struct metrics controlModuleMetrics;
static struct metrics sensorsModuleMetrics;
static struct metrics parametersModuleMetrics;
static struct metrics linkModuleMetrics;

void mainSetup(void)
{
  DEBUG_BEGIN;
  
  // TODO: cleanup prints
  DEBUG_PRINT("setup");

  // TODO: Handle failure conditions
  controlModuleInit();
  sensorsModuleInit();
  parametersModuleInit();
  linkModuleInit();

  metricsReset(&scheduleMetrics);
  metricsReset(&controlModuleMetrics);
  metricsReset(&sensorsModuleMetrics);
  metricsReset(&parametersModuleMetrics);
  metricsReset(&linkModuleMetrics);
  DEBUG_PRINT("setup done");
}

void mainLoop(void)
{
  metricsStart(&scheduleMetrics);
  
  // Run all modules in RR; take specified actions in the event of failure
  metricsStart(&controlModuleMetrics);
  if (controlModuleRun() != MODULE_OK) {
    // TODO: control module exited, trigger severe error
  }
  metricsStop(&controlModuleMetrics);
  
  metricsStart(&sensorsModuleMetrics);
  if (sensorsModuleRun() != MODULE_OK) {
    // TODO: sensor module exited, trigger severe error  
  }
  metricsStop(&sensorsModuleMetrics);

  metricsStart(&parametersModuleMetrics);
  if (parametersModuleRun() != MODULE_OK) {
    // TODO: parameters module exited, attempt recovery
  }
  metricsStop(&parametersModuleMetrics);

  metricsStart(&linkModuleMetrics);
  if (linkModuleRun() != MODULE_OK) {
    // TODO: link module exited, attempt recovery
  }
  metricsStop(&linkModuleMetrics);

  // TODO: Failure actions (alarm, watchdog)

  metricsStop(&scheduleMetrics);
  DEBUG_PRINT_EVERY(25000, "Runtime Metrics:");
  DEBUG_PRINT_EVERY(25000, " Main loop:  %10lu us : %10lu us : %10lu us",
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
}