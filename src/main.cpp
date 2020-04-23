
#include "modules/link.h"
#include "modules/module.h"
#include "modules/sensors.h"
#include "modules/control.h"
#include "modules/parameters.h"

#define DEBUG
#define DEBUG_MODULE "main"
#include "util/debug.h"

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
}

void mainLoop(void)
{
  // TODO: Clean up prints
  // DEBUG_PRINT_EVERY(10000, "in main loop");
  // Run all modules in RR; take specified actions in the event of failure
  
  if (controlModuleRun() != MODULE_OK) {
    // TODO: control module exited, trigger severe error
  }
  
  if (sensorsModuleRun() != MODULE_OK) {
    // TODO: sensor module exited, trigger severe error  
  }

  if (parametersModuleRun() != MODULE_OK) {
    // TODO: parameters module exited, attempt recovery
  }

  if (linkModuleRun() != MODULE_OK) {
    // TODO: link module exited, attempt recovery
  }

  // TODO: Failure actions (alarm, watchdog)
}
