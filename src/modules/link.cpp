//
// Link Module
//
#include "../pt/pt.h"
#include "../modules/link.h"
#include "../modules/module.h"
#include "../hal/serial.h"

// Private Variables
static struct pt serialThread;
static struct pt serialReadThread;
static struct pt serialSendThread;

static PT_THREAD(serialReadThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalGetData() != HAL_IN_PROGRESS);

  //int rawPressure;
  //pressureSensorHalGetValue(&rawPressure);
  
  // TODO: Process pressure reading, store in public variable
  //currentPressure = 0;
  //peakPressure = 0;
  //plateauPressure = 0;
  
  // TODO: add sampling rate delay

  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(serialSendThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalSendData() != HAL_IN_PROGRESS);

  //int rawPressure;
  //pressureSensorHalGetValue(&rawPressure);
  
  // TODO: Process pressure reading, store in public variable
  //currentPressure = 0;
  //peakPressure = 0;
  //plateauPressure = 0;
  
  // TODO: add sampling rate delay

  PT_RESTART(pt);
  PT_END(pt);
}

PT_THREAD(serialThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  if (!PT_SCHEDULE(serialReadThreadMain(&serialReadThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(serialSendThreadMain(&serialSendThread))) {
    PT_EXIT(pt);
  }
  PT_RESTART(pt);
  PT_END(pt);
}

int linkModuleInit(void)
{
  if (serialHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  // TODO: Implement
  return MODULE_OK;
}

int linkModuleRun(void)
{
  // TODO: Implement
  return (PT_SCHEDULE(serialThreadMain(&serialThread))) ? MODULE_OK : MODULE_FAIL;
}