//
// Parameters Module
//

#include <string.h>
#include <stdbool.h>

#include "../pt/pt.h"

#include "../hal/storage.h"

#include "../modules/link.h"
#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/parameters.h"
#include "../hal/serial.h"

#define PARAMETERS_BANK_ADDRESS 0
#define PARAMETERS_BANK(b) (STORAGE_HAL_PAGE_SIZE * \
                            (1 + (sizeof(struct parameters) + \
                                  STORAGE_HAL_PAGE_SIZE - 1) / STORAGE_HAL_PAGE_SIZE) * (b))

// Public Variables
struct parameters parameters;

// Private Variables
static struct pt parametersThread;
static struct parameters tmpParameters[2];
static unsigned int parametersAddress;

static struct pt serialThread;
static struct pt serialReadThread;
static struct pt serialSendThread;

static bool checkParameters(struct parameters* params)
{
  // TODO: check ranges of everything in the parameters
  return true;
}

static PT_THREAD(parametersThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  // Read out the current parameter settings
  PT_WAIT_UNTIL(pt, storageHalRead(PARAMETERS_BANK_ADDRESS,
                                   &parametersAddress,
                                   sizeof(parametersAddress)) != HAL_IN_PROGRESS);
  PT_WAIT_UNTIL(pt, storageHalRead(PARAMETERS_BANK(parametersAddress),
                                   &tmpParameters[0],
                                   sizeof(tmpParameters[0])) != HAL_IN_PROGRESS);
  
  if (!checkParameters(&tmpParameters[0])) {
    // TODO: Handle error reading back in parameters (ie, parameters out of range)
    PT_EXIT(pt);
  }

  // Copy the parameters from storage into the publicly available buffer
  parameters = tmpParameters[0];
 
  while (1) {
    // TODO: Determine all the conditions that update the parameters
    PT_WAIT_UNTIL(pt, comm.update);
    
    tmpParameters[0] = parameters;
    if (comm.update) {
      // Copy over the paramaters from the comm module
      tmpParameters[0].assist = comm.assist;
      tmpParameters[0].volumeRequested = comm.volumeRequested;
      tmpParameters[0].respirationRateRequested = comm.respirationRateRequested;
      tmpParameters[0].ieRatioRequested = comm.ieRatioRequested;
      comm.update = false;
    }
    
    if (!checkParameters(&tmpParameters[0])) {
      // TODO: Handle bad parameters in update
    } else {
      // Write the new parameters into the other bank and then read them back,
      // confirming the new parameters are valid
      PT_WAIT_UNTIL(pt, storageHalWrite(PARAMETERS_BANK(!parametersAddress),
                                        &tmpParameters[0],
                                        sizeof(tmpParameters[0])) != HAL_IN_PROGRESS);
      PT_WAIT_UNTIL(pt, storageHalRead(PARAMETERS_BANK(!parametersAddress),
                                       &tmpParameters[1],
                                       sizeof(tmpParameters[1])) != HAL_IN_PROGRESS);
      if (!memcmp(&tmpParameters[0], &tmpParameters[1], sizeof(tmpParameters[0]))) {
        // TODO: Error on parameter mismatch after read-back
      } else {
        // Write back new parameters address to get the data from the new bank next time
        parametersAddress = !parametersAddress;
        PT_WAIT_UNTIL(pt, storageHalWrite(PARAMETERS_BANK_ADDRESS,
                                          &parametersAddress,
                                          sizeof(parametersAddress)) != HAL_IN_PROGRESS);
        // Finally, update current operating parameters
        parameters = tmpParameters[0];
      }
    }
  }
  
  // Should never reach here
  PT_END(pt);
}

int parametersModuleInit(void)
{
  PT_INIT(&parametersThread);
  
  return MODULE_OK;
}

int parametersModuleRun(void)
{
  return (PT_SCHEDULE(parametersThreadMain(&parametersThread))) ? MODULE_OK : MODULE_FAIL;
}


// Serial
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

int serialModuleInit(void)
{
  if (serialHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  // TODO: Implement
  return MODULE_OK;
}

int serialModuleRun(void)
{
  // TODO: Implement
  return (PT_SCHEDULE(serialThreadMain(&serialThread))) ? MODULE_OK : MODULE_FAIL;
}