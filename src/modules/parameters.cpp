/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/
//
// Parameters Module
//

#include <string.h>
#include <stdbool.h>

#include "../pt/pt.h"

#include "../hal/estop.h"
#include "../hal/storage.h"

#include "../modules/link.h"
#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/parameters.h"

#include "../util/alarm.h"

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

static bool checkParameters(struct parameters* params)
{
  // TODO: check ranges of everything in the parameters
  return true;
}

static bool differentLinkAndParameters(void)
{
  // TODO: Come up with a better solution
  return ((comm.startVentilation != parameters.startVentilation) ||
          (comm.calibrationStep != parameters.calibrationStep) ||
          (comm.ventilationMode != parameters.ventilationMode) ||
          (comm.volumeRequested != parameters.volumeRequested) ||
          (comm.respirationRateRequested != parameters.respirationRateRequested) ||
          (comm.ieRatioRequested != parameters.ieRatioRequested) ||
          (comm.pressureRequested != parameters.pressureRequested) ||
          (comm.highVolumeLimit != parameters.highVolumeLimit) ||
          (comm.lowVolumeLimit != parameters.lowVolumeLimit) ||
          (comm.highPressureLimit != parameters.highPressureLimit) ||
          (comm.lowPressureLimit != parameters.lowPressureLimit) ||
          (comm.highRespiratoryRateLimit != parameters.highRespiratoryRateLimit) ||
          (comm.lowRespiratoryRateLimit != parameters.lowRespiratoryRateLimit));
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
  
  // TODO: Actually use sensible limits; just do this now for testing
  parameters.highPressureLimit = 10000;
  parameters.lowPressureLimit = -100;
  parameters.highVolumeLimit = 2000;
  parameters.lowVolumeLimit = 50;
 
  while (1) {
    // TODO: Determine all the conditions that update the parameters; placeholder for now
    PT_WAIT_UNTIL(pt, differentLinkAndParameters() || estopHalAsserted());
    
    tmpParameters[0] = parameters;
    if (differentLinkAndParameters()) {
      // Copy over the paramaters from the comm module
      tmpParameters[0].startVentilation = comm.startVentilation;
      tmpParameters[0].calibrationStep = comm.calibrationStep;
      tmpParameters[0].ventilationMode = comm.ventilationMode;
      tmpParameters[0].volumeRequested = comm.volumeRequested;
      tmpParameters[0].respirationRateRequested = comm.respirationRateRequested;
      tmpParameters[0].ieRatioRequested = comm.ieRatioRequested;
      tmpParameters[0].pressureRequested = comm.pressureRequested;
      tmpParameters[0].highVolumeLimit = comm.highVolumeLimit;
      tmpParameters[0].lowVolumeLimit = comm.lowVolumeLimit;
      tmpParameters[0].highPressureLimit = comm.highPressureLimit;
      tmpParameters[0].lowPressureLimit = comm.lowPressureLimit;
      tmpParameters[0].highRespiratoryRateLimit = comm.highRespiratoryRateLimit;
      tmpParameters[0].lowRespiratoryRateLimit = comm.lowRespiratoryRateLimit;
    } else if (estopHalAsserted()) {
      tmpParameters[0].startVentilation = false;
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
      if (memcmp(&tmpParameters[0], &tmpParameters[1], sizeof(tmpParameters[0])) && 0) { // TODO: disabled until storage HAL implemented
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
    
    // Return to the scheduler after each loop
    PT_YIELD(pt);
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