/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "../hal/hal.h"
#include "../hal/watchdog.h"

#if defined(__AVR__)
  #include <avr/wdt.h>
#elif defined(__arm__)
// TODO: Consider making this more specific if needed
  #include <include/wdt.h>
#else
  #error "Unsupported Architecture for Watchdog HAL"
#endif

#if defined(__arm__)
// Atmel SAM3X8E only allows a single write to WDT_MR; which is written to by
// the core libraries to disable the watchdog by default; override this function
// to prevent that
void watchdogSetup(void) {
  return;
}
#endif

int watchdogHalInit(void)
{
#if defined(__AVR__)
  switch (WATCHDOG_PERIOD_MS) {
    case   15: wdt_enable(WDTO_15MS); break;
    case   30: wdt_enable(WDTO_30MS); break;
    case   60: wdt_enable(WDTO_60MS); break;
    case  120: wdt_enable(WDTO_120MS); break;
    case  250: wdt_enable(WDTO_250MS); break;
    case  500: wdt_enable(WDTO_500MS); break;
    case 1000: wdt_enable(WDTO_1S); break;
    case 2000: wdt_enable(WDTO_2S); break;
    default: return HAL_FAIL;
  }
#elif defined(__arm__)
  if (WATCHDOG_PERIOD_MS <= 16000) {
    WDT_Enable(WDT,
               WDT_MR_WDRSTEN | WDT_MR_WDV(WATCHDOG_PERIOD_MS * 256 / 1000));
  } else {
    return HAL_FAIL;
  }
#endif

  return HAL_OK;
}

int watchdogHalReset(void)
{
#if defined(__AVR__)
  wdt_reset();
#elif defined(__arm__)
  WDT_Restart(WDT);
#endif
  return HAL_OK;
}