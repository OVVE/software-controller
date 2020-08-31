
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