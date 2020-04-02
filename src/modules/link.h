
#ifndef __LINK_MODULE_H__
#define __LINK_MODULE_H__

#include <stdint.h>

struct link {
  // Variables
  uint8_t  startVentilation;
  uint8_t  ventilationMode;
  uint32_t volumeRequested;
  uint32_t respirationRateRequested;
  uint32_t ieRatioRequested;
  
  // Alarms
  int8_t   droppedPacketAlarm;
  int8_t   crcErrorAlarm;
  int8_t   unsupportedPacketVersionAlarm;
};

// Public Variables
extern struct link comm;

// TODO: Doc
int linkModuleInit(void);

// TODO: Doc
int linkModuleRun(void);

#endif /* __LINK_MODULE_H__ */