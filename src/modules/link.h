
#ifndef __LINK_MODULE_H__
#define __LINK_MODULE_H__

#include <stdbool.h>

struct link {
  bool update;
  bool assist;
  unsigned int volumeRequested;
  unsigned int respirationRateRequested;
  unsigned int ieRatioRequested;
};

// Public Variables
extern struct link comm;

// TODO: Doc
int linkModuleInit(void);

// TODO: Doc
int linkModuleRun(void);

#endif /* __LINK_MODULE_H__ */