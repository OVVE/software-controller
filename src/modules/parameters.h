
#ifndef __PARAMETERS_MODULE_H__
#define __PARAMETERS_MODULE_H__

#include <stdbool.h>

struct parameters {
  // TODO; fill in parameters
  bool run;
  bool assist;
  unsigned int volumeRequested;
  unsigned int respirationRateRequested;
  unsigned int ieRatioRequested;
};

// Public Variables
extern struct parameters parameters;

// TODO: Doc
int parametersModuleInit(void);

// TODO: Doc
int parametersModuleRun(void);

#endif /* __PARAMETERS_MODULE_H__ */