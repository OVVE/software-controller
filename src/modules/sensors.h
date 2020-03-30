
#ifndef __SENSORS_MODULE_H__
#define __SENSORS_MODULE_H__

// Public Variables
// TODO: Units?
extern int currentPressure;
extern int peakPressure;
extern int plateauPressure; 

// TODO: Doc
int sensorsModuleInit(void);

// TODO: Doc
int sensorsModuleRun(void);

#endif /* __SENSORS_MODULE_H__ */