
#ifndef __STORAGE_HAL_H__
#define __STORAGE_HAL_H__

#include "../hal/hal.h"

#define STORAGE_HAL_PAGE_SIZE 256

// TODO: Doc
int storageHalInit(void);

// TODO: Doc
int storageHalRead(unsigned int address, void* buffer, unsigned int size);

// TODO: Doc
int storageHalWrite(unsigned int address, void* buffer, unsigned int size);

#endif /* __STORAGE_HAL_H__ */