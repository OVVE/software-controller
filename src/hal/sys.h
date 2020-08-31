/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __SYS_HAL_H__
#define __SYS_HAL_H__

#include <stdint.h>
#include <stdbool.h>

#include "../hal/hal.h"

#define POWERBTN_PIN         64
#define POWEROFF_PIN         12

#define POWERBTN_ASSERT_OFF   0
#define POWERBTN_ASSERT_ON    1

#define POWEROFF_ASSERT       0
#define POWEROFF_DEASSERT     1

// TODO: Doc
int sysHalInit(void);

// TODO: Doc
uint32_t sysHalTime(void);

// TODO: Doc
void sysHalPowerOff(void);

// TODO: Doc
bool sysHalPowerButtonAsserted(void);

#endif /* __SYS_HAL_H__ */
