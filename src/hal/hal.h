
#ifndef __HAL_H__
#define __HAL_H__

#include <stdint.h>

// TODO: Encoding?
#define HAL_OK           0
#define HAL_IN_PROGRESS  1
#define HAL_TIMEOUT      2
#define HAL_FAIL         3

// Since all timeout values should be uint32_t, allow for an indication of indefinite
// timeout with maximum value
#define HAL_FOREVER      UINT32_MAX

#endif /* __HAL_H__ */