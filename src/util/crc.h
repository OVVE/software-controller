
#ifndef __CRC_UTIL_H__
#define __CRC_UTIL_H__

#include <stddef.h>
#include <stdint.h>

#define CRC16_INITIAL_VALUE 0xFFFFU

uint16_t crc16(uint8_t* data, size_t size, uint16_t prev);

#endif /* __CRC_UTIL_H__ */