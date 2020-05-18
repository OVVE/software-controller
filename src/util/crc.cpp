
#include <stddef.h>
#include <stdint.h>

#include <util/crc16.h>

#include "../util/crc.h"

uint16_t crc16(uint8_t* data, size_t size, uint16_t prev)
{
  uint16_t crc = prev;
  for (int i = 0; i < size; i++) {
    crc = _crc_xmodem_update(crc, data[i]);
  }
  return crc;
}