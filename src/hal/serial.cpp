// TODO: Document what this module does

#include <stdint.h>

#include "../config.h"

#include "../hal/hal.h"
#include "../hal/serial.h"

struct serialHalPort serialHalPortUI = {
  port : &SERIAL_PORT_UI_PORT,
};

struct serialHalPort serialHalPortDebug = {
  port : &SERIAL_PORT_DEBUG_PORT,
};

int serialHalInit(void)
{
  SERIAL_PORT_UI_PORT.begin(SERIAL_PORT_UI_BAUD);
  SERIAL_PORT_DEBUG_PORT.begin(SERIAL_PORT_DEBUG_BAUD);
  return HAL_OK;
}

int serialHalWrite(struct serialHalPort* port, void* data, size_t size, uint32_t timeout)
{
  if (port->writeBytesLeft == 0) {
    // Starting up for the first time, start the timeout timer and setup the state
    if (timeout < HAL_FOREVER) {
      timerHalBegin(&(port->writeTimer), timeout, false);
    }
    
    port->writeBytesLeft = size;
  } else if ((timeout < HAL_FOREVER) && (timerHalRun(&(port->writeTimer)) != HAL_IN_PROGRESS)) {
    // Check for timeout
    port->writeBytesLeft = 0;
    return HAL_TIMEOUT;
  }

  // Write as much as is possible
  int available = port->port->availableForWrite();
  while ((port->writeBytesLeft > 0) && (available > 0)) {
    uint8_t* dataBuffer = (uint8_t*) data;
    port->port->write(dataBuffer[size - port->writeBytesLeft]);
    port->writeBytesLeft--;
    available--;
  }
  
  return (port->writeBytesLeft > 0) ? HAL_IN_PROGRESS : HAL_OK;
}

int serialHalRead(struct serialHalPort* port, void* data, size_t size, uint32_t timeout)
{
  if (port->readBytesLeft == 0) {
    // Starting up for the first time, start the timeout timer and setup the state
    if (timeout < HAL_FOREVER) {
      timerHalBegin(&(port->readTimer), timeout, false);
    }
    
    port->readBytesLeft = size;
  } else if ((timeout < HAL_FOREVER) && (timerHalRun(&(port->readTimer)) != HAL_IN_PROGRESS)) {
    // Check for timeout
    port->readBytesLeft = 0;
    return HAL_TIMEOUT;
  }

  // Read as much as is possible
  int available = port->port->available();
  while ((port->readBytesLeft > 0) && (available > 0)) {
    uint8_t* dataBuffer = (uint8_t*) data;
    dataBuffer[size - port->readBytesLeft] = port->port->read();
    port->readBytesLeft--;
    available--;
  }
  
  return (port->readBytesLeft > 0) ? HAL_IN_PROGRESS : HAL_OK;
}
