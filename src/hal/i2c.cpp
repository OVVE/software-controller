/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>
#include <util/twi.h>

#include <Arduino.h>

#include "../config.h"

#include "../hal/i2c.h"

#define LOG_MODULE "i2c"
#define LOG_LEVEL  LOG_I2C_HAL
#include "../util/log.h"

#define I2C_STATE_IDLE             0
#define I2C_STATE_SENT_START       1
#define I2C_STATE_SENT_DEVICE_ADDR 2
#define I2C_STATE_PREP_DATA        3
#define I2C_STATE_SENT_DATA        4
#define I2C_STATE_GET_DATA         5
#define I2C_STATE_GOT_DATA         6
#define I2C_STATE_SENT_STOP        7

static bool i2cHalInProgress(void)
{
  return bit_is_clear(TWCR, TWINT);
}

int i2cHalInit(void)
{
  // Initialize TWI controller to have 100MHz baud
  // SCL = CLKcpu / (16 + 2 * TWBR * 4 ^ TWSR[1:0])
  //     = 16 MHz / (16 + 2 * (72) * 4 ^ (0))
  //     = 16 MHz / (16 + 144)
  //     = 16 MHz / (160)
  //     = 100 KHz
  TWBR = 72;
  TWSR = 0;

  return HAL_OK;
}

int i2cHalWrite(uint8_t deviceAddr, uint8_t* data, uint8_t size)
{
  static uint8_t state = I2C_STATE_IDLE;
  static uint8_t bytesSent = 0;
  
  switch (state) {
    case I2C_STATE_IDLE:
      bytesSent = 0;
      state = I2C_STATE_SENT_START;
      TWCR = bit(TWINT) | bit(TWSTA) | bit(TWEN);
      LOG_PRINT(VERBOSE, "write: begin");

    case I2C_STATE_SENT_START:
      if (i2cHalInProgress()) {
        // Still working on it
        break;
      }
      // Check for I2C error
      if (TWSR != TW_START) {
        LOG_PRINT(VERBOSE, "write: failed to sent start");
        state = I2C_STATE_IDLE;
        TWCR = bit(TWINT) | bit(TWSTO);
        return HAL_FAIL;
      }
      // Now send device address
      TWDR = TW_WRITE | (deviceAddr << 1);
      state = I2C_STATE_SENT_DEVICE_ADDR;
      TWCR = bit(TWINT) | bit(TWEN);
      LOG_PRINT(VERBOSE, "write: start sent, sending device address %02x", deviceAddr);
      
    case I2C_STATE_SENT_DEVICE_ADDR:
      if (i2cHalInProgress()) {
        // Still working on it
        break;
      }
      // Check for I2C error
      if (TWSR != TW_MT_SLA_ACK) {
        LOG_PRINT(VERBOSE, "write: NAK from device address");
        state = I2C_STATE_IDLE;
        return HAL_FAIL;
      }
      state = I2C_STATE_PREP_DATA;
    
    case I2C_STATE_PREP_DATA:
      TWDR = data[bytesSent];
      state = I2C_STATE_SENT_DATA;
      TWCR = bit(TWINT) | bit(TWEN);
      LOG_PRINT(VERBOSE, "write: sending data %02x", data[bytesSent]);
    
    case I2C_STATE_SENT_DATA:
      if (i2cHalInProgress()) {
        // Still working on it
        break;
      }
      // Check for I2C error
      if (TWSR != TW_MT_DATA_ACK) {
        LOG_PRINT(VERBOSE, "write: NAK from device data");
        state = I2C_STATE_IDLE;
        return HAL_FAIL;
      }
      bytesSent++;
      
      // Sent everything, time to return
      if (bytesSent == size) {
        state = I2C_STATE_SENT_STOP;
        TWCR = bit(TWINT) | bit(TWSTO) | bit(TWEN);
        LOG_PRINT(VERBOSE, "write: sending stop");
      } else {
        state = I2C_STATE_PREP_DATA;
        break;
      }
    
    case I2C_STATE_SENT_STOP:
      if (bit_is_set(TWCR, TWSTO)) {
        // STOP is different and clears TWSTO instead of TWINT
        break;
      }
      LOG_PRINT(VERBOSE, "write: stop sent, complete");
      state = I2C_STATE_IDLE;
      return HAL_OK;
  }
  
  return HAL_IN_PROGRESS;
}

int i2cHalRead(uint8_t deviceAddr, uint8_t* data, uint8_t size)
{
  static uint8_t state = I2C_STATE_IDLE;
  static uint8_t bytesRecv = 0;
  
  switch (state) {
    case I2C_STATE_IDLE:
      bytesRecv = 0;
      state = I2C_STATE_SENT_START;
      TWCR = bit(TWINT) | bit(TWSTA) | bit(TWEN);
      LOG_PRINT(VERBOSE, "read: begin");

    case I2C_STATE_SENT_START:
      if (i2cHalInProgress()) {
        // Still working on it
        break;
      }
      // Check for I2C error
      if (TWSR != TW_START) {
        LOG_PRINT(VERBOSE, "read: failed to sent start");
        state = I2C_STATE_IDLE;
        TWCR = bit(TWINT) | bit(TWSTO);
        return HAL_FAIL;
      }
      // Now send device address
      TWDR = TW_READ | (deviceAddr << 1);
      state = I2C_STATE_SENT_DEVICE_ADDR;
      TWCR = bit(TWINT) | bit(TWEN);
      LOG_PRINT(VERBOSE, "read: start sent, sending device address %02x", deviceAddr);
      
    case I2C_STATE_SENT_DEVICE_ADDR:
      if (i2cHalInProgress()) {
        // Still working on it
        break;
      }
      // Check for I2C error
      if (TWSR != TW_MR_SLA_ACK) {
        LOG_PRINT(VERBOSE, "read: NAK from device address");
        state = I2C_STATE_IDLE;
        return HAL_FAIL;
      }
      state = I2C_STATE_GET_DATA;
    
    case I2C_STATE_GET_DATA:
      state = I2C_STATE_GOT_DATA;
      if (size - bytesRecv > 1) {
        TWCR = bit(TWINT) | bit(TWEA) | bit(TWEN);
      } else {
        TWCR = bit(TWINT) | bit(TWEN);
      }
      LOG_PRINT(VERBOSE, "read: getting more data (%u bytes left)", size - bytesRecv);
    
    case I2C_STATE_GOT_DATA:
      if (i2cHalInProgress()) {
        // Still working on it
        break;
      }
      // Check for I2C error
      if (((size - bytesRecv > 1) && (TWSR != TW_MR_DATA_ACK)) ||
          ((size - bytesRecv == 1) && (TWSR != TW_MR_DATA_NACK))) {
        state = I2C_STATE_IDLE;
        return HAL_FAIL;
      }
      data[bytesRecv] = TWDR;
      LOG_PRINT(VERBOSE, "read: got data %02x", data[bytesRecv]);
      bytesRecv++;
      
      // Sent everything, time to return
      if (bytesRecv == size) {
        state = I2C_STATE_SENT_STOP;
        TWCR = bit(TWINT) | bit(TWSTO) | bit(TWEN);
        LOG_PRINT(VERBOSE, "read: sending stop");
      } else {
        state = I2C_STATE_GET_DATA;
        break;
      }
      
    case I2C_STATE_SENT_STOP:
      if (bit_is_set(TWCR, TWSTO)) {
        // STOP is different and clears TWSTO instead of TWINT
        break;
      }
      LOG_PRINT(VERBOSE, "read: stop sent, complete");
      state = I2C_STATE_IDLE;
      return HAL_OK;
  }
  
  return HAL_IN_PROGRESS;
}