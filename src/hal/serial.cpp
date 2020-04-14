// TODO: Document what this module does

#include <stdint.h>

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"

// Private Variables
uint32_t last_send_ms = 0;        // this is used for send interval

//uint16_t bytesSent;               // Serial.write() returns this - we should increase the default Serial buffer size so that the function does not block
uint8_t inByte;                   // store each byte read
//int incoming_index = 0;           // index for array of bytes received as we are getting them one at a time
uint32_t watchdog_start_ms;       // save the watchdog start time
bool read_active = false;     // watchdog timer in progress (waiting for command/confirm packet)

// Public Variables
// this is the data bytes that correspond to data structures from link.cpp - serial.cpp sends/receives generic bytes just knowing the count and timeout value
extern uint16_t sizeof_data_bytes;
extern uint8_t data_bytes[];
extern uint16_t sizeof_command_bytes;
extern uint8_t command_bytes[];
bool watchdog_exceeded = false;   // flag for watchdog timer received. this is for link.cpp.
bool clear_input = false;         // if we get a bad packet (wrong sequence id or crc) set this to true and trash input buffer just before sending next data packet

int serialHalInit(void)
{
  SERIAL_UI.begin(BAUD_RATE);  // this function does not return anything
  // not sure how we can do something like while (!serial)
  // return MODULE_FAIL;
  do
  { SERIAL_UI.read();
  } while (SERIAL_UI.available() > 0);
  
#ifdef SERIAL_DEBUG
  SERIAL_DEBUG.begin(BAUD_RATE);
#endif  
  return HAL_OK;
}

int serialHalGetData(void)
{
  static int inComingIndex = 0;
  
  if (read_active != true)
  {
    return HAL_IN_PROGRESS;
  }
  // change to while if reading all the bytes currently available
  if (SERIAL_UI.available())
  {       
    SERIAL_UI.readBytes(&inByte, 1);
    
    command_bytes[inComingIndex] = inByte;
    
    inComingIndex++;

    if (inComingIndex >= sizeof_command_bytes)
    {
      // save a copy for other modules, but keep a reference in case the shared copy gets modified
      inComingIndex = 0;
      read_active = false;
      watchdog_exceeded = false;
      return HAL_OK;
    } 
  }
  if (read_active == true)
  {
    if ((millis() - watchdog_start_ms) > WATCHDOG_MS)
    {
      read_active = false; 
      watchdog_exceeded = true; 
      inComingIndex = 0;
      return HAL_OK;        
    }
  } // if serial.available
  return HAL_IN_PROGRESS;
}

int serialHalSendData()
{
  static uint32_t current_ms;
  current_ms = millis();
  if ((current_ms - last_send_ms) >= SEND_INTERVAL_MS && read_active != true)
  {
    if (clear_input == true)
    {
      clear_input = false;
      do
      { SERIAL_UI.read();
      } while (SERIAL_UI.available() > 0);
    }
    static uint16_t bytesSent = 0;
    static int bytesToWrite;
    static uint32_t startSendTime = 0;
    if (bytesSent < sizeof_data_bytes)
    {
      if (startSendTime == 0)
      {
        startSendTime = current_ms;
      }
#ifdef USE_AVAILABLE_WRITE 
      static uint16_t availableForWrite;     
      availableForWrite = SERIAL_UI.availableForWrite();
      bytesToWrite = min(availableForWrite, sizeof_data_bytes - bytesSent);
#else
      bytesToWrite = sizeof_data_bytes;
#endif  
      bytesSent += SERIAL_UI.write((byte *)&data_bytes[bytesSent], bytesToWrite);
    }
    if (bytesSent < sizeof_data_bytes) {
      if (current_ms - startSendTime > SEND_MAX_TIME_MS)
      {
        bytesSent = 0;
        startSendTime = 0;
        clear_input = true;
        last_send_ms = current_ms;
        return HAL_TIMEOUT;
      }
      return HAL_IN_PROGRESS;
    }
    bytesSent = 0;
    startSendTime = 0;
    //last_sequence_count = sequence_count;
    //sequence_count++;
    last_send_ms = current_ms;
    watchdog_start_ms = millis();
    //inComingIndex = 0; // set this for incoming bytes
#ifdef SERIAL_DEBUG
    SERIAL_DEBUG.print("watchdog_start_ms initialized: ");
    SERIAL_DEBUG.println(watchdog_start_ms, DEC);
#endif    
    read_active = true;
    return HAL_OK;
  }
  return HAL_IN_PROGRESS;
}


/*
// this function can be deleted if everything is stable
// had some slowness with above function which had something
// to do with the debug prints that were being used.
int serialHalSendData()
{
  current_send_ms = millis();
  if ((current_send_ms - last_send_ms) >= SEND_INTERVAL_MS && read_active != true)
  {
    if (clear_input == true)
    {
      clear_input = false;
      do
      { SERIAL_UI.read();
      } while (SERIAL_UI.available() > 0);
    }
    static uint16_t bytesSent;
    //static uint16_t availableForWrite;
    //static int bytesToWrite;
    //static int bytesToWrite = min(SERIAL_UI.availableForWrite(), sizeof_data_bytes) - bytesSent);      
    bytesSent = SERIAL_UI.write((byte *)&data_bytes, sizeof_data_bytes);

    if (bytesSent != sizeof_data_bytes) {
      // handle error
    }
       
    //last_sequence_count = sequence_count;
    //sequence_count++;
    last_send_ms = current_send_ms;
    watchdog_start_ms = millis();
    //inComingIndex = 0; // set this for incoming bytes
#ifdef SERIAL_DEBUG
    SERIAL_DEBUG.print("watchdog_start_ms initialized: ");
    SERIAL_DEBUG.println(watchdog_start_ms, DEC);
#endif    
    read_active = true;
    return HAL_OK;
  }
  return HAL_IN_PROGRESS;
}
*/