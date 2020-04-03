#include <Arduino.h>

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"
#include "../modules/link.h"

#define BAUD_RATE 115200

unsigned long last_send_ms = 0;  // this is used for send interval
unsigned long current_send_ms = 0;
unsigned long send_interval_ms = 1000;
unsigned int bytesSent;
unsigned int inByte;    // store each byte
int incoming_index = 0; // index for array of bytes

unsigned long watchdog_max_ms = 50;
unsigned long watchdog_start_ms;
bool watchdog_active = false;
bool watchdog_exceeded = false;
bool clear_input = false;

// this will be used by module/link to send packets
unsigned int sequence_count = 0;
unsigned int last_sequence_count = 0;

bool response_received = true; // do not send data until response received. start with true no response needed.

int serialHalInit(void)
{
  Serial.begin(BAUD_RATE);  // this function does not return anything
  // not sure how we can do something like while (!serial)
  // return MODULE_FAIL;
  return HAL_OK;
}

int serialHalGetData(void)
{
  while (Serial.available())
  {       
    inByte = Serial.read();
    
    // verify that the command is responding to the correct data packet by checking the sequence number and rev
    /*
    if (incoming_index == 0)
    {
      if (incoming_packet.command_packet.sequence_count != data_packet_last_sent.sequence_count)
      {
        // set alarm bit
        
        // do not increment incoming_index and return
        //return (HAL_OK);
      }
    }

    if (incoming_index == 1)
    {
      if (incoming_packet.command_packet.packet_version != data_packet_last_sent.packet_version)
      {
        // set alarm bit
        
        // reset incoming_index and return
        //incoming_index = 0;
        //return (HAL_OK);
        
      }      
    }
    */
    pcommand_packet[incoming_index] = inByte;
    
    //incoming_packet.command_bytes[incoming_index] = inByte;
    incoming_index++;
    if (incoming_index >= sizeof(public_command_packet))
    {
      // save a copy for other modules, but keep a reference in case the shared copy gets modified
      incoming_index = 0;
      ready_to_send = true;
      watchdog_active = false;
      return HAL_OK;
    } 
    if (watchdog_active == true)
    {
      if ((millis() - watchdog_start_ms) > watchdog_max_ms)
      {
        watchdog_active = false; 
        watchdog_exceeded = true; 
        ready_to_send = true;
        return HAL_OK;        
      }
    }
    
  } // while serial.available
  ready_to_send = false;
  return HAL_IN_PROGRESS;
}

int serialHalSendData()
{
  current_send_ms = millis();
  if ((current_send_ms - last_send_ms) >= send_interval_ms)
  {
    if (clear_input == true)
    {
      clear_input = false;
      do
      { unsigned int t = Serial.read();
      } while (Serial.available() > 0);
    }
    bytesSent = Serial.write((byte *)&public_data_packet, sizeof(public_data_packet));
    if (bytesSent != sizeof(public_data_packet)) {
      // handle error
    }
    last_sequence_count = sequence_count;
    sequence_count++;
    last_send_ms = current_send_ms;
    watchdog_start_ms = millis();
    watchdog_active = true;
  }
  return HAL_OK;
}

