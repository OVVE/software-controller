#include <Arduino.h>

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"
#include "../modules/link.h"

#define BAUD_RATE 115200

uint32_t last_send_ms = 0;  // this is used for send interval
uint32_t current_send_ms = 0;
uint32_t send_interval_ms = 100;
uint16_t bytesSent;
uint8_t inByte;    // store each byte
int incoming_index = 0; // index for array of bytes

uint32_t watchdog_max_ms = 50;
uint32_t watchdog_start_ms;
bool watchdog_active = false;
bool watchdog_exceeded = false;
bool clear_input = false;

// this will be used by module/link to send packets
uint16_t sequence_count = 0;
uint16_t last_sequence_count = 0;

bool response_received = true; // do not send data until response received. start with true no response needed.

typedef union command_packet_union{
command_packet_def command_packet;
uint8_t command_bytes[sizeof(command_packet_def)];
};

command_packet_union command_packet_u;

int serialHalInit(void)
{
  Serial.begin(BAUD_RATE);  // this function does not return anything
  // not sure how we can do something like while (!serial)
  // return MODULE_FAIL;
  do
  { Serial.read();
  } while (Serial.available() > 0);
  return HAL_OK;
}

int serialHalGetData(void)
{
  if (Serial.available())
  {       
    Serial.readBytes(&inByte, 1);
    
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
    //pcommand_packet[incoming_index] = inByte;
    command_packet_u.command_bytes[incoming_index] = inByte;
    
    //incoming_packet.command_bytes[incoming_index] = inByte;
    incoming_index++;

    if (incoming_index >= sizeof(public_command_packet))
    {
      // save a copy for other modules, but keep a reference in case the shared copy gets modified
      incoming_index = 0;
      watchdog_active = false;

      memcpy((void *)&public_command_packet, (void *)&command_packet_u.command_packet, sizeof(public_command_packet));
      //Serial.println("count: " + incoming_index);
      // clear alarm bits
      public_command_packet.alarm_bits &= ~(1 << ALARM_DROPPED_PACKET);
      public_command_packet.alarm_bits &= ~(1 << ALARM_CRC_ERROR);
      ready_to_send = true;
      return HAL_OK;
    } 
    if (watchdog_active == true)
    {
      if ((millis() - watchdog_start_ms) > watchdog_max_ms)
      {
        watchdog_active = false; 
        watchdog_exceeded = true; 
        ready_to_send = true;
        incoming_index = 0;
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
      { Serial.read();
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

