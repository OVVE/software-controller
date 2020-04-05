#include <Arduino.h>

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"
#include "../modules/link.h"

#define BAUD_RATE 38400

//#define SERIAL_DEBUG found in serial.h - debug Serial1.print() in serial.cpp and link.cpp
#ifdef SERIAL_DEBUG
uint8_t debug_byte;
uint16_t debug_index;
#endif

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
  
#ifdef SERIAL_DEBUG
  Serial1.begin(BAUD_RATE);
#endif  
  return HAL_OK;
}

int serialHalGetData(void)
{
  if (Serial.available())
  {       
    Serial.readBytes(&inByte, 1);
    
    command_packet_u.command_bytes[incoming_index] = inByte;
    
    incoming_index++;

    if (incoming_index >= sizeof(comm.public_command_packet))
    {
      // save a copy for other modules, but keep a reference in case the shared copy gets modified
      incoming_index = 0;
      watchdog_active = false;

      memcpy((void *)&comm.public_command_packet, (void *)&command_packet_u.command_packet, sizeof(comm.public_command_packet));
      //Serial.println("count: " + incoming_index);
      // clear alarm bits
      comm.public_command_packet.alarm_bits &= ~(1 << ALARM_DROPPED_PACKET);
      comm.public_command_packet.alarm_bits &= ~(1 << ALARM_CRC_ERROR);
      ready_to_send = true;
#ifdef SERIAL_DEBUG
      Serial1.print("Received from Rpi: 0x");
      Serial1.print(comm.public_command_packet.sequence_count, HEX);
      Serial1.print(" 0x");
      Serial1.println(comm.public_command_packet.crc, HEX);
      Serial1.println(" ");
#endif      
      return HAL_OK;
    } 
    if (watchdog_active == true)
    {
      if ((millis() - watchdog_start_ms) > watchdog_max_ms)
      {
#ifdef SERIAL_DEBUG
        Serial1.println("!!!Watchdog timeout");
#endif        
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
#ifdef SERIAL_DEBUG
      Serial1.print("Sent to Rpi: ");
      Serial1.print(comm.public_data_packet.sequence_count, HEX);
      Serial1.print(" ");
      Serial1.println(comm.public_data_packet.crc, HEX);
      Serial1.println(" ");
#endif 
       
    bytesSent = Serial.write((byte *)&comm.public_data_packet, sizeof(comm.public_data_packet));

    if (bytesSent != sizeof(comm.public_data_packet)) {
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

