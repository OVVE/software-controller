#include <Arduino.h>

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"
#include "../modules/link.h"

#define PACKET_VERSION 1

unsigned long last_send_ms = 0;  // this is used for send interval
unsigned long current_send_ms = 0;
unsigned long send_interval_ms = 1000;
unsigned int bytesSent;
unsigned int inByte;

data_packet_def data_packet;
data_packet_def data_packet_last_sent;

//command_packet_def command_packet;
// this is used to handle byte by byte input
union incoming_packet_u {
  command_packet_def command_packet;
  unsigned int command_bytes[sizeof(command_packet)];
};


// data for handling serial reads
incoming_packet_u incoming_packet;
int incoming_index = 0;
//incoming_packet_u shared_command_packet; // this should be an extern in other modules that need to read this
//incoming_packet_u command_packet_reference; // this is saved in case other modules overwrite the shared copy
command_packet_def command_packet_reference;

command_packet_def public_command_packet;
data_packet_def public_data_packet;


// this will be used by module/link to send packets
unsigned int sequence_count = 0;
unsigned int last_sequence_count = 0;

boolean command_ready = false;

#define POLY 0x8408
/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/
unsigned int crc_i;
unsigned int crc_data;
unsigned int crc = 0xffff;

unsigned int crc16(unsigned int *data_p, unsigned int length)
{
      //unsigned char i;
      //unsigned int data;
      //unsigned int crc = 0xffff;

      if (length == 0)
            return (~crc);

      do
      {
            for (crc_i=0, crc_data=(unsigned int)0xff & *data_p++;
                 crc_i < 8; 
                 crc_i++, crc_data >>= 1)
            {
                  if ((crc_i & 0x0001) ^ (crc_data & 0x0001))
                        crc = (crc >> 1) ^ POLY;
                  else  crc >>= 1;
            }
      } while (--length);

      crc = ~crc;
      crc_data = crc;
      crc = (crc << 8) | (crc_data >> 8 & 0xff);

      return (crc);
}

int serialHalInit(void)
{
  Serial.begin(115200);  // this function does not return anything
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
    incoming_packet.command_bytes[incoming_index] = inByte;
    incoming_index++;
    if (incoming_index >= sizeof(incoming_packet.command_bytes))
    {
      // save a copy for other modules, but keep a reference in case the shared copy gets modified
      memcpy((void *)&public_command_packet, (void *)&incoming_packet.command_packet, sizeof(public_command_packet));
      memcpy((void *)&command_packet_reference, (void *)&incoming_packet.command_packet, sizeof(command_packet_reference));
      incoming_index = 0;
      command_ready = true;
      return HAL_OK;
    } 
  } // while serial.available
  return HAL_IN_PROGRESS;
}

int serialHalSendData()
{
  current_send_ms = millis();
  if ((current_send_ms - last_send_ms) >= send_interval_ms)
  {
    //unsigned short crc16(char *data_p, unsigned short length)
    public_data_packet.crc = ' ';    
    public_data_packet.sequence_count = sequence_count;
    public_data_packet.packet_version = PACKET_VERSION;
    //public_data_packet.battery_level = 'B';
    
    //public_data_packet.crc = crc16((unsigned int *)&public_data_packet, sizeof(public_data_packet) - 2);
    memcpy((void *)&data_packet_last_sent, (void *)&public_data_packet, sizeof(data_packet_last_sent));
    memcpy((void *)&public_data_packet, (void *)&public_data_packet, sizeof(public_data_packet));
    
    
#define TESTVALUE
#ifdef TESTVALUE

#endif    
    
    bytesSent = Serial.write((byte *)&public_data_packet, sizeof(public_data_packet));
    if (bytesSent != sizeof(public_data_packet)) {
      // handle error
    }
    sequence_count++;
    last_send_ms = current_send_ms;
  }
  return HAL_OK;
}

