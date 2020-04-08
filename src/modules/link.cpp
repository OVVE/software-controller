//
// Link Module
//
#include <stdint.h>
#include <FastCRC.h>
#include "../pt/pt.h"
#include "../modules/link.h"
#include "../modules/module.h"
#include "../hal/serial.h"
#include <Arduino.h>

FastCRC16 CRC16;  // this is the class to create crc16 to match the CCITT crc16 that rpi is using

// Private Variables
struct link comm;
static struct pt serialThread;
static struct pt serialReadThread;
static struct pt serialSendThread;

uint16_t calc_crc;

// copy the public data packet to this buffer because there might be changes
// by other modules while serial.cpp is waiting to send this
data_packet_def update_crc_data_packet;
command_packet_def command_packet_from_serial;

#ifdef SERIAL_DEBUG
extern HardwareSerial &serial_debug;
#endif
uint32_t watchdog_count = 0;
uint32_t dropped_packet_count = 0;
uint32_t packet_count = 0;

uint16_t debug_index;

static PT_THREAD(serialReadThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalGetData() != HAL_IN_PROGRESS);
 
#ifdef SERIAL_DEBUG
  serial_debug.print("packet count: ");
  serial_debug.println(packet_count, DEC);
  serial_debug.print("dropped packet count: ");
  serial_debug.println(dropped_packet_count, DEC);
  serial_debug.print("timeout count: ");
  serial_debug.println(watchdog_count);
#endif
  if (watchdog_exceeded == true)
  {
    watchdog_count++;    
    watchdog_exceeded = false;
    clear_input = true;
  }
  else
  {
    if ((command_packet_from_serial.sequence_count != last_sequence_count))
    {
      dropped_packet_count++;
#ifdef SERIAL_DEBUG
      serial_debug.print("unexpected sequence count: ");
      serial_debug.println(command_packet_from_serial.sequence_count, DEC);
#endif    
      clear_input = true;
    }
    else
    {
      calc_crc = CRC16.ccitt((uint8_t *)&command_packet_from_serial, sizeof(command_packet_from_serial) - 2);
#ifdef SERIAL_DEBUG
      serial_debug.print("CRC calculated from command packet: ");
      serial_debug.println(calc_crc, DEC);
#endif      
      if (command_packet_from_serial.crc != calc_crc)
      {
        dropped_packet_count++;
#ifdef SERIAL_DEBUG
        serial_debug.print("bad CRC 0x ");
        serial_debug.println(command_packet_from_serial.crc, HEX);
#endif  
        clear_input = true;
      }
      else
      {
        memcpy((void *)&comm.public_command_packet, (void *)&command_packet_from_serial, sizeof(comm.public_command_packet));
#ifdef SERIAL_DEBUG
        serial_debug.print("Successful packet received CRC from command packet: ");
        serial_debug.println(comm.public_command_packet.crc, DEC);
#endif           
      }          
    }
  }  

  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(serialSendThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  comm.public_data_packet.sequence_count = sequence_count;
  comm.public_data_packet.packet_version = PACKET_VERSION;
  comm.public_data_packet.crc = CRC16.ccitt((uint8_t *)&comm.public_data_packet, sizeof(comm.public_data_packet) - 2);
#ifdef SERIAL_DEBUG
  serial_debug.print("CRC calculated and will be sent with next data packet: ");
  serial_debug.println(comm.public_data_packet.crc, DEC);
#endif    
  memcpy((void *)&update_crc_data_packet, (void *)&comm.public_data_packet, sizeof(update_crc_data_packet));
  packet_count++;    
  PT_WAIT_UNTIL(pt, serialHalSendData() != HAL_IN_PROGRESS);
  //}    
  PT_RESTART(pt);
  PT_END(pt);
}

PT_THREAD(serialThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  if (!PT_SCHEDULE(serialSendThreadMain(&serialSendThread))) {
    PT_EXIT(pt);
  }
  
  if (!PT_SCHEDULE(serialReadThreadMain(&serialReadThread))) {
    PT_EXIT(pt);
  }  
  PT_RESTART(pt);
  PT_END(pt);
}

int linkModuleInit(void)
{
  if (serialHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  return MODULE_OK;
}

int linkModuleRun(void)
{
  return (PT_SCHEDULE(serialThreadMain(&serialThread))) ? MODULE_OK : MODULE_FAIL;
}
