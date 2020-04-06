//
// Link Module
//
#include <Arduino.h>
#include <FastCRC.h>
#include "../pt/pt.h"
#include "../modules/link.h"
#include "../modules/module.h"
#include "../hal/serial.h"

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

#ifdef SERIAL_DEBUG
extern HardwareSerial &serial_debug;
#endif

static PT_THREAD(serialReadThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalGetData() != HAL_IN_PROGRESS);

  if (watchdog_exceeded == true)
  {
    comm.public_command_packet.alarm_bits |= 1 << ALARM_DROPPED_PACKET;
    watchdog_exceeded = false;
    clear_input = true;
  }
  else
  {
    if ((comm.public_command_packet.sequence_count != sequence_count))
    {
#ifdef SERIAL_DEBUG
      serial_debug.print("unexpected sequence count: ");
      serial_debug.println(comm.public_command_packet.sequence_count, DEC);
#endif    
      comm.public_command_packet.alarm_bits |= 1 << ALARM_CRC_ERROR;
      clear_input = true;
    }
    else
    {
      calc_crc = CRC16.ccitt((uint8_t *)&comm.public_command_packet, sizeof(comm.public_command_packet) - 2);
#ifdef SERIAL_DEBUG
      serial_debug.print("CRC calculated from command packet: ");
      serial_debug.println(calc_crc, DEC);
#endif      
      if (comm.public_command_packet.crc != calc_crc)
      {
        comm.public_command_packet.alarm_bits |= 1 << ALARM_CRC_ERROR;
#ifdef SERIAL_DEBUG
        serial_debug.print("bad CRC 0x ");
        serial_debug.println(comm.public_command_packet.crc, HEX);
#endif  
        clear_input = true;
      }
      else
      {
          comm.public_command_packet.alarm_bits &= ~(1 << ALARM_CRC_ERROR); 
      }          
    }
  }  
#ifdef SERIAL_DEBUG
  serial_debug.print("CRC from data packet: ");
  serial_debug.println(comm.public_command_packet.crc, DEC);
  serial_debug.print("CRC calculated from data packet: ");
  serial_debug.println(CRC16.ccitt((uint8_t *)&comm.public_command_packet, sizeof(comm.public_command_packet) - 2));
#endif 
  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(serialSendThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalSendData() != HAL_IN_PROGRESS);
    memcpy((void *)&update_crc_data_packet, (void *)&comm.public_data_packet, sizeof(update_crc_data_packet));
    update_crc_data_packet.sequence_count = sequence_count;
    update_crc_data_packet.packet_version = PACKET_VERSION;
    update_crc_data_packet.crc = CRC16.ccitt((uint8_t *)&comm.public_data_packet, sizeof(comm.public_data_packet) - 2);
  //}    
  PT_RESTART(pt);
  PT_END(pt);
}

PT_THREAD(serialThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  if (!PT_SCHEDULE(serialReadThreadMain(&serialReadThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(serialSendThreadMain(&serialSendThread))) {
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
