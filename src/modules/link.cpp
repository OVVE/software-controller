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

bool ready_to_send = true;

static PT_THREAD(serialReadThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalGetData() != HAL_IN_PROGRESS);

  if (watchdog_exceeded == true)
  {
    comm.public_command_packet.alarm_bits |= 1 << ALARM_DROPPED_PACKET;
    watchdog_exceeded = false;
    clear_input = true;
    ready_to_send = true;
  }
  else if ((comm.public_command_packet.sequence_count != last_sequence_count))
  {
#ifdef SERIAL_DEBUG
    Serial1.print("unexpected sequence count: ");
    Serial1.println(comm.public_command_packet.sequence_count, DEC);
#endif    
    comm.public_command_packet.alarm_bits |= 1 << ALARM_CRC_ERROR;
  }
  else if (comm.public_command_packet.crc != CRC16.ccitt((uint8_t *)&comm.public_command_packet, sizeof(comm.public_command_packet) - 2))
  {
    comm.public_command_packet.alarm_bits |= 1 << ALARM_CRC_ERROR;
#ifdef SERIAL_DEBUG
    Serial1.print("bad CRC 0x ");
    Serial1.println(comm.public_command_packet.crc, HEX);
#endif     
  }    
  ready_to_send = true; // serial.cpp will wait for time interval, but go ahead and set this to ready
  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(serialSendThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalSendData() != HAL_IN_PROGRESS);

  if (sequence_count != last_sequence_count) {
    comm.public_data_packet.sequence_count = sequence_count;
    comm.public_data_packet.packet_version = PACKET_VERSION;
    comm.public_data_packet.crc = CRC16.ccitt((uint8_t *)&comm.public_data_packet, sizeof(comm.public_data_packet) - 2);
  }    
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
