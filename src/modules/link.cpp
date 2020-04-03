//
// Link Module
//
#include <Arduino.h>
#include "../pt/pt.h"
#include "../modules/link.h"
#include "../modules/module.h"
#include "../hal/serial.h"

// Private Variables
struct link comm;
static struct pt serialThread;
static struct pt serialReadThread;
static struct pt serialSendThread;

command_packet_def public_command_packet;
data_packet_def public_data_packet;
//data_packet_def data_packet_last_sent;

//incoming_packet_u shared_command_packet; // this should be an extern in other modules that need to read this
//incoming_packet_u command_packet_reference; // this is saved in case other modules overwrite the shared copy
command_packet_def command_packet_reference;
command_packet_def command_packet;
bool ready_to_send = true;

#define POLY 0x8408
/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/
uint8_t crc_i;
uint32_t crc_data;
uint32_t crc = 0xffff;

uint16_t crc16(int8_t *data_p, uint16_t length)
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

static PT_THREAD(serialReadThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalGetData() != HAL_IN_PROGRESS);
  // save a copy for other modules, but keep a reference in case the shared copy gets modified
  //memcpy((void *)&public_command_packet, (void *)&incoming_packet.command_packet, sizeof(public_command_packet));
  //memcpy((void *)&command_packet_reference, (void *)&incoming_packet.command_packet, sizeof(command_packet_reference));
  if (watchdog_exceeded == true)
  {
    public_command_packet.alarm_bits |= 1 << ALARM_DROPPED_PACKET;
    watchdog_exceeded = false;
    clear_input = true;
    ready_to_send = true;
  }
  //pcommand_packet = (unsigned char *)&command_packet;
  else if ((public_command_packet.sequence_count != last_sequence_count) || (public_command_packet.start_byte != 0xFF))
  {
    public_command_packet.alarm_bits |= 1 << ALARM_CRC_ERROR;
  }
  //memcpy((void *)&command_packet_reference, (void *)pcommand_packet, sizeof(command_packet_reference)); 
  
  ready_to_send = true;
  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(serialSendThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalSendData() != HAL_IN_PROGRESS);

  if (sequence_count != last_sequence_count) {
    public_data_packet.start_byte = 0xFF;
    public_data_packet.sequence_count = sequence_count;
    public_data_packet.packet_version = PACKET_VERSION;
    public_data_packet.crc = crc16((int8_t *)&public_data_packet, sizeof(public_data_packet) - 2);
    //memcpy((void *)&data_packet_last_sent, (void *)&public_data_packet, sizeof(data_packet_last_sent));
    //public_data_packet.crc = crc16((unsigned int *)&public_data_packet, sizeof(public_data_packet) - 2);    
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
