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

//data_packet_def data_packet_last_sent;

//incoming_packet_u shared_command_packet; // this should be an extern in other modules that need to read this
command_packet_def command_packet;
bool ready_to_send = true;


int16_t sum_return;
int16_t sum_index;

int16_t sum(uint8_t datap[], uint16_t data_length)
{
  sum_return = 0;
  for (sum_index = 0; sum_index < data_length; sum_index++)
  {
    sum_return += datap[sum_index];
  }
  return (sum_return);
}

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
  //pcommand_packet = (unsigned char *)&command_packet;
  else if ((comm.public_command_packet.sequence_count != last_sequence_count) || (comm.public_command_packet.start_byte != 0xFF))
  {
    comm.public_command_packet.alarm_bits |= 1 << ALARM_CRC_ERROR;
  }
  else if (comm.public_command_packet.crc != sum((uint8_t *)&comm.public_command_packet, sizeof(comm.public_command_packet) - 2))
  {
    //public_command_packet.alarm_bits |= 1 << ALARM_CRC_ERROR;
  }    
  
  ready_to_send = true;
  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(serialSendThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalSendData() != HAL_IN_PROGRESS);

  if (sequence_count != last_sequence_count) {
    comm.public_data_packet.start_byte = 0xFF;
    comm.public_data_packet.sequence_count = sequence_count;
    comm.public_data_packet.packet_version = PACKET_VERSION;
    comm.public_data_packet.crc = sum((uint8_t *)&comm.public_data_packet, sizeof(comm.public_data_packet) - 2);
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
