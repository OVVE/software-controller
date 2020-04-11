//
// Link Module
//
#include <stdint.h>
#include <FastCRC.h>
#include "../pt/pt.h"
#include "../modules/link.h"
#include "../modules/module.h"
#include "../modules/parameters.h"
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

// update variables for modules to read after good sequence and crc from command packet
void updateFromCommandPacket()
{
  comm.startVentilation = (comm.public_command_packet.mode_value & MODE_START_STOP) != 0x00;

  // check for a conflict - more than one mode - not the best logic going forward
  if (comm.ventilationMode != (MODE_ASSIST)&0x7f || comm.ventilationMode != (MODE_NON_ASSIST)&0x7f || comm.ventilationMode != (MODE_SIM)&0x7f)
  {
    comm.public_data_packet.alarm_bits |= ALARM_UI_MODE_MISMATCH;
  }
  else
  {    
    comm.ventilationMode = comm.public_command_packet.mode_value;
    comm.public_data_packet.alarm_bits = comm.public_data_packet.alarm_bits & ~ALARM_UI_MODE_MISMATCH;
  }
  comm.volumeRequested = comm.public_command_packet.tidal_volume_set;
  comm.respirationRateRequested= comm.public_command_packet.respiratory_rate_set;
  comm.ieRatioRequested = comm.public_command_packet.ie_ratio_set;
  
  // Alarms
  comm.droppedPacketAlarm = (comm.public_command_packet.alarm_bits & ALARM_DROPPED_PACKET) != 0x00;
  comm.crcErrorAlarm = (comm.public_command_packet.alarm_bits & ALARM_CRC_ERROR) != 0x00;  
  comm.unsupportedPacketVersionAlarm = (comm.public_command_packet.alarm_bits & ALARM_PACKET_VERSION) != 0x00;
}

// get data from modules to be sent back to ui. this is called just before sequence count update and crc set
void updateDataPacket()
{
/*
 struct parameters {
  // Variables
  uint8_t  startVentilation;
  uint8_t  ventilationMode;
  uint32_t volumeRequested;
  uint32_t respirationRateRequested;
  uint32_t ieRatioRequested;
  
  // Alarms
  int8_t   parametersInvalidAlarm;
};

extern struct parameters parameters; 
*/
  if (comm.startVentilation)
  {
    comm.public_data_packet.mode_value |= MODE_START_STOP;
  }
  else
  {
    comm.public_data_packet.mode_value = comm.public_data_packet.mode_value & ~MODE_START_STOP;
  }  
}

static PT_THREAD(serialReadThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalGetData() != HAL_IN_PROGRESS);
 
#ifdef SERIAL_DEBUG
if ((packet_count % 100) == 0)
{  
  serial_debug.print("packet count: ");
  serial_debug.println(packet_count, DEC);
  serial_debug.print("dropped packet count: ");
  serial_debug.println(dropped_packet_count, DEC);
  serial_debug.print("timeout count: ");
  serial_debug.println(watchdog_count);
  
  uint8_t databytes[sizeof(update_crc_data_packet)];
  memcpy((void *)&databytes, (void *)&update_crc_data_packet, sizeof(update_crc_data_packet));
  for (debug_index = 0; debug_index < sizeof(update_crc_data_packet); debug_index++)
  {
    serial_debug.print(" ");
    serial_debug.print(databytes[debug_index], HEX);
  }
  serial_debug.println("  done with bytes");
}
#endif
  if (watchdog_exceeded == true)
  {
    watchdog_count++;    
    watchdog_exceeded = false;
    clear_input = true;
    comm.public_data_packet.alarm_bits |= ALARM_DROPPED_PACKET;
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
      comm.public_data_packet.alarm_bits |= ALARM_DROPPED_PACKET;
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
        comm.public_data_packet.alarm_bits |= ALARM_CRC_ERROR;
#ifdef SERIAL_DEBUG
        serial_debug.print("bad CRC 0x ");
        serial_debug.println(command_packet_from_serial.crc, HEX);
#endif  
        clear_input = true;
      }
      else
      {
        comm.public_data_packet.alarm_bits = comm.public_data_packet.alarm_bits & ~ALARM_DROPPED_PACKET;
        comm.public_data_packet.alarm_bits = comm.public_data_packet.alarm_bits & ~ALARM_CRC_ERROR;        
        memcpy((void *)&comm.public_command_packet, (void *)&command_packet_from_serial, sizeof(comm.public_command_packet));
        updateFromCommandPacket();
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
