// TODO: Document what this module does
//
// Link Module
//
#include <stdint.h>

//#define USE_DUE
#ifdef USE_DUE  // this crc16 header will not compile with DUE so use external lib fastcrc
#include <FastCRC.h>
FastCRC16 CRC16;  // this is the class to create crc16 to match the CCITT crc16 that rpi is using
#else
#include <util/crc16.h>
#endif

#include "../pt/pt.h"
#include "../modules/link.h"
#include "../modules/module.h"
#include "../modules/parameters.h"
#include "../modules/control.h"
#include "../modules/sensors.h"
#include "../hal/serial.h"

// Private Variables
struct link comm;
static struct pt serialThread;
static struct pt serialReadThread;
static struct pt serialSendThread;

// Private Variables
uint16_t calc_crc;
command_packet_def command_packet;

uint32_t watchdog_count = 0;
uint32_t dropped_packet_count = 0;
uint32_t packet_count = 0;
uint8_t tmpMode;  // used for setting data for simulation
// this will be used by module/link to send packets
uint16_t sequence_count = 0;      // this is the sequence count stored in the data packet and confirmed in the command packet. wrapping is fine as crc checks are done.
uint16_t last_sequence_count; // what to expect
uint16_t crc_base;

// Public Variables
// shared with serial.cpp
uint8_t data_bytes[sizeof(data_packet_def)];
uint16_t sizeof_data_bytes = sizeof(data_bytes);
uint8_t command_bytes[sizeof(command_packet_def)];
uint16_t sizeof_command_bytes = sizeof(command_bytes);
extern bool watchdog_exceeded;  // timeout waiting for number of received bytes (command packet)
extern bool clear_input;        // if there are issues with command packet data then let serial know to clear the input buffer
extern struct parameters parameters;
extern struct control control;
extern struct sensors sensors;

#ifndef USE_DUE
uint16_t calc_crc_avrlib(unsigned char *bytes, int byteLength)
{
 crc_base = 0xFFFF;

 while(byteLength--)
 {
   crc_base = _crc_xmodem_update(crc_base, (uint16_t)*bytes);
 }
 return(crc_base);
}
#endif

// update variables for modules to read after good sequence and crc from command packet
void updateFromCommandPacket()
{
  comm.startVentilation = (comm.public_command_packet.mode_value & MODE_START_STOP) != 0x00;

  tmpMode = 0x7f & comm.public_command_packet.mode_value;
  
  // check for a conflict - more than one mode - not the best logic going forward
  if (tmpMode != MODE_ASSIST || tmpMode != MODE_NON_ASSIST || tmpMode != MODE_SIM)
  {
    comm.public_data_packet.alarm_bits |= ALARM_UI_MODE_MISMATCH;
  }
  else
  {    
    comm.ventilationMode = tmpMode;
    comm.public_data_packet.alarm_bits = comm.public_data_packet.alarm_bits & ~ALARM_UI_MODE_MISMATCH;
  }
  comm.volumeRequested = comm.public_command_packet.tidal_volume_set;
  comm.respirationRateRequested= comm.public_command_packet.respiratory_rate_set;
  comm.ieRatioRequested = comm.public_command_packet.ie_ratio_set;
//#define SET_VALUES - for testing
#ifdef SET_VALUES
  comm.public_data_packet.tidal_volume_set = comm.public_command_packet.tidal_volume_set;
  comm.public_data_packet.respiratory_rate_set = comm.public_command_packet.respiratory_rate_set;
  comm.public_data_packet.ie_ratio_set = comm.public_command_packet.ie_ratio_set;
  comm.public_data_packet.tidal_volume_measured = 100;
  comm.public_data_packet.respiratory_rate_measured = 200;
  comm.public_data_packet.ie_ratio_set = 300; 
#endif  
  
  // Alarms
  comm.droppedPacketAlarm = (comm.public_command_packet.alarm_bits & ALARM_DROPPED_PACKET) != 0x00;
  comm.crcErrorAlarm = (comm.public_command_packet.alarm_bits & ALARM_CRC_ERROR) != 0x00;  
  comm.unsupportedPacketVersionAlarm = (comm.public_command_packet.alarm_bits & ALARM_PACKET_VERSION) != 0x00;
}

// get data from modules to be sent back to ui. this is called just before sequence count update and crc set
void updateDataPacket()
{
  // only set the lower 7 bits of mode value
  comm.public_data_packet.mode_value = 0x7f & parameters.ventilationMode;
  if (parameters.startVentilation)
  {
    comm.public_data_packet.mode_value |= MODE_START_STOP;
  }
  else
  {
    comm.public_data_packet.mode_value &= ~MODE_START_STOP;
  }
  // could not find 
  //    respiratory_rate_measured
  //    tidal_volume_measured
  //    battery_level
  //
  comm.public_data_packet.tidal_volume_set = parameters.volumeRequested;
  comm.public_data_packet.respiratory_rate_set = parameters.respirationRateRequested;  // same field on control structure
  comm.public_data_packet.ie_ratio_set = parameters.ieRatioRequested;
  
  comm.public_data_packet.control_state = control.state;
  // should we use the one from parameters or this one
  comm.public_data_packet.respiratory_rate_set = control.respirationRateRequested; // // should we use the one from parameters or this one
  
  comm.public_data_packet.ie_ratio_measured = control.ieRatioMeasured;
  
  comm.public_data_packet.plateau_value_measurement = sensors.plateauPressure;
  
  comm.public_data_packet.pressure_measured = sensors.currentPressure;  
  
  comm.public_data_packet.peak_pressure_measured = sensors.peakPressure;
  comm.public_data_packet.peep_value_measured = sensors.peepPressure;

  comm.public_data_packet.volume_in_measured = sensors.volumeIn;
  comm.public_data_packet.volume_out_measured = sensors.volumeOut;  
  comm.public_data_packet.volume_rate_measured = sensors.volumePerMinute; 
  comm.public_data_packet.flow_measured = sensors.currentFlow; 
/*


typedef struct data_packet_def {
  uint16_t sequence_count;            // bytes 0 - 1 - rpi unsigned short int
  uint8_t packet_version;             // byte 2      - rpi unsigned char
  uint8_t mode_value;                 // byte 3      - rpi unsige char
  uint32_t respiratory_rate_measured; // bytes 4 - 7 - rpi unsigned int
  uint32_t respiratory_rate_set;      // bytes 8 - 11
  uint32_t tidal_volume_measured;     // bytes 12 - 15
  uint32_t tidal_volume_set;          // bytes 16 - 19
  uint32_t ie_ratio_measured;         // bytes 20 - 23
  uint32_t ie_ratio_set;              // bytes 24 - 27
  uint32_t peep_value_measured;       // bytes 28 - 31
  uint32_t peak_pressure_measured;    // bytes 32 - 35
  uint32_t plateau_value_measurement; // bytes 36 - 39
  uint32_t pressure_measured;         // bytes 40 - 43
  uint32_t flow_measured;             // bytes 44 - 47
  uint32_t volume_in_measured;        // bytes 48 - 51
  uint32_t volume_out_measured;       // bytes 52 - 55
  uint32_t volume_rate_measured;      // bytes 56 - 59
  uint8_t control_state;              // byte 60       - rpi unsigned char
  uint8_t battery_level;              // byte 61
  uint16_t reserved;                  // bytes 62 - 63 - rpi unsigned int
  uint32_t alarm_bits;                // bytes 64 - 67
  uint16_t crc;                       // bytes 68 - 69 - rpi unsigned short int
}__attribute__((packed));  
*/
/*
struct sensors {
  // Variables
  int32_t currentFlow;
  int32_t currentVolume;
  int32_t volumeIn;
  int32_t volumeOut;
  int32_t volumePerMinute;
  int32_t currentPressure;
  int32_t peakPressure;
  int32_t plateauPressure;
  int32_t peepPressure;
  bool    inhalationDetected;
  bool    exhalationDetected;
  
  // Alarms
  int8_t  onBatteryAlarm;
  int8_t  lowBatteryAlarm;
  int8_t  highPressureAlarm;
  int8_t  lowPressureAlarm;
  int8_t  lowVolumeAlarm;
  int8_t  apneaAlarm;
};
*/

/*  
struct control {
  // Variables
  uint8_t  state;
  uint32_t respirationRateRequested;
  uint32_t ieRatioMeasured;
  uint32_t breathCount;
  
  // Alarms
  int8_t   breathTimeoutAlarm;
  int8_t   unknownStateAlarm;
};
*/
}

static PT_THREAD(serialReadThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalGetData() != HAL_IN_PROGRESS);
  
  if (watchdog_exceeded == true)
  {
    watchdog_count++;    
    watchdog_exceeded = false;
    clear_input = true;
    comm.public_data_packet.alarm_bits |= ALARM_DROPPED_PACKET;
  }
  else
  {
    memcpy((void *)&command_packet, (void *)command_bytes, sizeof(command_packet));
    if ((command_packet.sequence_count != last_sequence_count))
    {
      dropped_packet_count++;
#ifdef SERIAL_DEBUG
      SERIAL_DEBUG.print("unexpected sequence count: ");
      SERIAL_DEBUG.println(command_packet.sequence_count, DEC);
#endif    
      clear_input = true;
      comm.public_data_packet.alarm_bits |= ALARM_DROPPED_PACKET;
    }
    else
    {
#ifdef USE_DUE
      calc_crc = CRC16.ccitt((uint8_t *)&command_packet, sizeof(command_packet) - 2);
#else 
      calc_crc = calc_crc_avrlib((uint8_t *)&command_packet, sizeof(command_packet) - 2);  
#endif
 
#ifdef SERIAL_DEBUG
      SERIAL_DEBUG.print("CRC calculated from command packet: ");
      SERIAL_DEBUG.println(calc_crc, DEC);
#endif      
      if (command_packet.crc != calc_crc)
      {
        dropped_packet_count++;
        comm.public_data_packet.alarm_bits |= ALARM_CRC_ERROR;
#ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print("bad CRC 0x ");
        SERIAL_DEBUG.println(command_packet.crc, HEX);
#endif  
        clear_input = true;
      }
      else
      {
        comm.public_data_packet.alarm_bits = comm.public_data_packet.alarm_bits & ~ALARM_DROPPED_PACKET;
        comm.public_data_packet.alarm_bits = comm.public_data_packet.alarm_bits & ~ALARM_CRC_ERROR;        
        memcpy((void *)&comm.public_command_packet, (void *)&command_packet, sizeof(comm.public_command_packet));
        updateFromCommandPacket();
#ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print("Successful packet received CRC from command packet: ");
        SERIAL_DEBUG.println(comm.public_command_packet.crc, DEC);
#endif           
      }          
    }
  }  
#ifdef SERIAL_DEBUG
  SERIAL_DEBUG.print("packet count: ");
  SERIAL_DEBUG.println(packet_count, DEC);
  SERIAL_DEBUG.print("dropped packet count: ");
  SERIAL_DEBUG.println(dropped_packet_count, DEC);
  SERIAL_DEBUG.print("timeout count: ");
  SERIAL_DEBUG.println(watchdog_count);
  SERIAL_DEBUG.println(" ");
#endif
  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(serialSendThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  last_sequence_count = sequence_count;  // set this for the next read as the sequence count will advance and wait for read to complete
  ++sequence_count;
  updateDataPacket();
  comm.public_data_packet.sequence_count = sequence_count;
  comm.public_data_packet.packet_version = PACKET_VERSION;
#ifdef USE_DUE
   comm.public_data_packet.crc = CRC16.ccitt((uint8_t *)&comm.public_data_packet, sizeof(comm.public_data_packet) - 2);   
#else  
  comm.public_data_packet.crc = calc_crc_avrlib((uint8_t *)&comm.public_data_packet, sizeof(comm.public_data_packet) - 2);
#endif

#ifdef SERIAL_DEBUG
  SERIAL_DEBUG.print("Microcontroller sending sequence: ");
  SERIAL_DEBUG.print(sequence_count, DEC);
  SERIAL_DEBUG.print(", CRC: ");
  SERIAL_DEBUG.println(comm.public_data_packet.crc, DEC);
#endif    
  memcpy((void *)&data_bytes, (void *)&comm.public_data_packet, sizeof(data_bytes));
  packet_count++; 
  PT_WAIT_UNTIL(pt, serialHalSendData() != HAL_IN_PROGRESS);
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
  PT_INIT(&serialSendThread);
  PT_INIT(&serialReadThread);
}

int linkModuleRun(void)
{
  return (PT_SCHEDULE(serialThreadMain(&serialThread))) ? MODULE_OK : MODULE_FAIL;
}
