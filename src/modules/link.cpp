// TODO: Document what this module does
//
// Link Module
//
#include <stdint.h>

//#define USE_FAST_CRC
#ifdef USE_FAST_CRC  // utils/crc16.h not found for DUE and the function used here to wrap it is causing timeouts
                     // the header file has functions defined using inline assembly. It works fine on Mega with a 
                     // standalone program, but using it with the controller is always taking too much time.
                     
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
uint16_t sequence_count = 0;      // this is the sequence count stored in the data packet and confirmed in the command packet. wrapping is fine as crc checks are done.
uint16_t last_sequence_count; // what to expect for next sequence, set in write and checked in read
data_packet_def public_data_packet;
command_packet_def public_command_packet;


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

#ifndef USE_FAST_CRC

uint16_t calc_crc_avrlib(unsigned char *bytes, int byteLength)
{
 static uint16_t crc_base;
 crc_base = 0xFFFF;

 while(byteLength--)
 {
   crc_base = _crc_xmodem_update(crc_base, (uint8_t)*bytes++);
 }
 return(crc_base);
}
#endif

// update variables for modules to read after good sequence and crc from command packet
void updateFromCommandPacket()
{
  static uint8_t tmpMode;  // used for setting bits
  comm.startVentilation = (public_command_packet.mode_value & MODE_START_STOP) != 0x00;

  tmpMode = 0x7f & public_command_packet.mode_value;
  
  // check for a conflict - more than one mode - not the best logic going forward
  if (tmpMode != MODE_ASSIST && tmpMode != MODE_NON_ASSIST && tmpMode != MODE_SIM)
  {
    public_data_packet.alarm_bits |= ALARM_UI_MODE_MISMATCH;
  }
  else
  {    
    comm.ventilationMode = tmpMode;
    public_data_packet.alarm_bits = public_data_packet.alarm_bits & ~ALARM_UI_MODE_MISMATCH;
  }
  comm.volumeRequested = public_command_packet.tidal_volume_set;
  comm.respirationRateRequested= public_command_packet.respiratory_rate_set;
  comm.ieRatioRequested = public_command_packet.ie_ratio_set;
 
  public_data_packet.battery_status = 0; // TBD set to real value when available
#ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print("mode bits: 0x");
        SERIAL_DEBUG.println(public_command_packet.mode_value, HEX);
        SERIAL_DEBUG.print("start/stop value: 0x");
        SERIAL_DEBUG.println(comm.startVentilation, HEX);
        SERIAL_DEBUG.print("mode value: 0x");
        SERIAL_DEBUG.println(comm.ventilationMode, HEX);        
#endif  
  // Alarms
  comm.droppedPacketAlarm = (public_command_packet.alarm_bits & ALARM_DROPPED_PACKET) != 0x00;
  comm.crcErrorAlarm = (public_command_packet.alarm_bits & ALARM_CRC_ERROR) != 0x00;  
  comm.unsupportedPacketVersionAlarm = (public_command_packet.alarm_bits & ALARM_PACKET_VERSION) != 0x00;
}

// get data from modules to be sent back to ui. this is called just before sequence count update and crc set
void updateDataPacket()
{
  // only set the lower 7 bits of mode value
  public_data_packet.mode_value = 0x7f & parameters.ventilationMode;
  if (parameters.startVentilation)
  {
    public_data_packet.mode_value |= MODE_START_STOP;
  }
  else
  {
    public_data_packet.mode_value &= ~MODE_START_STOP;
  }
  // could not find 
  //    respiratory_rate_measured
  //    tidal_volume_measured
  //    battery_level
  //
  public_data_packet.tidal_volume_set = parameters.volumeRequested;
  public_data_packet.tidal_volume_measured = sensors.currentVolume;
  public_data_packet.respiratory_rate_set = parameters.respirationRateRequested;  // same field on control structure
  public_data_packet.ie_ratio_set = parameters.ieRatioRequested; // comm.ieRatioRequested; 
  
  public_data_packet.control_state = control.state;
  // should we use the one from parameters or this one
  public_data_packet.respiratory_rate_set = control.respirationRateRequested; // // should we use the one from parameters or this one
  
  public_data_packet.ie_ratio_measured = control.ieRatioMeasured;
  //public_data_packet.respiratory_rate_measured = control.  // could not find the field for this
  
  // readings from sensor module
  public_data_packet.plateau_value_measurement = sensors.plateauPressure;
  public_data_packet.pressure_measured = sensors.currentPressure;  
  public_data_packet.peak_pressure_measured = sensors.peakPressure;
  public_data_packet.peep_value_measured = sensors.peepPressure;
  public_data_packet.tidal_volume_measured = sensors.currentVolume;
  public_data_packet.volume_in_measured = sensors.volumeIn;
  public_data_packet.volume_out_measured = sensors.volumeOut;  
  public_data_packet.volume_rate_measured = sensors.volumePerMinute; 
  public_data_packet.flow_measured = sensors.currentFlow; 
//#define SET_VALUES // - for testing
#ifdef SET_VALUES
  public_data_packet.mode_value = public_command_packet.mode_value; //comm.ventilationMode;
  public_data_packet.tidal_volume_set = public_command_packet.tidal_volume_set;
  public_data_packet.respiratory_rate_set = public_command_packet.respiratory_rate_set;
  public_data_packet.ie_ratio_set = public_command_packet.ie_ratio_set;
  public_data_packet.tidal_volume_measured = 475;
  public_data_packet.respiratory_rate_measured = 35;
  public_data_packet.peep_value_measured = 426;
  public_data_packet.peak_pressure_measured =  527;
  public_data_packet.pressure_measured  =  980;
  public_data_packet.flow_measured = 980;
  public_data_packet.plateau_value_measurement  = 150;
  public_data_packet.volume_in_measured = 18;
  public_data_packet.volume_out_measured = 13;
  public_data_packet.volume_rate_measured = 12;
#endif  

  
}

static PT_THREAD(serialReadThreadMain(struct pt* pt))
{
  static uint16_t calc_crc;
  static uint32_t dropped_packet_count = 0;
  static uint32_t watchdog_count = 0;
  static command_packet_def command_packet;
  
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, serialHalGetData() != HAL_IN_PROGRESS);
  
  if (watchdog_exceeded == true)
  {
    watchdog_count++;    
    watchdog_exceeded = false;
    clear_input = true;
    public_data_packet.alarm_bits |= ALARM_DROPPED_PACKET;
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
      public_data_packet.alarm_bits |= ALARM_DROPPED_PACKET;
    }
    else
    {
#ifdef USE_FAST_CRC
      calc_crc = CRC16.ccitt((uint8_t *)&command_packet, sizeof(command_packet) - 2);
#else 
      calc_crc = calc_crc_avrlib((uint8_t *)&command_packet, sizeof(command_packet) - 2); 
#endif 
      if (command_packet.crc != calc_crc)
      {
        dropped_packet_count++;
        public_data_packet.alarm_bits |= ALARM_CRC_ERROR;
#ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print("bad CRC 0x ");
        SERIAL_DEBUG.println(command_packet.crc, HEX);
#endif  
        clear_input = true;
      }
      else
      {
        public_data_packet.alarm_bits = public_data_packet.alarm_bits & ~ALARM_DROPPED_PACKET;
        public_data_packet.alarm_bits = public_data_packet.alarm_bits & ~ALARM_CRC_ERROR;        
        memcpy((void *)&public_command_packet, (void *)&command_packet, sizeof(public_command_packet));
        updateFromCommandPacket();
#ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print("Successful packet received CRC from command packet: ");
        SERIAL_DEBUG.println(public_command_packet.crc, DEC);
#endif           
      }          
    }
  }  
#ifdef SERIAL_DEBUG
  SERIAL_DEBUG.print("packet count: ");
  SERIAL_DEBUG.println(sequence_count, DEC);
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
  public_data_packet.sequence_count = sequence_count;
  public_data_packet.packet_version = PACKET_VERSION;
#ifdef USE_FAST_CRC
   public_data_packet.crc = CRC16.ccitt((uint8_t *)&public_data_packet, sizeof(public_data_packet) - 2);   
#else  
  public_data_packet.crc = calc_crc_avrlib((uint8_t *)&public_data_packet, sizeof(public_data_packet) - 2);
#endif

#ifdef SERIAL_DEBUG
  SERIAL_DEBUG.print("Microcontroller sending sequence: ");
  SERIAL_DEBUG.print(sequence_count, DEC);
  SERIAL_DEBUG.print(", CRC: ");
  SERIAL_DEBUG.println(public_data_packet.crc, DEC);
#endif    
  memcpy((void *)&data_bytes, (void *)&public_data_packet, sizeof(data_bytes));
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

  return MODULE_OK;
}

int linkModuleRun(void)
{
  return (PT_SCHEDULE(serialThreadMain(&serialThread))) ? MODULE_OK : MODULE_FAIL;
}
