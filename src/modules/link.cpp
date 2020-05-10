// TODO: Document what this module does
//
// Link Module
//
#include <stdint.h>

#include "../config.h"
#include "../pt/pt.h"
#include "../modules/link.h"
#include "../modules/module.h"
#include "../modules/parameters.h"
#include "../modules/control.h"
#include "../modules/sensors.h"
#include "../hal/serial.h"

#ifdef DEBUG_LINK
#define DEBUG_MODULE "link"
#include "../util/debug.h"
#endif


#define LINK_PACKET_TYPE_PUBLICDATA_PACKET 0x01
#define LINK_PACKET_TYPE_COMMAND_PACKET 0x02

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
extern struct parameters parameters;
extern struct control control;
extern struct sensors sensors;



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
 
  public_data_packet.battery_level = 0; // TBD set to real value when available
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
  public_data_packet.respiratory_rate_set = parameters.respirationRateRequested;
  public_data_packet.ie_ratio_set = parameters.ieRatioRequested; // comm.ieRatioRequested; 
  
  public_data_packet.control_state = control.state;
  public_data_packet.ie_ratio_measured = control.ieRatioMeasured;
  public_data_packet.respiratory_rate_measured = control.respirationRateMeasured;
  
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



int linkProcessPacket(uint8_t packetType, uint8_t packetLen, uint16_t sequenceNumber, uint8_t* data)
{
  static command_packet_def command_packet;
  static uint16_t lastSequence=-1;
  if ((packetType==LINK_PACKET_TYPE_COMMAND_PACKET) && (packetLen==sizeof(command_packet)))
  {
    memcpy((void *)&command_packet, (void *)data, sizeof(command_packet));
    if ( (sequenceNumber != lastSequence+1) && (lastSequence!=-1))
    {
      serial_statistics.commandPacketSequenceWrongCnt++;
      public_data_packet.alarm_bits |= ALARM_DROPPED_PACKET;
    } else
      {
        public_data_packet.alarm_bits = public_data_packet.alarm_bits & ~ALARM_DROPPED_PACKET;
        public_data_packet.alarm_bits = public_data_packet.alarm_bits & ~ALARM_CRC_ERROR;        
        memcpy((void *)&public_command_packet, (void *)&command_packet, sizeof(public_command_packet));
        updateFromCommandPacket();
      }          
    lastSequence=sequenceNumber;
  }
}

static PT_THREAD(serialReadThreadMain(struct pt* pt))
{
  static uint16_t calc_crc;
  static uint32_t dropped_packet_count = 0;
  static uint32_t watchdog_count = 0;
  
  PT_BEGIN(pt);

  serialHalHandleRx(linkProcessPacket);

  
  PT_RESTART(pt);
  PT_END(pt);
}

#define LINK_PUBLIC_DATA_TIMEOUT 100 // in ms
#define LINK_DEBUG_MSG_TIMEOUT 20

static PT_THREAD(serialSendThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  static long lastPublicDataMillis=0;
  static long lastDebugMsgMillis=0;
  //continously process tx data
  serialHalSendData();

  //check if we need to send the next public data packet
  if (millis()>lastPublicDataMillis+LINK_PUBLIC_DATA_TIMEOUT)
  {
    lastPublicDataMillis=millis();
    sequence_count++;
    last_sequence_count = sequence_count; 
    updateDataPacket();

    serialHalSendPacket(LINK_PACKET_TYPE_PUBLICDATA_PACKET,sizeof(public_data_packet),sequence_count,(uint8_t*)&public_data_packet);

    //directly start sending if possible
    serialHalSendData();
  }
//check if we need to send the next public data packet
  if (millis()>lastDebugMsgMillis+LINK_DEBUG_MSG_TIMEOUT)
  {
    lastDebugMsgMillis=millis();
    

  }
  
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

  DEBUG_PRINT_EVERY(10000,"Serial Stats: TX(OK,FAIL,SEQERR): %li,%li RX (OK,FAIL,SEQ): %li,%li,%li ",serial_statistics.packetsCntSentOk,serial_statistics.packetsCntSentBufferOverFlow,serial_statistics.packetsCntReceivedOk,serial_statistics.packetsCntHeaderSyncFailed+serial_statistics.packetsCntWrongCrc+serial_statistics.packetsCntWrongLength+serial_statistics.packetsCntWrongVersion,serial_statistics.commandPacketSequenceWrongCnt);
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
