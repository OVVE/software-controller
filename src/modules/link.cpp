#include <stdint.h>
#include <stdbool.h>

#include <Arduino.h>

#include "../config.h"
#include "../pt/pt.h"
#include "../modules/link.h"
#include "../modules/module.h"
#include "../modules/parameters.h"
#include "../modules/control.h"
#include "../modules/sensors.h"
#include "../hal/serial.h"

#define LOG_MODULE "link"
#define LOG_LEVEL  LOG_LINK_MODULE
#include "../util/log.h"

// TODO: look into a nicer way
extern struct alarm estopAlarm;
extern bool powerOff;

//how many ms until we sound an alarm that we didn't hear anything from the UI
#define UI_PACKET_RECEIVE_TIMEOUT 2000 

// Private Variables
struct link comm;
static struct pt serialThread;
static struct pt serialReadThread;
static struct pt serialSendThread;
data_packet_def public_data_packet;
command_packet_def public_command_packet;

uint32_t ConsecutiveDroppedPacketCount=0;

// Public Variables
// shared with serial.cpp
extern struct parameters parameters;
extern struct control control;
extern struct sensors sensors;


static uint32_t previousLocalAlarms;
static uint32_t previousRemoteAlarms;

extern struct parameters parameters;
extern struct control control;
extern struct sensors sensors;

static long lastUiPacketReceivedTime=0;

uint32_t lastDataPacketAlarmBits;

static struct alarmProperties onCommunicationFailureAlarmProperties = {
  .priority = ALARM_PRIORITY_HIGH,
  .preventWatchdog = false,
  .suppressionTimeout = (120 SEC),
};

void handleUIAlarms()
{
  if (public_command_packet.alarm_bits & ALARM_ECU_POWER_LOSS && lastDataPacketAlarmBits & ALARM_ECU_POWER_LOSS)
  {
    alarmSuppress(&sensors.onBatteryAlarm);
  }
  if (public_command_packet.alarm_bits & ALARM_ECU_LOW_BATTERY && lastDataPacketAlarmBits & ALARM_ECU_LOW_BATTERY)
  {
    alarmSuppress(&sensors.lowBatteryAlarm);
  }
  if (public_command_packet.alarm_bits & ALARM_ECU_BAD_PRESSURE_SENSOR && lastDataPacketAlarmBits & ALARM_ECU_BAD_PRESSURE_SENSOR)
  {
    alarmSuppress(&sensors.badPressureSensorAlarm);
  }
  if (public_command_packet.alarm_bits & ALARM_ECU_BAD_FLOW_SENSOR && lastDataPacketAlarmBits & ALARM_ECU_BAD_FLOW_SENSOR)
  {
    alarmSuppress(&sensors.badAirflowSensorAlarm);
  }
  if (public_command_packet.alarm_bits & ALARM_ECU_HIGH_PRESSURE && lastDataPacketAlarmBits & ALARM_ECU_HIGH_PRESSURE)
  {
    alarmSuppress(&sensors.highPressureAlarm);
  } 
  if (public_command_packet.alarm_bits & ALARM_ECU_LOW_PRESSURE && lastDataPacketAlarmBits & ALARM_ECU_LOW_PRESSURE)
  {
    alarmSuppress(&sensors.lowPressureAlarm);
  }
  if (public_command_packet.alarm_bits & ALARM_ECU_CONTINUOUS_PRESSURE && lastDataPacketAlarmBits & ALARM_ECU_CONTINUOUS_PRESSURE)
  {
    alarmSuppress(&sensors.continuousPressureAlarm);
  }   
  if (public_command_packet.alarm_bits & ALARM_ECU_HIGH_VOLUME && lastDataPacketAlarmBits & ALARM_ECU_HIGH_VOLUME)
  {
    alarmSuppress(&sensors.highVolumeAlarm);
  } 
  if (public_command_packet.alarm_bits & ALARM_ECU_LOW_VOLUME && lastDataPacketAlarmBits & ALARM_ECU_LOW_VOLUME)
  {
    alarmSuppress(&sensors.lowVolumeAlarm);
  } 
  if (public_command_packet.alarm_bits & ALARM_ECU_HIGH_RESPIRATORY_RATE && lastDataPacketAlarmBits & ALARM_ECU_HIGH_RESPIRATORY_RATE)
  {
    alarmSuppress(&sensors.highRespiratoryRateAlarm);
  } 
  if (public_command_packet.alarm_bits & ALARM_ECU_LOW_RESPIRATORY_RATE && lastDataPacketAlarmBits & ALARM_ECU_LOW_RESPIRATORY_RATE)
  {
    alarmSuppress(&sensors.lowRespiratoryRateAlarm);
  } 
  if (public_command_packet.alarm_bits & ALARM_ECU_COMMUNICATION_FAILURE && lastDataPacketAlarmBits & ALARM_ECU_COMMUNICATION_FAILURE)
  {
    alarmSuppress(&comm.onCommunicationFailureAlarm);
  }  
}

void setDataPacketAlarmBits()
{
  // control.h defines this but not used   struct alarm unknownStateAlarm;
  // parameters.h defines this but not used struct alarm parametersInvalidAlarm;
  // ALARM_ECU_POWER_LOSS
  // ALARM_ECU_HARDWARE_FAILURE
  // ALARM_ECU_ESTOP_PRESSED
  // what alarm bit - public_data_packet.alarm_bits = alarmGet(&sensors.onBatteryAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU... : public_data_packet.alarm_bits &= ~ALARM_ECU;
  
  public_data_packet.alarm_bits = alarmGet(&sensors.onBatteryAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_POWER_LOSS : public_data_packet.alarm_bits &= ~ALARM_ECU_POWER_LOSS;
  public_data_packet.alarm_bits = alarmGet(&sensors.lowBatteryAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_LOW_BATTERY : public_data_packet.alarm_bits &= ~ALARM_ECU_LOW_BATTERY;
  public_data_packet.alarm_bits = alarmGet(&sensors.badPressureSensorAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_BAD_PRESSURE_SENSOR : public_data_packet.alarm_bits &= ~ALARM_ECU_BAD_PRESSURE_SENSOR;
  public_data_packet.alarm_bits = alarmGet(&sensors.badAirflowSensorAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_BAD_FLOW_SENSOR : public_data_packet.alarm_bits &= ~ALARM_ECU_BAD_FLOW_SENSOR;
  public_data_packet.alarm_bits = alarmGet(&estopAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_ESTOP_PRESSED : public_data_packet.alarm_bits &= ~ALARM_ECU_ESTOP_PRESSED;
  public_data_packet.alarm_bits = alarmGet(&sensors.highPressureAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_HIGH_PRESSURE : public_data_packet.alarm_bits &= ~ALARM_ECU_HIGH_PRESSURE;
  public_data_packet.alarm_bits = alarmGet(&sensors.lowPressureAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_LOW_PRESSURE : public_data_packet.alarm_bits &= ~ALARM_ECU_LOW_PRESSURE;
  public_data_packet.alarm_bits = alarmGet(&sensors.continuousPressureAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_CONTINUOUS_PRESSURE : public_data_packet.alarm_bits &= ~ALARM_ECU_CONTINUOUS_PRESSURE;
  public_data_packet.alarm_bits = alarmGet(&sensors.highVolumeAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_HIGH_VOLUME : public_data_packet.alarm_bits &= ~ALARM_ECU_HIGH_VOLUME;
  public_data_packet.alarm_bits = alarmGet(&sensors.lowVolumeAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_LOW_VOLUME : public_data_packet.alarm_bits &= ~ALARM_ECU_LOW_VOLUME; 
  public_data_packet.alarm_bits = alarmGet(&sensors.highRespiratoryRateAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_HIGH_RESPIRATORY_RATE : public_data_packet.alarm_bits &= ~ALARM_ECU_HIGH_RESPIRATORY_RATE;
  public_data_packet.alarm_bits = alarmGet(&sensors.lowRespiratoryRateAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_LOW_RESPIRATORY_RATE : public_data_packet.alarm_bits &= ~ALARM_ECU_LOW_RESPIRATORY_RATE;
  public_data_packet.alarm_bits = alarmGet(&comm.onCommunicationFailureAlarm) ? public_data_packet.alarm_bits |= ALARM_ECU_COMMUNICATION_FAILURE : public_data_packet.alarm_bits &= ~ALARM_ECU_COMMUNICATION_FAILURE;
  lastDataPacketAlarmBits = public_data_packet.alarm_bits;
}

// update variables for modules to read after good sequence and crc from command packet
void updateFromCommandPacket()
{
  static uint8_t tmpMode;  // used for setting bits
  

  comm.startVentilation = (public_command_packet.command & COMMAND_BIT_START) != 0x00;
  comm.powerOff = (public_command_packet.command & COMMAND_BIT_POWEROFF) != 0x00;
  comm.calibrationStep = (public_command_packet.command & COMMAND_BIT_CALIBRATIONSTEP) != 0x00;
  tmpMode = public_command_packet.mode_value;
  
  if (tmpMode != MODE_VC_CMV && tmpMode != MODE_SIMV)
  {
    //public_data_packet.alarm_bits |= ALARM_UI_SETPOINT_MISMATCH;
  }
  else
  {    
    comm.ventilationMode = tmpMode;
    //public_data_packet.alarm_bits = public_data_packet.alarm_bits & ~ALARM_UI_SETPOINT_MISMATCH;
  }

  comm.respirationRateRequested= public_command_packet.respiratory_rate_set;
  comm.volumeRequested = public_command_packet.tidal_volume_set;
  comm.ieRatioRequested = public_command_packet.ie_ratio_set;
  comm.pressureRequested = public_command_packet.pressure_set;
  comm.highPressureLimit = public_command_packet.high_pressure_limit_set;
  comm.lowPressureLimit = public_command_packet.low_pressure_limit_set; 
  comm.highVolumeLimit = public_command_packet.high_volume_limit_set;
  comm.lowVolumeLimit = public_command_packet.low_volume_limit_set;
  comm.highPressureLimit = public_command_packet.high_pressure_limit_set;
  comm.lowPressureLimit = public_command_packet.low_pressure_limit_set;   
  comm.highRespiratoryRateLimit = public_command_packet.high_respiratory_rate_limit_set;
  comm.lowRespiratoryRateLimit = public_command_packet.low_respiratory_rate_limit_set;
 
  public_data_packet.battery_status = 0; // TBD set to real value when available
  // Alarms
  handleUIAlarms();
}

// get data from modules to be sent back to ui. this is called just before sequence count update and crc set
void updateDataPacket()
{
  
  // only set the lower 7 bits of mode value
  //public_data_packet.control_state = 0x7f & parameters.ventilationMode;
  public_data_packet.mode_value = parameters.ventilationMode;
  public_data_packet.control_state = control.state;
  if (parameters.startVentilation)
  {
    public_data_packet.control_state |= CONTROLLER_BIT_START;
  }
  else
  {
    public_data_packet.control_state &= ~CONTROLLER_BIT_START;
  }
  
  setDataPacketAlarmBits();
  
  public_data_packet.battery_status = (0x7f & sensors.batteryPercent) | (sensors.batteryCharging << 7);

  public_data_packet.respiratory_rate_set = parameters.respirationRateRequested;  // same field on control structure

  public_data_packet.respiratory_rate_measured = control.respirationRateMeasured;

  public_data_packet.tidal_volume_set = parameters.volumeRequested;
  public_data_packet.tidal_volume_measured = sensors.currentVolume;
  
  public_data_packet.ie_ratio_set = parameters.ieRatioRequested;
  public_data_packet.ie_ratio_measured = control.ieRatioMeasured;
  
  public_data_packet.peep_value_measured = sensors.peepPressure; 
  public_data_packet.peak_pressure_measured = sensors.peakPressure;
  public_data_packet.plateau_value_measurement = sensors.plateauPressure;
  
  public_data_packet.pressure_set = parameters.pressureRequested;
  public_data_packet.pressure_measured = sensors.currentPressure;

  public_data_packet.flow_measured = sensors.currentFlow; 

  public_data_packet.volume_in_measured = sensors.volumeIn;
  public_data_packet.volume_out_measured = sensors.volumeOut;  
  public_data_packet.volume_rate_measured = sensors.minuteVolume;

  public_data_packet.high_pressure_limit_set = parameters.highPressureLimit; 
  public_data_packet.low_pressure_limit_set = parameters.lowPressureLimit;   

  public_data_packet.high_volume_limit_set = parameters.highVolumeLimit; 
  public_data_packet.low_volume_limit_set = parameters.lowVolumeLimit;

  public_data_packet.high_respiratory_rate_limit_set = parameters.highRespiratoryRateLimit; 
  public_data_packet.low_respiratory_rate_limit_set = parameters.lowRespiratoryRateLimit;
 
//#define SET_VALUES // - for testing
#ifdef SET_VALUES
  public_data_packet.mode_value = public_command_packet.mode_value; //comm.ventilationMode;
  public_data_packet.packet_type = PACKET_TYPE_DATA;

  public_data_packet.control_state = 0x1;
  public_data_packet.control_state |= CONTROLLER_BIT_START; // turn on top bit

  public_data_packet.respiratory_rate_set = public_command_packet.respiratory_rate_set;
  public_data_packet.respiratory_rate_measured = 0xE;
  
  public_data_packet.tidal_volume_set = public_command_packet.tidal_volume_set;
  public_data_packet.tidal_volume_measured = 0x1DB;

  public_data_packet.ie_ratio_set = public_command_packet.ie_ratio_set;
  public_data_packet.ie_ratio_measured = 0x2;  

  public_data_packet.peep_value_measured = 0x1AB;
  public_data_packet.peak_pressure_measured =  0x20F;
  public_data_packet.plateau_value_measurement  = 0x96;  
  
  public_data_packet.pressure_measured  =  0x3D4;
  public_data_packet.pressure_set  =  0x3D6;  
 
  public_data_packet.flow_measured = 0x3D7;

  public_data_packet.volume_in_measured = 0xC;
  public_data_packet.volume_out_measured = 0xD;
  public_data_packet.volume_rate_measured = 0xB;

  public_data_packet.high_pressure_limit_set = public_command_packet.high_pressure_limit_set; 
  public_data_packet.low_pressure_limit_set = public_command_packet.low_pressure_limit_set;   

  public_data_packet.high_volume_limit_set = public_command_packet.high_volume_limit_set; 
  public_data_packet.low_volume_limit_set = public_command_packet.low_volume_limit_set;

  public_data_packet.high_respiratory_rate_limit_set = public_command_packet.high_respiratory_rate_limit_set; 
  public_data_packet.low_respiratory_rate_limit_set = public_command_packet.low_respiratory_rate_limit_set;
#endif  

  
}
      
int linkProcessPacket(uint8_t packetType, uint8_t packetLen, uint8_t* data)
{
  static command_packet_def command_packet;
  if ((packetType==LINK_PACKET_TYPE_COMMAND_PACKET) && (packetLen==sizeof(command_packet)))
  {
    memcpy((void *)&command_packet, (void *)data, sizeof(command_packet));
    if ((serial_statistics.packetsCntReceivedOk>200) && (((serial_statistics.sequenceNoWrongCnt*100)/serial_statistics.packetsCntReceivedOk)) > MAX_DROPPED_PACKETS) 
    {  
      alarmSet(&comm.onCommunicationFailureAlarm);
      LOG_PRINT(INFO,"Uart RX Too many Failures!");
    }
    
      memcpy((void *)&public_command_packet, (void *)&command_packet, sizeof(public_command_packet));
      updateFromCommandPacket();
      lastUiPacketReceivedTime=millis();
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

static PT_THREAD(serialSendThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  static long lastPublicDataMillis=0;

  //continously process tx data
  serialHalSendData();

  //check if we need to send the next public data packet
  if (millis()>lastPublicDataMillis+LINK_PUBLIC_DATA_TIMEOUT)
  {
    lastPublicDataMillis=millis();
    updateDataPacket();

    serialHalSendPacket(LINK_PACKET_TYPE_PUBLICDATA_PACKET,sizeof(public_data_packet),(uint8_t*)&public_data_packet);

    //directly start sending if possible
    serialHalSendData();
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

  if ((lastUiPacketReceivedTime) && (millis()-lastUiPacketReceivedTime>UI_PACKET_RECEIVE_TIMEOUT) && (control.state!=CONTROL_STATE_IDLE) && !powerOff)
  {
       LOG_PRINT(INFO,"Uart RX Comm Timeout Hit!");
       alarmSet(&comm.onCommunicationFailureAlarm);
       lastUiPacketReceivedTime=0;
  }
  LOG_PRINT_EVERY(10000,INFO,"Serial Stats: TX(OK,FAIL): %li,%li RX (OK,FAIL_CRC,FAIL_OTHER,SEQ): %li,%li,%li,%li Buf(TX/RX): %li,%li ",serial_statistics.packetsCntSentOk,serial_statistics.packetsCntSentBufferOverFlow,serial_statistics.packetsCntReceivedOk,serial_statistics.packetsCntWrongCrc,serial_statistics.packetsCntHeaderSyncFailed+serial_statistics.packetsCntWrongLength+serial_statistics.packetsCntWrongVersion,serial_statistics.sequenceNoWrongCnt,serial_statistics.handleMaxTxBufferCnt,serial_statistics.handleMaxRxBufferCnt);
  PT_RESTART(pt);
  PT_END(pt);
}

int linkModuleInit(void)
{
  if (serialHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }

  alarmInit(&comm.onCommunicationFailureAlarm,&onCommunicationFailureAlarmProperties);

  PT_INIT(&serialSendThread);
  PT_INIT(&serialReadThread);

  return MODULE_OK;
}

int linkModuleRun(void)
{
  return (PT_SCHEDULE(serialThreadMain(&serialThread))) ? MODULE_OK : MODULE_FAIL;
}
