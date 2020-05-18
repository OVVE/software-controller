// TODO: Document what this module does
//
// Link Module
//
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "../config.h"

#include "../pt/pt.h"

#include "../hal/timer.h"
#include "../hal/serial.h"

#include "../modules/link.h"
#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/sensors.h"
#include "../modules/parameters.h"

#include "../util/crc.h"
#include "../util/alarm.h"
#include "../util/queue.h"
#include "../util/utils.h"

#define LOG_MODULE "link"
#define LOG_LEVEL  LOG_LINK_MODULE
#include "../util/log.h"

#define UNPACK_START_VENTILATION(cmd) \
  (((cmd) >> 0) & 0x1);

#define PACK_CONTROLLER_STATE(start, state) \
  ((((start) << 7) & 0x80) | (((state) << 0) & 0x0F))
#define PACK_BATTERY_STATUS(charge, level) \
  ((((charge) << 7) & 0x80) | (((level) << 0) & 0x7F))

// Public Variables
struct link comm;

// Private Variables
static struct pt serialThread;
static struct pt serialTxThread;
static struct pt serialRxThread;

static struct timer commandPacketTimer;
static uint16_t expectedDataSequenceNumber;
static bool waitOnCommandPacket;
static uint32_t previousLocalAlarms;
static uint32_t previousRemoteAlarms;

static uint8_t logBuffer[LOG_BUFFER_SIZE];

static struct alarm** alarmList[] = {
  (struct alarm*[]) { NULL }, // Bit 0  - Power Los
  (struct alarm*[]) { NULL }, // Bit 1  - Low Battery
  (struct alarm*[]) { NULL }, // Bit 2  - Bad Pressure Sensor
  (struct alarm*[]) { NULL }, // Bit 3  - Bad Flow Sensors
  (struct alarm*[]) { NULL }, // Bit 4  - Communication Failure (ECU)
  (struct alarm*[]) { NULL }, // Bit 5  - Hardware Failure (UI)
  (struct alarm*[]) { NULL }, // Bit 6  - E-stop
  (struct alarm*[]) { NULL }, // Bit 7  -
  (struct alarm*[]) { NULL }, // Bit 8  - High Pressure
  (struct alarm*[]) { NULL }, // Bit 9  - Low Pressure
  (struct alarm*[]) { NULL }, // Bit 10 - High Volume
  (struct alarm*[]) { NULL }, // Bit 11 - Low Volume
  (struct alarm*[]) { NULL }, // Bit 12 - High RR
  (struct alarm*[]) { NULL }, // Bit 13 - Low RR
  (struct alarm*[]) { NULL }, // Bit 14 -
  (struct alarm*[]) { NULL }, // Bit 15 -
  (struct alarm*[]) { NULL }, // Bit 16 - Communication Failure (UI)
  (struct alarm*[]) { NULL }, // Bit 17 - Hardware Failure (UI)
  (struct alarm*[]) { NULL }, // Bit 18 -
  (struct alarm*[]) { NULL }, // Bit 19 -
  (struct alarm*[]) { NULL }, // Bit 20 -
  (struct alarm*[]) { NULL }, // Bit 21 -
  (struct alarm*[]) { NULL }, // Bit 22 -
  (struct alarm*[]) { NULL }, // Bit 23 -
  (struct alarm*[]) { NULL }, // Bit 24 - Setpoint Mismatch
  (struct alarm*[]) { NULL }, // Bit 25 -
  (struct alarm*[]) { NULL }, // Bit 26 -
  (struct alarm*[]) { NULL }, // Bit 27 -
  (struct alarm*[]) { NULL }, // Bit 28 -
  (struct alarm*[]) { NULL }, // Bit 29 -
  (struct alarm*[]) { NULL }, // Bit 30 -
  (struct alarm*[]) { NULL }, // Bit 31 -
};

void updateFromCommandPacket(struct commandPacket* command)
{
  comm.startVentilation = UNPACK_START_VENTILATION(command->cmdBits);
  comm.ventilationMode = command->mode;
  comm.volumeRequested = command->volumeSet;
  comm.respirationRateRequested = command->respiratoryRateSet;
  comm.ieRatioRequested = command->ieRatioSet;
  comm.pressureRequested = command->pressureSet;
  comm.highPressureLimit = command->highPressureLimit;
  comm.lowPressureLimit = command->lowPressureLimit;
  comm.highVolumeLimit = command->highVolumeLimit;
  comm.lowVolumeLimit = command->lowVolumeLimit;
  comm.highRespiratoryRateLimit = command->highRespiratoryRateLimit;
  comm.lowRespiratoryRateLimit = command->lowRespiratoryRateLimit;
  
  // Loop through all the alarm bits, if one is set but was previously unset,
  // suppress all internal alarms associated with that bit
  for (int i = 0; i < sizeof(alarmList) / sizeof(alarmList[0]); i++) {
    bool localSet = (previousLocalAlarms & (1 << i)) != 0;
    bool currentRemoteSet = ((command->alarmBits) & (1 << i)) != 0;
    bool previousRemoteSet = (previousRemoteAlarms & (1 << i)) != 0;
    if (localSet && currentRemoteSet && !previousRemoteSet) {
      struct alarm** alarmsPtr = alarmList[i];
      while (*alarmsPtr) {
        alarmSuppress(*alarmsPtr);
        alarmsPtr++;
      }
    }
  }
  
  previousRemoteAlarms = command->alarmBits;
}

size_t updateDataPacket(struct dataPacket* packet)
{
  packet->mode = parameters.ventilationMode;
  packet->controllerState = PACK_CONTROLLER_STATE(parameters.startVentilation,
                                                  control.state);
  packet->batteryStatus = PACK_BATTERY_STATUS(sensors.batteryCharging,
                                              sensors.batteryLevel);

  packet->respiratoryRateSet = parameters.respirationRateRequested;
  packet->respiratoryRateMeasured = control.respirationRateMeasured;
  packet->volumeSet = parameters.volumeRequested;
  packet->volumeMeasured = sensors.currentVolume;
  packet->ieRatioSet = parameters.ieRatioRequested;
  packet->ieRatioMeasured = control.ieRatioMeasured;
  packet->peepMeasured = sensors.peepPressure;
  packet->peakPressureMeasured = sensors.peakPressure;
  packet->plateauPressureMeasured = sensors.plateauPressure;
  packet->pressureSet = 0;
  packet->pressureMeasured = sensors.currentPressure;
  packet->flowMeasured = sensors.currentFlow;
  packet->volumeInMeasured = sensors.volumeIn;
  packet->volumeOutMeasured = sensors.volumeOut;
  packet->minuteVolumeMeasured = sensors.volumePerMinute;
/*
  packet->highPressureLimit = parameters.highPressureLimit;
  packet->lowPressureLimit = parameters.lowPressureLimit;
  packet->highVolumeLimit = parameters.highVolumeLimit;
  packet->lowVolumeLimit = parameters.lowVolumeLimit;
  packet->highRespiratoryRateLimit = parameters.highRespiratoryRateLimit;
  packet->lowRespiratoryRateLimit = parameters.lowRespiratoryRateLimit;
  */
  packet->alarmBits = 0;
  
  for (int i = 0; i < sizeof(alarmList) / sizeof(alarmList[0]); i++) {
    struct alarm** alarmsPtr = alarmList[i];
    while (*alarmsPtr) {
      if (alarmGet(*alarmsPtr)) {
        packet->alarmBits |= 1 << i;
      }
      alarmsPtr++;
    }
  }
  
  previousLocalAlarms = packet->alarmBits;

  return sizeof(struct dataPacket);
}

size_t updateLogPacket(struct logPacket* packet)
{
  struct logMessage message;
  uint8_t packetSize = 0;
  uint8_t emptyByte;

  // Pop off log message header to to determine how much data needs to be taken
  // out of the log queue
  for (int i = 0; i < sizeof(struct logMessage); i++) {
    queuePop(&(comm.logQueue), &(message.header.raw[i]));
  }
  
  // Load log id into the packet
  packet->id = message.header.id;
  packetSize += sizeof(packet->id);
  
  // Pop out the entire message and put it in the packet, if the message is too
  // long, truncate it and pop the rest to 
  for (int i = 0; i < message.header.size; i++) {
    if (packetSize < MAX_PACKET_DATA_SIZE) {
      queuePop(&(comm.logQueue), &(packet->data[i]));
      packetSize++;
    } else {
      queuePop(&(comm.logQueue), &emptyByte);
    }
  }
  
  return packetSize;
}

size_t finalizePacket(struct packet* packet, uint16_t packetSequenceNumber, uint8_t packetType, size_t dataSize)
{
  // Fill in packet header
  packet->start[0] = (PACKET_START_SEQUENCE >> 0) & 0xff;
  packet->start[1] = (PACKET_START_SEQUENCE >> 8) & 0xff;
  packet->start[2] = (PACKET_START_SEQUENCE >> 16) & 0xff;
  packet->header.sequenceNumber = packetSequenceNumber;
  packet->header.version = PACKET_VERSION;
  packet->header.type = packetType;
  packet->header.length = dataSize;
  
  // Calculate CRC over packet and place at the end of the packet
  uint16_t crc = crc16((uint8_t*) &(packet->header),
                       sizeof(packet->header) + dataSize,
                       CRC16_INITIAL_VALUE);
  memcpy(&(packet->data[dataSize]), &crc, sizeof(uint16_t));
  
  return sizeof(packet->start) + sizeof(packet->header) + dataSize + sizeof(crc);
}

bool processByte(struct packet* packet, uint8_t byteIn)
{
  static uint16_t processedBytes = 0;
  static uint16_t crc = CRC16_INITIAL_VALUE;
  
  if (processedBytes < offsetof(struct packet, header)) {
    // Look for Start Sequence
    if (byteIn != (PACKET_START_SEQUENCE >> processedBytes) & 0xff) {
      // Start sequence error, go back to either having seen none of the start
      // sequence or just the first byte
      comm.falseStarts++;
      processedBytes = (byteIn == (PACKET_START_SEQUENCE & 0xff)) ? 1 : 0;
    } else {
      processedBytes++;
    }
  } else if (processedBytes < offsetof(struct packet, data)) {
    // Process packet header
    packet->header.raw[processedBytes - offsetof(struct packet, header)] = byteIn;
    crc = crc16(&byteIn, sizeof(uint8_t), crc);
    // Check the version
    if (processedBytes == offsetof(struct packet, header.version)) {
      if (packet->header.version != PACKET_VERSION) {
        // Version mismatch
        processedBytes = 0;
        crc = CRC16_INITIAL_VALUE;
        comm.badVersion++;
        comm.recievedPackets++;
      } else {
        processedBytes++;
      }
    } else {
      processedBytes++;
    }
  } else if (processedBytes < offsetof(struct packet, data) + packet->header.length) {
    // Process packet data
    packet->data[processedBytes - offsetof(struct packet, data)] = byteIn;
    crc = crc16(&byteIn, sizeof(uint8_t), crc);
    processedBytes++;
  } else {
    // Process CRC16
    uint16_t crcOffset = processedBytes - (offsetof(struct packet, data) + packet->header.length);
    crc ^= byteIn << crcOffset;
    bool crcGood = (crc == 0);
    processedBytes++;
    if (crcOffset) {
      processedBytes = 0;
      crc = CRC16_INITIAL_VALUE;
      comm.recievedPackets++;
      if (crcGood) {
        // Good packet delivered, pass it to the high level to handle
        return true;
      } else {
        // CRC error
        comm.badCRC++;
      }
    }
  }
  
  return false;
}

static PT_THREAD(serialRxThreadMain(struct pt* pt))
{  
  static int rc = HAL_OK;
  static uint8_t readByte = 0;
  static struct packet packet;

  PT_BEGIN(pt);

  while (1) {
    PT_WAIT_UNTIL(pt,
                  (rc = serialHalRead(SERIAL_PORT_UI, &readByte,
                                      sizeof(readByte), 5 MSEC)) != HAL_IN_PROGRESS);
    
    if (waitOnCommandPacket &&
        (timerHalRun(&commandPacketTimer) == HAL_TIMEOUT)) {
      // Command packet lost
      comm.droppedCommandPackets++;
    }
    
    if (rc == HAL_TIMEOUT) {
      continue;
    }

    if (processByte(&packet, readByte)) {
      if (packet.header.type == PACKET_TYPE_COMMAND) {
        if (packet.header.sequenceNumber != expectedDataSequenceNumber) {
          // Bad sequence number, but keep going as otherwise the packet is good
          comm.badSequenceNumber++;
        }
        updateFromCommandPacket((struct commandPacket*) packet.data);
      } else {
        // Unknown packet type
      }
    }
    
    PT_YIELD(pt);
  }
  
  PT_END(pt);
}

static PT_THREAD(serialTxThreadMain(struct pt* pt))
{
  static int rc = HAL_OK;
  static struct timer dataPacketTimer;
  static struct packet packet;
  static uint16_t packetSequenceNumber = 0;
  static size_t packetSize;

  PT_BEGIN(pt);
  
  // Kickoff periodic timer for 
  timerHalBegin(&dataPacketTimer, 100 MSEC, true);

  while (1) {
    PT_WAIT_UNTIL(pt,
                  ((rc = timerHalRun(&dataPacketTimer)) != HAL_IN_PROGRESS ||
                   (queueSize(&comm.logQueue) > sizeof(struct logMessage))));
    
    // Either send a data packet if the timer ran out, or a log packet if one is
    // ready
    size_t dataSize;
    uint8_t packetType;
    if (rc == HAL_TIMEOUT) {
      dataSize = updateDataPacket((struct dataPacket*) packet.data);
      packetType = PACKET_TYPE_DATA;
    } else {
      dataSize = updateLogPacket((struct logPacket*) packet.data);
      packetType = PACKET_TYPE_LOG;
    }
    packetSize = finalizePacket(&packet, packetSequenceNumber, packetType, dataSize);
    
    // Actually send the packet
    PT_WAIT_UNTIL(pt, serialHalWrite(SERIAL_PORT_UI, &packet,
                                     packetSize, 30 MSEC) != HAL_IN_PROGRESS);

    // For data packets, set the expected return type and timeout
    if (packet.header.type == PACKET_TYPE_DATA) {
      expectedDataSequenceNumber = packetSequenceNumber;
      timerHalBegin(&commandPacketTimer, 50 MSEC, false);
      waitOnCommandPacket = true;
      
      // Since data packets are predictable, print out some info on the link
      // module here
      LOG_PRINT_EVERY(1000, DEBUG, "msgs: %lu sent, %lu lost",
                      comm.loggingSentMessages, comm.loggingDroppedMessages);
      LOG_PRINT_EVERY(1000, DEBUG, "pkts: %lu sent, %lu recv",
                      comm.sentPackets, comm.recievedPackets);
      LOG_PRINT_EVERY(1000, DEBUG, "pkt err: %lu fs, %lu crc, %lu sq#, %lu ver, %lu drp",
                      comm.recievedPackets, comm.badCRC,
                      comm.badSequenceNumber, comm.badVersion,
                      comm.droppedCommandPackets);
    }
    packetSequenceNumber++;
    comm.sentPackets++;
  }
  
  PT_END(pt);
}

PT_THREAD(serialThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  if (!PT_SCHEDULE(serialTxThreadMain(&serialTxThread))) {
    PT_EXIT(pt);
  }
  
  if (!PT_SCHEDULE(serialRxThreadMain(&serialRxThread))) {
    PT_EXIT(pt);
  }

  PT_RESTART(pt);
  PT_END(pt);
}

int linkModuleInit(void)
{  
  queueInit(&(comm.logQueue), sizeof(logBuffer), sizeof(uint8_t), logBuffer);

  PT_INIT(&serialThread);
  PT_INIT(&serialTxThread);
  PT_INIT(&serialRxThread);

  return MODULE_OK;
}

int linkModuleRun(void)
{
  return (PT_SCHEDULE(serialThreadMain(&serialThread))) ? MODULE_OK : MODULE_FAIL;
}
