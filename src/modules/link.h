// TODO: Document what this module does
#ifndef __LINK_MODULE_H__
#define __LINK_MODULE_H__

#include <stdint.h>

#include "../util/queue.h"

#define MAX_PACKET_DATA_SIZE  255
#define MAX_LOG_DATA_SIZE     255
#define LOG_BUFFER_SIZE       512

#define PACKET_START_SEQUENCE 0x26567EUL

#define PACKET_VERSION        4U

#define PACKET_TYPE_DATA      1U
#define PACKET_TYPE_COMMAND   2U
#define PACKET_TYPE_FIRMWARE  3U
#define PACKET_TYPE_LOG       4U

#define LOG_MESSAGE_ID        1U
#define LOG_CONTROL_DATA_ID   2U

struct packet {
  uint8_t start[3];
  union {
    struct {
      uint16_t sequenceNumber;
      uint8_t  version;
      uint8_t  type;
      uint8_t  length;
    };
    uint8_t raw[5];
  } header;
  uint8_t data[MAX_PACKET_DATA_SIZE + sizeof(uint16_t)]; // Max packet data size plus bytes for the CRC
} __attribute__((packed));

struct dataPacket {
  uint8_t mode;
  uint8_t controllerState;
  uint8_t batteryStatus;
  uint8_t reserved;
  uint16_t respiratoryRateSet;
  uint16_t respiratoryRateMeasured;
  int16_t  volumeSet;
  int16_t  volumeMeasured;
  uint16_t ieRatioSet;
  uint16_t ieRatioMeasured;
  int16_t  peepMeasured;
  int16_t  peakPressureMeasured;
  int16_t  plateauPressureMeasured;
  int16_t  pressureSet;
  int16_t  pressureMeasured;
  int16_t  flowMeasured;
  int16_t  volumeInMeasured;
  int16_t  volumeOutMeasured;
  int16_t  minuteVolumeMeasured;
  int16_t  highPressureLimit;
  int16_t  lowPressureLimit;
  int16_t  highVolumeLimit;
  int16_t  lowVolumeLimit;
  uint16_t highRespiratoryRateLimit;
  uint16_t lowRespiratoryRateLimit;
  uint32_t alarmBits;
} __attribute__((packed));

struct commandPacket {
  uint8_t  mode;
  uint8_t  cmdBits;
  uint16_t respiratoryRateSet;
  int16_t  volumeSet;
  uint16_t ieRatioSet;
  int16_t  pressureSet;
  int16_t  highPressureLimit;
  int16_t  lowPressureLimit;
  int16_t  highVolumeLimit;
  int16_t  lowVolumeLimit;
  uint16_t highRespiratoryRateLimit;
  uint16_t lowRespiratoryRateLimit;
  uint32_t alarmBits;
} __attribute__((packed));

struct logPacket {
  uint16_t id;
  uint8_t data[];
} __attribute__((packed));

struct logMessage {
  union {
    struct {
      uint8_t  size;
      uint16_t id;
    };
    uint8_t raw[3];
  } header;
  uint8_t data[];
} __attribute__((packed));

struct link {
  // Variables
  uint8_t  startVentilation;
  uint8_t  ventilationMode;
  int16_t  volumeRequested;
  int16_t  pressureRequested;
  uint16_t respirationRateRequested;
  uint16_t ieRatioRequested;
  int16_t  highPressureLimit;
  int16_t  lowPressureLimit;
  int16_t  highVolumeLimit;
  int16_t  lowVolumeLimit;
  uint16_t highRespiratoryRateLimit;
  uint16_t lowRespiratoryRateLimit;
  
  struct queue logQueue;
  uint32_t loggingDroppedMessages;
  uint32_t loggingSentMessages;
  uint32_t sentPackets;
  uint32_t recievedPackets;
  uint32_t falseStarts;
  uint32_t badCRC;
  uint32_t badVersion;
  uint32_t badSequenceNumber;
  uint32_t droppedCommandPackets;
  
  // Alarms
  int8_t   droppedPacketAlarm;
  int8_t   crcErrorAlarm;
  int8_t   unsupportedPacketVersionAlarm;
};

// Public Variables
extern struct link comm;

// TODO: Doc
int linkModuleInit(void);

// TODO: Doc
int linkModuleRun(void);

#endif /* __LINK_MODULE_H__ */
