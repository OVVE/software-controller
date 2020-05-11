// TODO: Document what this module does

#ifndef __SERIAL_HAL_H__
#define __SERIAL_HAL_H__
#include <Arduino.h>
#include "../hal/hal.h"

//MSG IDs
#define LINK_PACKET_TYPE_PUBLICDATA_PACKET 0x01
#define LINK_PACKET_TYPE_COMMAND_PACKET 0x02
#define LOG_PACKET_TEXT                 0x80
#define LOG_PACKET_CONTROL              0x81
#define LOG_PACKET_SENSORS              0x82
#define LOG_PACKET_SERIAL_STATS         0x83


#define BAUD_RATE 125000 //due to 16 Mhz crystal without fractional baudrate divider any of these baudrate has 0% bitrate error: 1000000 500000 250000 125000 62500

#define SERIAL_UI Serial1
//#define SERIAL_DEBUG Serial

typedef struct{
  uint32_t packetsCntReceivedOk;
  uint32_t packetsCntWrongLength;
  uint32_t packetsCntHeaderSyncFailed;
  uint32_t packetsCntWrongCrc;
  uint32_t packetsCntWrongVersion;
  uint32_t packetsCntSentOk;
  uint32_t packetsCntSentBufferOverFlow;
  uint32_t packetsCntSentMaxDatasizeError;
  uint32_t sequenceNoWrongCnt;
} SERIAL_STATISTICS;

extern SERIAL_STATISTICS serial_statistics;

int serialHalInit(void);
int serialHalHandleRx(int (*processPacket)(uint8_t packetType, uint8_t packetLen, uint8_t* data));
int serialHalSendPacket(uint8_t packetType, uint8_t packetLength, uint8_t* data);
int serialHalSendData(void);


#endif /* __SERIAL_HAL_H__ */