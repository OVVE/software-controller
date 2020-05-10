// TODO: Document what this module does

#ifndef __SERIAL_HAL_H__
#define __SERIAL_HAL_H__
#include <Arduino.h>
#include "../hal/hal.h"

#define BAUD_RATE 125000 //due to 16 Mhz crystal without fractional baudrate divider any of these baudrate has 0% bitrate error: 1000000 500000 250000 125000 62500
#define WATCHDOG_MS 200  // maximum time to receive all bytes from command packet. if time expires trash data and send next packet.
#define SEND_INTERVAL_MS 100 // minimum time between sending packets. The time could take longer if response is late.
#define SEND_MAX_TIME_MS 30  // maximum time to allow for sending data packet. This is used with sending data in parts if send buffer full.
#define USE_AVAILABLE_WRITE  // #ifdef in serial.cpp for checking the available bytes on write queue before sending to keep from blocking

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
  uint32_t commandPacketSequenceWrongCnt;
} SERIAL_STATISTICS;

extern SERIAL_STATISTICS serial_statistics;

int serialHalInit(void);
int serialHalHandleRx(int (*processPacket)(uint8_t packetType, uint8_t packetLen, uint16_t sequenceNumber, uint8_t* data));
int serialHalSendPacket(uint8_t packetType, uint8_t packetLength, uint16_t sequenceNumber, uint8_t* data);
int serialHalSendData(void);


#endif /* __SERIAL_HAL_H__ */