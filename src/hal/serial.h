/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/
// TODO: Document what this module does

#ifndef __SERIAL_HAL_H__
#define __SERIAL_HAL_H__

#include "../hal/hal.h"

//MSG IDs
#define LINK_PACKET_TYPE_PUBLICDATA_PACKET    0x01
#define LINK_PACKET_TYPE_COMMAND_PACKET       0x02
#define LINK_PACKET_TYPE_TEXT                 0x80
#define LINK_PACKET_TYPE_CONTROL              0x81
#define LINK_PACKET_TYPE_SENSORS              0x82
#define LINK_PACKET_TYPE_SERIAL_STATS         0x83

#define SERIAL_MAX_DATA_SIZE 128
#define SERIAL_PROTOCOL_VERSION 4
#define SERIAL_HEADER_LENGTH (3+2+1+1+1+2) //3 start bytes, 2 seq numbers, 1 protocol version, 1 id, 1 length, 2 CRC


#define BAUD_RATE 500000 //due to 16 Mhz crystal without fractional baudrate divider any of these baudrate has 0% bitrate error: 1000000 500000 250000 125000 62500
#define BAUD_RATE_DEBUG 500000 //due to 16 Mhz crystal without fractional baudrate divider any of these baudrate has 0% bitrate error: 1000000 500000 250000 125000 62500

#define SERIAL_UI Serial1
#define SERIAL_PORT_DEBUG Serial

typedef struct{
  uint32_t packetsCntReceivedOk;
  uint32_t packetsCntWrongLength;
  uint32_t packetsCntHeaderSyncFailed;
  uint32_t packetsCntWrongCrc;
  uint32_t packetsCntWrongVersion;
  uint32_t packetsCntSentOk;
  uint32_t packetsCntSentBufferOverFlow;
  uint32_t packetsCntSentMaxDatasizeError;
  uint32_t textPacketsSentCnt;
  uint32_t textPacketsDroppedCnt;
  uint32_t sequenceNoWrongCnt;
  uint32_t handleMaxRxBufferCnt;
  uint32_t handleMaxTxBufferCnt;

} SERIAL_STATISTICS;

extern SERIAL_STATISTICS serial_statistics;

int serialHalInit(void);
int serialHalHandleRx(int (*processPacket)(uint8_t packetType, uint8_t packetLen, uint8_t* data));
int serialHalSendPacket(uint8_t packetType, uint8_t packetLength, uint8_t* data);
int serialHalSendData(void);
int serialDebugWrite(uint8_t* buffer, uint16_t size);



#endif /* __SERIAL_HAL_H__ */