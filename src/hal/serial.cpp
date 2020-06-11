// TODO: Document what this module does
/*#define SERIAL_TX_BUFFER_SIZE 256
#define SERIAL_RX_BUFFER_SIZE 256

#include "../../arduino_lib/HardwareSerial.h"*/
#include <stdint.h>

#include <Arduino.h>

#include "../config.h"

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"
#include <util/crc16.h>

#define LOG_MODULE "serial"
#define LOG_LEVEL  LOG_LINK_MODULE
#include "../util/log.h"

#define UI_TX_BUFFERSIZE 1024
#define DEBUG_TX_BUFFERSIZE 512

typedef struct 
{
  uint16_t head;
  uint16_t tail;
  uint8_t *buffer;
  uint16_t bufferSize;
} txbuffer;

uint8_t txBufferData[UI_TX_BUFFERSIZE];
uint8_t debugTxBufferData[DEBUG_TX_BUFFERSIZE];

txbuffer uiTxBuffer;
txbuffer debugTxBuffer;

static uint8_t lastByte=0;

inline int serialTxBufferBytesAvailable(txbuffer* buffer)
{
  if (!buffer)
    return 0;

  if (buffer->head>=buffer->tail) 
    return buffer->bufferSize +buffer->tail - buffer->head -1;
  else
    return buffer->tail-buffer->head-1;
}
inline int serialTxBufferAddByte(txbuffer* buffer, uint8_t byte)
{
  if (!buffer)
    return HAL_FAIL;
  if (((buffer->head+1)%buffer->bufferSize)==buffer->tail)
    {
        return HAL_FAIL;
    }
  buffer->buffer[buffer->head]=byte;
  buffer->head++;
  buffer->head%=buffer->bufferSize;
  return HAL_OK;
}

inline int serialTxBufferGetByte(txbuffer* buffer, uint8_t* byte)
{
  if (!buffer)
    return HAL_FAIL;

  if (buffer->tail==buffer->head)
    return HAL_FAIL;
  *byte=buffer->buffer[buffer->tail];
  buffer->tail++;
  buffer->tail%=buffer->bufferSize;
  return HAL_OK;
}

SERIAL_STATISTICS serial_statistics;

int serialDebugWrite(uint8_t* buffer, uint16_t size)
{
      if (serialTxBufferBytesAvailable(&debugTxBuffer)>=size)
      {
          for (int i=0;i<size;i++)
            serialTxBufferAddByte(&debugTxBuffer,buffer[i]);
          return HAL_OK;
      }else
          return HAL_FAIL;
}

int serialHalInit(void)
{
  SERIAL_UI.begin(BAUD_RATE);  
  
  // this function does not return anything
  // not sure how we can do something like while (!serial)
  // return MODULE_FAIL;

  uiTxBuffer.buffer=&txBufferData[0];
  uiTxBuffer.bufferSize=sizeof(txBufferData);
  uiTxBuffer.head=0;
  uiTxBuffer.tail=0;

  debugTxBuffer.buffer=&debugTxBufferData[0];
  debugTxBuffer.bufferSize=sizeof(debugTxBufferData);
  debugTxBuffer.head=0;
  debugTxBuffer.tail=0;

  serial_statistics.handleMaxTxBufferCnt=10000;

  SERIAL_PORT_DEBUG.begin(BAUD_RATE_DEBUG);

  LOG_PRINT(DEBUG,"Serial Buffers: TX:%i RX:%i",SERIAL_TX_BUFFER_SIZE,SERIAL_RX_BUFFER_SIZE);

  return HAL_OK;
}

#define SERIAL_RX_MAX_BYTES_PER_CYCLE 64
#define SERIAL_TX_MAX_BYTES_PER_CYCLE 64

#define SERIAL_RX_STATE_START1  0
#define SERIAL_RX_STATE_START2  1
#define SERIAL_RX_STATE_START3  2
#define SERIAL_RX_STATE_SEQ1    3
#define SERIAL_RX_STATE_SEQ2    4
#define SERIAL_RX_STATE_PRO_VER 5
#define SERIAL_RX_STATE_PKTTYPE 6
#define SERIAL_RX_STATE_LEN     7
#define SERIAL_RX_STATE_DATA    8
#define SERIAL_RX_STATE_CRC1    9
#define SERIAL_RX_STATE_CRC2    10

#define SERIAL_STARTBYTE_1      0x26
#define SERIAL_STARTBYTE_2      0x56
#define SERIAL_STARTBYTE_3      0x7e

//handles all incoming serial data and returns a processPacket callback for every correctly processed packet
int serialHalHandleRx(int (*processPacket)(uint8_t packetType, uint8_t packetLen, uint8_t* data))
{
  uint8_t bytesAvailable;
  uint8_t inByte;
  static uint8_t rxState;
  static uint8_t byteCnt;
  static uint16_t seqNum;
  static uint16_t crcReceived;
  static uint16_t crcCalculated;
  static uint8_t rxDataBuffer[SERIAL_MAX_DATA_SIZE];
  static uint8_t packetType;
  static uint8_t msgLen;

  bytesAvailable=SERIAL_UI.available();
  
  if (bytesAvailable>serial_statistics.handleMaxRxBufferCnt)
    serial_statistics.handleMaxRxBufferCnt=bytesAvailable;

  if (bytesAvailable>SERIAL_RX_MAX_BYTES_PER_CYCLE)
  {
    bytesAvailable=SERIAL_RX_MAX_BYTES_PER_CYCLE;
  }

  while(bytesAvailable)
  {       
    inByte=SERIAL_UI.read();

    bytesAvailable--;

    if (rxState==SERIAL_RX_STATE_START1)
    {
        if (inByte==SERIAL_STARTBYTE_1)
          rxState=SERIAL_RX_STATE_START2;

    }else if (rxState==SERIAL_RX_STATE_START2)
    {
        if (inByte==SERIAL_STARTBYTE_2)
          rxState=SERIAL_RX_STATE_START3;
        else if (inByte!=SERIAL_STARTBYTE_1) //sync on S1S1S2S3 correctly
        {
          serial_statistics.packetsCntHeaderSyncFailed++;
          rxState=SERIAL_RX_STATE_START1;
        }

    }else if (rxState==SERIAL_RX_STATE_START3)
    {
        if (inByte==SERIAL_STARTBYTE_3)
          rxState=SERIAL_RX_STATE_SEQ1;
        else
        {
          serial_statistics.packetsCntHeaderSyncFailed++;
          rxState=SERIAL_RX_STATE_START1;
        }

    }else if (rxState==SERIAL_RX_STATE_SEQ1)
    {
      crcCalculated=0xFFFF;
      crcCalculated=_crc_xmodem_update(crcCalculated,inByte);
      seqNum=inByte;
      rxState=SERIAL_RX_STATE_SEQ2;
    }else if (rxState==SERIAL_RX_STATE_SEQ2)
    {
      crcCalculated=_crc_xmodem_update(crcCalculated,inByte);
      seqNum|=inByte<<8;
      rxState=SERIAL_RX_STATE_PRO_VER;
    }else if (rxState==SERIAL_RX_STATE_PRO_VER)
    {
      crcCalculated=_crc_xmodem_update(crcCalculated,inByte);
      if (inByte!=SERIAL_PROTOCOL_VERSION)
      {
        rxState=SERIAL_RX_STATE_START1;
        serial_statistics.packetsCntWrongVersion++;
      }else
        rxState=SERIAL_RX_STATE_PKTTYPE;
      
    }else if (rxState==SERIAL_RX_STATE_PKTTYPE)
    {
      crcCalculated=_crc_xmodem_update(crcCalculated,inByte);
      packetType=inByte;
      rxState=SERIAL_RX_STATE_LEN;

    }else if (rxState==SERIAL_RX_STATE_LEN)
    {
      crcCalculated=_crc_xmodem_update(crcCalculated,inByte);
      msgLen=inByte;
      if (msgLen>SERIAL_MAX_DATA_SIZE)
      {
        rxState=SERIAL_RX_STATE_START1;
        serial_statistics.packetsCntWrongLength++;
      }else
      {
        rxState=SERIAL_RX_STATE_DATA;
        byteCnt=0;
      }
    }else if (rxState==SERIAL_RX_STATE_DATA)
    {
      crcCalculated=_crc_xmodem_update(crcCalculated,inByte);
      rxDataBuffer[byteCnt]=inByte;
      byteCnt++;
      if (byteCnt==msgLen)
      {
        rxState=SERIAL_RX_STATE_CRC1;
      }
    }else if (rxState==SERIAL_RX_STATE_CRC1)
    {
      crcReceived=inByte;
      rxState=SERIAL_RX_STATE_CRC2;
    }else if (rxState==SERIAL_RX_STATE_CRC2)
    {
      crcReceived|=inByte<<8;
      rxState=SERIAL_RX_STATE_START1;

      if (crcReceived==crcCalculated)
      {
          static uint16_t lastSeqNum=0;
          if ((seqNum!=lastSeqNum+1) && (serial_statistics.packetsCntReceivedOk)) //if serial_statistics.packetsCntReceivedOk==0 it's the first packet
          {
              serial_statistics.sequenceNoWrongCnt++;
          }

          serial_statistics.packetsCntReceivedOk++;
          if (processPacket)
            processPacket(packetType,msgLen,&rxDataBuffer[0]);
          lastSeqNum=seqNum;
      }else
      {
        serial_statistics.packetsCntWrongCrc++;
      }

    }else
    {
      rxState=SERIAL_RX_STATE_START1;
    }


  };
  return HAL_IN_PROGRESS;
}

int serialHalSendPacket(uint8_t packetType, uint8_t packetLength, uint8_t* data)
{
    uint8_t cnt;
    uint16_t crc=0xFFFF;
    static uint16_t sequenceNumber=0;

    //make sure that other packets are only accepted if buffer is <75%
    if (packetType!=LINK_PACKET_TYPE_PUBLICDATA_PACKET)
    {
      if (packetLength+SERIAL_HEADER_LENGTH>((2*serialTxBufferBytesAvailable(&uiTxBuffer))/4))
      {
        serial_statistics.packetsCntSentBufferOverFlow++;
        return HAL_FAIL;
      }
    }else
    {
      if (packetLength+SERIAL_HEADER_LENGTH>serialTxBufferBytesAvailable(&uiTxBuffer))
      {
        serial_statistics.packetsCntSentBufferOverFlow++;
        return HAL_FAIL;
      }
    }

    if (packetLength>SERIAL_MAX_DATA_SIZE)
    {
      serial_statistics.packetsCntSentMaxDatasizeError++;
      return HAL_FAIL;
    }      

    serialTxBufferAddByte(&uiTxBuffer,SERIAL_STARTBYTE_1);
    serialTxBufferAddByte(&uiTxBuffer,SERIAL_STARTBYTE_2);
    serialTxBufferAddByte(&uiTxBuffer,SERIAL_STARTBYTE_3);
    serialTxBufferAddByte(&uiTxBuffer,sequenceNumber&0xff);
    crc=_crc_xmodem_update(crc,sequenceNumber&0xff);
    serialTxBufferAddByte(&uiTxBuffer,sequenceNumber>>8);
    crc=_crc_xmodem_update(crc,sequenceNumber>>8);
    serialTxBufferAddByte(&uiTxBuffer,SERIAL_PROTOCOL_VERSION);
    crc=_crc_xmodem_update(crc,SERIAL_PROTOCOL_VERSION);
    serialTxBufferAddByte(&uiTxBuffer,packetType);
    crc=_crc_xmodem_update(crc,packetType);
    serialTxBufferAddByte(&uiTxBuffer,packetLength);
    crc=_crc_xmodem_update(crc,packetLength);
    for (cnt=0;cnt<packetLength;cnt++)
    {
      serialTxBufferAddByte(&uiTxBuffer,data[cnt]);
      crc=_crc_xmodem_update(crc,data[cnt]);
    }
    serialTxBufferAddByte(&uiTxBuffer,crc&0xff);
    serialTxBufferAddByte(&uiTxBuffer,crc>>8);

    serial_statistics.packetsCntSentOk++;
    sequenceNumber++;

    return HAL_OK;
}

int serialHalSendData()
{
    uint16_t availableForWrite;     
    uint8_t ret;
    uint8_t byte;

    if (serialTxBufferBytesAvailable(&uiTxBuffer)<serial_statistics.handleMaxTxBufferCnt)
      serial_statistics.handleMaxTxBufferCnt=serialTxBufferBytesAvailable(&uiTxBuffer);

    availableForWrite = SERIAL_UI.availableForWrite();
    
    if (availableForWrite>SERIAL_TX_MAX_BYTES_PER_CYCLE)
      availableForWrite=SERIAL_TX_MAX_BYTES_PER_CYCLE;

    while (availableForWrite)
    {
      if (serialTxBufferGetByte(&uiTxBuffer,&byte)==HAL_OK)
      {
        SERIAL_UI.write(byte);
        availableForWrite--;
      }else
        availableForWrite=0;
    }

    availableForWrite = SERIAL_PORT_DEBUG.availableForWrite();
    
    if (availableForWrite>SERIAL_TX_MAX_BYTES_PER_CYCLE)
      availableForWrite=SERIAL_TX_MAX_BYTES_PER_CYCLE;

    while (availableForWrite)
    {
      if (serialTxBufferGetByte(&debugTxBuffer,&byte)==HAL_OK)
      {
        SERIAL_PORT_DEBUG.write(byte);
        availableForWrite--;
      }else
        availableForWrite=0;
    }

    return HAL_OK;
}
