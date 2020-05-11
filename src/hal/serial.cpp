// TODO: Document what this module does

#include <stdint.h>

#include "../config.h"

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"
#include <util/crc16.h>


#ifdef DEBUG_SERIAL
#define DEBUG_MODULE "serial"
#include "../util/debug.h"
#endif

#define SERIAL_TX_BUFFERSIZE 2048

uint16_t serialTxBufferHead=0;
uint16_t serialTxBufferTail=0;

uint8_t serialTxBuffer[SERIAL_TX_BUFFERSIZE];

inline int serialTxBufferBytesAvailable(void)
{
  if (serialTxBufferHead>=serialTxBufferTail) 
    return SERIAL_TX_BUFFERSIZE +serialTxBufferTail - serialTxBufferHead -1;
  else
    return serialTxBufferTail-serialTxBufferHead-1;
}
inline int serialTxBufferAddByte(uint8_t byte)
{
  if (((serialTxBufferHead+1)%SERIAL_TX_BUFFERSIZE)==serialTxBufferTail)
    {
        return HAL_FAIL;
    }
  serialTxBuffer[serialTxBufferHead]=byte;
  serialTxBufferHead++;
  serialTxBufferHead%=SERIAL_TX_BUFFERSIZE;
}

inline int serialTxBufferGetByte(uint8_t* byte)
{
  if (serialTxBufferTail==serialTxBufferHead)
    return HAL_FAIL;
  *byte=serialTxBuffer[serialTxBufferTail];
  serialTxBufferTail++;
  serialTxBufferTail%=SERIAL_TX_BUFFERSIZE;
  return HAL_OK;
}

SERIAL_STATISTICS serial_statistics;

int serialHalInit(void)
{
  SERIAL_UI.begin(BAUD_RATE);  // this function does not return anything
  // not sure how we can do something like while (!serial)
  // return MODULE_FAIL;
  
  DEBUG_BEGIN;

  DEBUG_PRINT("Serial Buffers: TX:%i RX:%i",SERIAL_TX_BUFFER_SIZE,SERIAL_RX_BUFFER_SIZE);

  return HAL_OK;
}

#define SERIAL_MAX_DATA_SIZE 128
#define SERIAL_PROTOCOL_VERSION 4
#define SERIAL_HEADER_LENGTH (3+2+1+1+2) //3 start bytes, 2 seq numbers, 1 protocol version, 1 length, 2 CRC

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

  if (bytesAvailable>SERIAL_RX_MAX_BYTES_PER_CYCLE)
    bytesAvailable=SERIAL_RX_MAX_BYTES_PER_CYCLE;

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
    if (packetLength+SERIAL_HEADER_LENGTH>serialTxBufferBytesAvailable())
    {
      serial_statistics.packetsCntSentBufferOverFlow++;
      return HAL_FAIL;
    }

    if (packetLength>SERIAL_MAX_DATA_SIZE)
    {
      serial_statistics.packetsCntSentMaxDatasizeError++;
      return HAL_FAIL;
    }      

    serialTxBufferAddByte(SERIAL_STARTBYTE_1);
    serialTxBufferAddByte(SERIAL_STARTBYTE_2);
    serialTxBufferAddByte(SERIAL_STARTBYTE_3);
    serialTxBufferAddByte(sequenceNumber&0xff);
    crc=_crc_xmodem_update(crc,sequenceNumber&0xff);
    serialTxBufferAddByte(sequenceNumber>>8);
    crc=_crc_xmodem_update(crc,sequenceNumber>>8);
    serialTxBufferAddByte(SERIAL_PROTOCOL_VERSION);
    crc=_crc_xmodem_update(crc,SERIAL_PROTOCOL_VERSION);
    serialTxBufferAddByte(packetType);
    crc=_crc_xmodem_update(crc,packetType);
    serialTxBufferAddByte(packetLength);
    crc=_crc_xmodem_update(crc,packetLength);
    for (cnt=0;cnt<packetLength;cnt++)
    {
      serialTxBufferAddByte(data[cnt]);
      crc=_crc_xmodem_update(crc,data[cnt]);
    }
    serialTxBufferAddByte(crc&0xff);
    serialTxBufferAddByte(crc>>8);

    serial_statistics.packetsCntSentOk++;
    sequenceNumber++;

    return HAL_OK;
}

int serialHalSendData()
{
    uint16_t availableForWrite;     
    uint8_t ret;
    uint8_t byte;
    availableForWrite = SERIAL_UI.availableForWrite();

    if (availableForWrite>SERIAL_TX_MAX_BYTES_PER_CYCLE)
      availableForWrite=SERIAL_TX_MAX_BYTES_PER_CYCLE;

    while (availableForWrite)
    {
      if (serialTxBufferGetByte(&byte)==HAL_OK)
      {
        SERIAL_UI.write(byte);
        availableForWrite--;
      }else
        availableForWrite=0;
    }

    return HAL_OK;
}
