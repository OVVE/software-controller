#include <Arduino.h>

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"

#define PACKET_VERSION 1

uint32_t last_send_ms = 0;  // this is used for send interval
uint32_t current_send_ms = 0;
uint32_t send_interval_ms = 1000;
uint16_t bytesSent;
uint16_t inByte;

#define ALARM_ECU_POWER_LOSS              0x01
#define ALARM_ECU_LOW_BATTERY             0x02 
#define ALARM_ECU_LOSS_BREATH_INTEGRITY   0x04
#define ALARM_ECU_HIGH_AIRWAY_PRESSURE    0x08
#define ALARM_ECU_LOW_AIRWAY_PRESSURE     0x10
#define ALARM_ECU_LOW_TIDAL_VOL_DELIVERED 0x20
#define ALARM_ECU_APNEA                   0x40
// bits 7 - 15 --
#define ALARM_CRC_ERROR                   0x00010000
#define ALARM_DROPPED_PACKET              0x00020000
#define ALARM_SERIAL_COMM                 0x00040000
#define ALARM_PACKET_VERSION              0x00080000
// bits 20 - 23 --
#define ALARM_UI_MODE_MISMATCH            0x01000000
#define ALARM_UI_RESP_RATE_MISMATCH       0x02000000
#define ALARM_UI_TIDAL_MISMATCH           0x04000000
#define ALARM_UI_IE_RATIO_MISMATCH        0x08000000
// bits 28 - 31 --

typedef struct data_packet_def {
  uint16_t sequence_count;            // bytes 0 - 1 - rpi unsigned short int
  uint8_t packet_version;             // byte 2      - rpi unsigned char
  uint8_t mode_value;                 // byte 3      - rpi unsigned char
  uint32_t respiratory_rate_measured; // bytes 4 - 7 - rpi unsigned int
  uint32_t respiratory_rate_set;      // bytes 8 - 11
  uint32_t tidal_volume_measured;     // bytes 12 - 15
  uint32_t tidal_volume_set;          // bytes 16 - 19
  uint32_t ie_ratio_measured;         // bytes 20 - 23
  uint32_t ie_ratio_set;              // bytes 24 - 27
  uint32_t peep_value_measured;       // bytes 28 - 31
  uint32_t peak_pressure_measured;    // bytes 32 - 35
  uint32_t plateau_value_measurement; // bytes 36 - 39
  uint32_t pressure_measured;         // bytes 40 - 43
  uint32_t flow_measured;             // bytes 44 - 47
  uint32_t volume_in_measured;        // bytes 48 - 51
  uint32_t volume_out_measured;       // bytes 52 - 55
  uint32_t volume_rate_measured;      // bytes 56 - 59
  uint8_t control_state;              // byte 60       - rpi unsigned char
  uint8_t battery_level;              // byte 61
  uint32_t reserved;                  // bytes 62 - 63 - rpi unsigned int
  uint32_t alarm_bits;                // bytes 64 - 67
  uint16_t crc;                       // bytes 68 - 69 - rpi unsigned short int
}__attribute__((packed));

typedef struct command_packet_def {
  uint16_t sequence_count;            // bytes 0 - 1 - rpi unsigned short int
  uint8_t packet_version;             // byte 2      - rpi unsigned char
  uint8_t mode_value;                 // byte 3      - rpi unsigned char
  uint32_t respiratory_rate_set;      // bytes 4 - 7 - rpi unsigned int
  uint32_t tidal_volume_set;          // bytes 8 - 11
  uint32_t ie_ratio_set;              // bytes 12 - 15
  uint32_t alarm_bits;                // bytes 16 - 19
  uint16_t crc;                       // bytes 20 - 21 - rpi unsigned short int  
}__attribute__((packed));

data_packet_def data_packet;
data_packet_def data_packet_last_sent;

//command_packet_def command_packet;
// this is used to handle byte by byte input
union incoming_packet_u {
  command_packet_def command_packet;
  uint8_t command_bytes[sizeof(command_packet)];
};


// data for handling serial reads
incoming_packet_u incoming_packet;
int incoming_index = 0;
//incoming_packet_u shared_command_packet; // this should be an extern in other modules that need to read this
//incoming_packet_u command_packet_reference; // this is saved in case other modules overwrite the shared copy
command_packet_def command_packet_reference;

command_packet_def public_command_packet;
data_packet_def public_data_packet;

#define POLY 0x8408
/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/
uint8_t crc_i;
uint16_t crc_data;
uint16_t crc = 0xffff;

uint16_t crc16(uint8_t *data_p, uint16_t length)
{
      //unsigned char i;
      //unsigned int data;
      //unsigned int crc = 0xffff;

      if (length == 0)
            return (~crc);

      do
      {
            for (crc_i=0, crc_data=(uint16_t)0xff & *data_p++;
                 crc_i < 8; 
                 crc_i++, crc_data >>= 1)
            {
                  if ((crc_i & 0x0001) ^ (crc_data & 0x0001))
                        crc = (crc >> 1) ^ POLY;
                  else  crc >>= 1;
            }
      } while (--length);

      crc = ~crc;
      crc_data = crc;
      crc = (crc << 8) | (crc_data >> 8 & 0xff);

      return (crc);
}

int serialHalInit(void)
{
  Serial.begin(115200);  // this function does not return anything
  // not sure how we can do something like while (!serial)
  // return MODULE_FAIL;
  return HAL_OK;
}

int serialHalGetData(void)
{
  while (Serial.available())
  {       
    inByte = Serial.read();
    
    // verify that the command is responding to the correct data packet by checking the sequence number and rev
    if (incoming_index == 0)
    {
      if (incoming_packet.command_packet.sequence_count != data_packet_last_sent.sequence_count)
      {
        // set alarm bit
        
        // do not increment incoming_index and return
        //return (HAL_OK);
      }
    }

    if (incoming_index == 1)
    {
      if (incoming_packet.command_packet.packet_version != data_packet_last_sent.packet_version)
      {
        // set alarm bit
        
        // reset incoming_index and return
        //incoming_index = 0;
        //return (HAL_OK);
        
      }      
    }
    incoming_packet.command_bytes[incoming_index] = inByte;
    incoming_index++;
    if (incoming_index >= sizeof(incoming_packet.command_bytes))
    {
      // save a copy for other modules, but keep a reference in case the shared copy gets modified
      memcpy((void *)&public_command_packet, (void *)&incoming_packet.command_packet, sizeof(public_command_packet));
      memcpy((void *)&command_packet_reference, (void *)&incoming_packet.command_packet, sizeof(command_packet_reference));
      incoming_index = 0;
      return HAL_OK;
    } 
  } // while serial.available
  return HAL_IN_PROGRESS;
}

int serialHalSendData()
{
  current_send_ms = millis();
  if ((current_send_ms - last_send_ms) >= send_interval_ms)
  {
    //data_packet.packet_version = '1';
    //unsigned short crc16(char *data_p, unsigned short length) 
    data_packet.packet_version = PACKET_VERSION;
    data_packet.crc = crc16((uint8_t *)&data_packet, sizeof(data_packet) - 2);
    memcpy((void *)&data_packet_last_sent, (void *)&data_packet, sizeof(data_packet_last_sent));
    memcpy((void *)&public_data_packet, (void *)&data_packet, sizeof(public_data_packet));
    bytesSent = Serial.write((byte *)&data_packet, sizeof(data_packet));
    if (bytesSent != sizeof(data_packet)) {
      // handle error
    }
    last_send_ms = current_send_ms;
  }
  return HAL_OK;
}

