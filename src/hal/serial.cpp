#include <Arduino.h>
#include <crc16.h>

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"

#define MaxLength 200

byte BufferLength;
String Buffer;
char SendBuffer[MaxLength + 1];

uint32_t last_send_ms = 0;  // this is used for send interval
uint32_t current_send_ms = 0;
uint32_t send_interval_ms = 1000;
uint16_t bytesSent;
uint16_t inByte;

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

union incoming_packet_u {
  command_packet_def command_packet;
  uint8_t command_bytes[sizeof(command_packet)];
};


// data for handling serial reads
incoming_packet_u incoming_packet;
int incoming_index = 0;
incoming_packet_u shared_command_packet; // this should be an extern in other modules that need to read this
incoming_packet_u command_packet_reference; // this is saved in case other modules overwrite the shared copy

int serialHalInit(void)
{
  SendBuffer[0] = NULL;
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
    incoming_packet.command_bytes[incoming_index] = inByte;
    incoming_index++;
    if (incoming_index >= sizeof(incoming_packet.command_bytes))
    {
      // save a copy for other modules, but keep a reference in case the shared copy gets modified
      memcpy(shared_command_packet.command_bytes, incoming_packet.command_bytes, sizeof(incoming_packet.command_bytes));
      memcpy(command_packet_reference.command_bytes, incoming_packet.command_bytes, sizeof(incoming_packet.command_bytes));
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
    data_packet.packet_version = '1';
    memcpy((void *)&data_packet_last_sent, (void *)&data_packet, sizeof(data_packet_last_sent));
    bytesSent = Serial.write((byte *)&data_packet, sizeof(data_packet));
    if (bytesSent != sizeof(data_packet)) {
      // handle error
    }
    last_send_ms = current_send_ms;
  }
  return HAL_OK;
}