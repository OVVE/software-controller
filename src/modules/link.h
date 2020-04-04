
#ifndef __LINK_MODULE_H__
#define __LINK_MODULE_H__

#include <stdbool.h>
#include <Arduino.h>

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


#define PACKET_VERSION 1

typedef struct data_packet_def {
  uint8_t start_byte; //                 byte  0 it could become difficult for Rpi to sync up so this was added
  uint16_t sequence_count;            // bytes 1 - 2 - rpi unsigned short int
  uint8_t packet_version;             // byte 3      - rpi unsigned char
  uint8_t mode_value;                 // byte 4      - rpi unsigned char
  uint32_t respiratory_rate_measured; // bytes 5 - 8 - rpi unsigned int
  uint32_t respiratory_rate_set;      // bytes 9 - 12
  uint32_t tidal_volume_measured;     // bytes 13 - 16
  uint32_t tidal_volume_set;          // bytes 17 - 20
  uint32_t ie_ratio_measured;         // bytes 21 - 24
  uint32_t ie_ratio_set;              // bytes 25 - 28
  uint32_t peep_value_measured;       // bytes 29 - 32
  uint32_t peak_pressure_measured;    // bytes 33 - 36
  uint32_t plateau_value_measurement; // bytes 37 - 40
  uint32_t pressure_measured;         // bytes 41 - 44
  uint32_t flow_measured;             // bytes 45 - 48
  uint32_t volume_in_measured;        // bytes 49 - 52
  uint32_t volume_out_measured;       // bytes 53 - 56
  uint32_t volume_rate_measured;      // bytes 57 - 60
  uint8_t control_state;              // byte 61       - rpi unsigned char
  uint8_t battery_level;              // byte 62
  uint16_t reserved;                  // bytes 63 - 64 - rpi unsigned int
  uint32_t alarm_bits;                // bytes 65 - 68
  uint16_t crc;                       // bytes 69 - 70 - rpi unsigned short int
}__attribute__((packed));

typedef struct command_packet_def {
  uint8_t start_byte;
  uint16_t sequence_count;            // bytes 0 - 1 - rpi unsigned short int
  uint8_t packet_version;             // byte 2      - rpi unsigned char
  uint8_t mode_value;                 // byte 3      - rpi unsigned char
  uint32_t respiratory_rate_set;      // bytes 4 - 7 - rpi unsigned int
  uint32_t tidal_volume_set;          // bytes 8 - 11
  uint32_t ie_ratio_set;              // bytes 12 - 15
  uint32_t alarm_bits;                // bytes 16 - 19
  uint16_t crc;                       // bytes 20 - 21 - rpi unsigned short int  
}__attribute__((packed));

struct link {
  bool update;
  bool assist;
  unsigned int volumeRequested;
  unsigned int respirationRateRequested;
  unsigned int ieRatioRequested;
};

// Public Variables
extern struct link comm;
extern uint16_t sequence_count;
extern uint16_t last_sequence_count;
extern command_packet_def command_packet;
extern command_packet_def command_packet_reference;
extern command_packet_def command_packet;
extern bool watchdog_exceeded;
extern bool watchdog_active;
extern bool clear_input; // if there is a problem with data then clear the remaining bytes
extern bool ready_to_send;

extern data_packet_def public_data_packet;
extern command_packet_def public_command_packet;

// TODO: Doc
int linkModuleInit(void);

// TODO: Doc
int linkModuleRun(void);

#endif /* __LINK_MODULE_H__ */