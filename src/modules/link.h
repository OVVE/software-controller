// TODO: Document what this module does
#ifndef __LINK_MODULE_H__
#define __LINK_MODULE_H__

#include <stdint.h>
#include <stdbool.h>


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

// bit mask
#define MODE_NON_ASSIST  0x00
#define MODE_ASSIST      0x01
#define MODE_SIM         0x02

// toggle bit for start stop
#define MODE_START_STOP  0x80


#define PACKET_VERSION 3

typedef struct __attribute__((packed)) {
  uint16_t sequence_count;                   // bytes 0 - 1
  uint8_t packet_version;                    // byte 2
  uint8_t packet_type;                       // byte 3
  
  uint8_t mode_value;                        // byte 4
  uint8_t control_state;                     // byte 5
  uint8_t battery_status;                    // byte 6
  
  uint8_t reserved;                          // byte 7
  
  uint16_t respiratory_rate_set;             // bytes 8 - 9  
  uint16_t respiratory_rate_measured;        // bytes 10 - 11

  int16_t tidal_volume_set;                  // bytes 12 - 13
  int16_t tidal_volume_measured;             // bytes 14 - 15
  
  uint16_t ie_ratio_set;                     // bytes 16 - 17
  uint16_t ie_ratio_measured;                // bytes 18 - 19
  
  int16_t peep_value_measured;               // bytes 20- 21
  
  int16_t peak_pressure_measured;            // bytes 22 - 23
  
  int16_t plateau_value_measurement;         // bytes 24 - 25
  
  int16_t pressure_set;                      // bytes 26 - 27
  int16_t pressure_measured;                 // bytes 28 - 29
  
  int16_t flow_measured;                     // bytes 30 - 31
  
  int16_t volume_in_measured;                // bytes 32 - 33
  int16_t volume_out_measured;               // bytes 34 - 35
  
  int16_t volume_rate_measured;              // bytes 36 - 37
  
  int16_t high_pressure_limit_set;           // bytes 38 - 39
  int16_t low_pressure_limit_set;            // bytes 40 - 41
  
  int16_t high_volume_limit_set;             // bytes 42 - 43
  int16_t low_volume_limit_set;              // bytes 44 - 45

  int16_t high_respiratory_rate_limit_set;   // bytes 46 - 47
  int16_t low_respiratory_rate_limit_set;    // bytes 48 - 49
  
  uint32_t alarm_bits;                       // bytes 50 - 53
  uint16_t crc;                              // bytes 54 - 55
} data_packet_def;

typedef struct __attribute__((packed)) {
  uint16_t sequence_count;                   // bytes 0 - 1
  uint8_t packet_version;                    // byte 2
  uint8_t packet_type;                       // byte 3
  
  uint8_t mode_value;                        // byte 4
  
  uint8_t command;                           // byte 5
  
  uint16_t reserved;                         // bytes 6 - 7
  
  uint16_t respiratory_rate_set;      // bytes 8 - 9
  
  int16_t tidal_volume_set;           // bytes 10 - 11
  
  uint16_t ie_ratio_set;              // bytes 12 - 13
  
  int16_t pressure_set;               // bytes 14 - 15
  
  int16_t high_pressure_limit_set;    // bytes 16 - 17
  int16_t low_pressure_limit_set;     // bytes 18 - 19

  int16_t high_volume_limit_set;      // bytes 20 - 21
  int16_t low_volume_limit_set;       // bytes 22 - 23

  int16_t high_respiratory_rate_limit_set;   // bytes 24 - 25
  int16_t low_respiratory_rate_limit_set;    // bytes 26 - 27
  
  uint32_t alarm_bits;                // bytes 28 - 31
  uint16_t crc;                       // bytes 32 - 33  
} command_packet_def;

struct link {
  // Variables
  uint8_t  startVentilation;
  uint8_t  ventilationMode;
  uint32_t volumeRequested;
  uint32_t respirationRateRequested;
  uint32_t ieRatioRequested;
  
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
