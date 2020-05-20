// TODO: Document what this module does
#ifndef __LINK_MODULE_H__
#define __LINK_MODULE_H__

#include <stdint.h>
#include <stdbool.h>
#include "../util/alarm.h"

// alarm bits to set that are triggered by the microcontroller
// link.cpp should check modules and set these bits
#define ALARM_ECU_POWER_LOSS              0x01
#define ALARM_ECU_LOW_BATTERY             0x02 
#define ALARM_ECU_BAD_PRESSURE_SENSOR     0x04
#define ALARM_ECU_BAD_FLOW_SENSOR         0x08
#define ALARM_ECU_COMMUNICATION_FAILURE   0x10
#define ALARM_ECU_HARDWARE_FAILURE        0x20
#define ALARM_ECU_ESTOP_PRESSED           0x40
#define ALARM_ECU_HIGH_PRESSURE           0x100 
#define ALARM_ECU_LOW_PRESSURE            0x200
#define ALARM_ECU_HIGH_VOLUME             0x400
#define ALARM_ECU_LOW_VOLUME              0x800
#define ALARM_ECU_HIGH_RESPIRATORY_RATE   0x1000
#define ALARM_ECU_LOW_RESPIRATORY_RATE    0x2000

#define ALARM_UI_COMMUNICATION_FAILURE    0x10000
#define ALARM_UI_HARDWARE_FAILURE         0x20000
#define ALARM_UI_SETPOINT_MISMATCH        0x1000000

// modes
#define MODE_VC_CMV  0x00
#define MODE_SIMV    0x01

// battery charging state bit
#define BATTERY_CHARGING 0x80

#define PACKET_VERSION 3

// packet_type
#define PACKET_TYPE_DATA 1
#define PACKET_TYPE_COMMAND 2
#define PACKET_TYPE_FIRMWARE 3

// control state
#define CONTROL_STATE_IDLE             0x00
#define CONTROL_STATE_BEGIN_INHALATION 0x01
#define CONTROL_STATE_INHALATION       0x02
#define CONTROL_STATE_BEGIN_HOLD       0x03
#define CONTROL_STATE_HOLD             0x04
#define CONTROL_STATE_BEGIN_EXHALATION 0x05
#define CONTROL_STATE_EXHALATION       0x06

#define COMMAND_BIT_START 0x01
#define COMMAND_BIT_FW    0x04

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
  uint16_t volumeRequested;
  uint16_t respirationRateRequested;
  uint16_t ieRatioRequested;
  uint16_t pressureRequested;
  uint16_t highVolumeLimit;
  uint16_t lowVolumeLimit;
  uint16_t highPressureLimit;
  uint16_t lowPressureLimit;
  uint16_t highRespiratoryRateLimit;
  uint16_t lowRespiratoryRateLimit;
  
  // Alarms
  struct alarm onCommunicationFailureAlarm;
};

// Public Variables
extern struct link comm;

// TODO: Doc
int linkModuleInit(void);

// TODO: Doc
int linkModuleRun(void);

#endif /* __LINK_MODULE_H__ */
