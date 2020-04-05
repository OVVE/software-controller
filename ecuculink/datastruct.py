ecuDataPacket = {
    'sequence_count': 0,             # bytes 0 - 1   - rpi unsigned short int
    'packet_version': 0,             # byte 2        - rpi unsigned char
    'mode_value': 0,                 # byte 3        - rpi unsigned char
    'respiratory_rate_measured': 0,  # bytes 4 - 7   - rpi unsigned int
    'respiratory_rate_set': 0,       # bytes 8 - 11
    'tidal_volume_measured': 0,      # bytes 12 - 15
    'tidal_volume_set': 0,           # bytes 16 - 19
    'ie_ratio_measured': 0,          # bytes 20 - 23
    'ie_ratio_set': 0,               # bytes 24 - 27
    'peep_value_measured': 0,        # bytes 28 - 31
    'peak_pressure_measured': 0,     # bytes 32 - 35
    'plateau_value_measurement': 0,  # bytes 36 - 39
    'pressure_measured': 0,          # bytes 40 - 43
    'flow_measured': 0,              # bytes 44 - 47
    'volume_in_measured': 0,         # bytes 48 - 51
    'volume_out_measured': 0,        # bytes 52 - 55
    'volume_rate_measured': 0,       # bytes 56 - 59
    'control_state': 0,              # byte 60       - rpi unsigned char
    'battery_level': 0,              # byte 61
    'reserved': 0,                   # bytes 62 - 63 - rpi unsigned int
    'alarm_bits': 0,                 # bytes 64 - 67
    'crc': 0
}  
print(ecuDataPacket)

uiCommandPacket = {
    'sequence_count': 0,        # bytes 0 - 1   - rpi unsigned short int
    'packet_version': 0,        # byte 2        - rpi unsigned char
    'mode_value': 0,            # byte 3        - rpi unsigned char
    'respiratory_rate_set': 0,  # bytes 4 - 7   - rpi unsigned int
    'tidal_volume_set': 0,      # bytes 8 - 11
    'ie_ratio_set': 0,          # bytes 12 - 15
    'alarm_bits': 0,            # bytes 16 - 19
    'crc': 0                    # bytes 20 - 21 - rpi unsigned short int
}
print(uiCommandPacket)