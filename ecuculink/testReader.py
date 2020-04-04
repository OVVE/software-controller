import serial
import sys
from time import sleep
BAUD = 38400
PORT = "/dev/ttyACM0"
SER_TIMEOUT = 0.06


#Global Variables
ser = 0
dicA = {'ver': '', 'seq': '' }

#Function to Initialize the Serial Port
def init_serial():
    
    global ser          #Must be declared in Each Function
    ser = serial.Serial()
    ser.baudrate = BAUD
    ser.port = PORT 
    #a little less than polling spped from arduino
    ser.timeout = SER_TIMEOUT
    ser.open()          #Opens SerialPort

    # print port open or closed
    if ser.isOpen():
        print ('Open: ' + ser.portstr)
    sleep(1)    
#Function Ends Here
        

#Call the Serial Initilization Function, Main Program Starts from here
init_serial()



def read_all(port, chunk_size=200):
    """Read all characters on the serial port and return them."""
    if not port.timeout:
        raise TypeError('Port needs to have a timeout set!')

    read_buffer = b''

    while True:
        # Read in chunks. Each chunk will wait as long as specified by
        # timeout. Increase chunk_size to fail quicker
        byte_chunk = port.read(size=chunk_size)
        read_buffer += byte_chunk
        if not len(byte_chunk) == chunk_size:
            break

    return read_buffer

#ser.flush()
count = 0
def process_in_serial():
    global ser
    in_pkt={'sequence_count': 0,            # bytes 1 - 2 - rpi unsigned short int
            'packet_version': 0,             # byte 3      - rpi unsigned char
            'mode_value': 0,                 # byte 4      - rpi unsigned char
            'respiratory_rate_measured': 0, # bytes 5 - 8 - rpi unsigned int
            'respiratory_rate_set': 0,      # bytes 9 - 12
            'tidal_volume_measured': 0,     # bytes 13 - 16
            'tidal_volume_set': 0,          # bytes 17 - 20
            'ie_ratio_measured': 0,         # bytes 22 - 24
            'ie_ratio_set': 0,              # bytes 25 - 28
            'peep_value_measured': 0,       # bytes 29 - 32
            'peak_pressure_measured': 0,    # bytes 33 - 36
            'plateau_value_measurement': 0, # bytes 37 - 40
            'pressure_measured': 0,         # bytes 41 - 44
            'flow_measured': 0,             # bytes 45 - 48
            'volume_in_measured': 0,        # bytes 49 - 52
            'volume_out_measured': 0,       # bytes 53 - 56
            'volume_rate_measured': 0,      # bytes 57 - 60
            'control_state': 0,              # byte 61       - rpi unsigned char
            'battery_level': 0,              # byte 62
            'reserved': 0,                  # bytes 63 - 64 - rpi unsigned int
            'alarm_bits': 0,                # bytes 65 - 68
            'crc': 0 }                      # bytes 69 - 70 

    cmd_pkt = {'start_byte':b'\xff',
                'sequence_count': b'\x00\x00',               # bytes 1 - 2 - rpi unsigned short int
                'packet_version': b'\x01',                         # byte 3      - rpi unsigned char
                'mode_value': b'\x00',                             # byte 4      - rpi unsigned char
                'respiratory_rate_set': b'\x00\x00\x00\x00', # bytes 5 - 8 - rpi unsigned int
                'tidal_volume_set':  b'\x00\x00\x00\x00',         # bytes 9 - 12
                'ie_ratio_set':  b'\x00\x00\x00\x00',             # bytes 13 - 16
                'alarm_bits':   b'\x00\x00\x00\x00',              # bytes 17 - 20
                'crc':   b'\x00\x00' }                    # bytes 21 - 22 - rpi unsigned short int  
    
           
    ser.reset_input_buffer()
    error_count = 0
    while 1:
        byteData = read_all(ser, 71)
        #byteData = ser.read(72)
        #2 ways to print
        #print (byteData)  #raw will show ascii if can be decoded
        #hex only -- byte order is reversed
        #print(''.join(r'\x'+hex(letter)[2:] for letter in byteData))
        if byteData[0] == 0xff:
            in_pkt['sequence_count']=int.from_bytes(byteData[1:3], byteorder='little')
            in_pkt['packet_version']=byteData[3]
            in_pkt['mode_value']=byteData[4]
            in_pkt['respiratory_rate_measured']=int.from_bytes(byteData[5:9], byteorder='little')
            in_pkt['respiratory_rate_set']=int.from_bytes(byteData[9:13], byteorder='little')
            in_pkt['tidal_volume_measured']=int.from_bytes(byteData[13:17], byteorder='little')
            in_pkt['tidal_volume_set']=int.from_bytes(byteData[9:13], byteorder='little')
            in_pkt['ie_ratio_measured']=int.from_bytes(byteData[13:17], byteorder='little')
            in_pkt['ie_ratio_set']=int.from_bytes(byteData[9:13], byteorder='little')
            in_pkt['crc']=int.from_bytes(byteData[69:71], byteorder='little')
            
            print (in_pkt['sequence_count'])
            cmd_pkt['sequence_count'] = in_pkt['sequence_count']
            
        else:
            error_count = error_count + 1
            print ('drop packets count ' + str(error_count))
        cmd_byteData = b""

        
        # cmd_byteData.append(255)
        # cmd_byteData.append(cmd_pkt['sequence_count'])
        # cmd_byteData.append(int.from_bytes(cmd_pkt['packet_version'], byteorder='little'))
        # cmd_byteData.append(int.from_bytes(cmd_pkt['mode_value'], byteorder='little'))
        # cmd_byteData.append(int.from_bytes(cmd_pkt['respiratory_rate_set'], byteorder='little'))
        # cmd_byteData.append(int.from_bytes(cmd_pkt['tidal_volume_set'], byteorder='big'))
        # cmd_byteData.append(int.from_bytes(cmd_pkt['ie_ratio_set'], byteorder='big'))
        # cmd_byteData.append(int.from_bytes(cmd_pkt['alarm_bits'], byteorder='big'))
        # cmd_byteData.append(int.from_bytes(cmd_pkt['crc'], byteorder='big'))

        ser.write(cmd_byteData)
        print (cmd_byteData)
        
process_in_serial()


