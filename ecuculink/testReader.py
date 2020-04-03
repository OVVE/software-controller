import serial
import sys
from time import sleep
BAUD = 115200
PORT = "/dev/ttyACM0"
SER_TIMEOUT = .9


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
    in_pkt={'sequence_count': 0,            # bytes 0 - 1 - rpi unsigned short int
            'packet_version': 0,             # byte 2      - rpi unsigned char
            'mode_value': 0,                 # byte 3      - rpi unsigned char
            'respiratory_rate_measured': 0, # bytes 4 - 7 - rpi unsigned int
            'respiratory_rate_set': 0,      # bytes 8 - 11
            'tidal_volume_measured': 0,     # bytes 12 - 15
            'tidal_volume_set': 0,          # bytes 16 - 19
            'ie_ratio_measured': 0,         # bytes 20 - 23
            'ie_ratio_set': 0,              # bytes 24 - 27
            'peep_value_measured': 0,       # bytes 28 - 31
            'peak_pressure_measured': 0,    # bytes 32 - 35
            'plateau_value_measurement': 0, # bytes 36 - 39
            'pressure_measured': 0,         # bytes 40 - 43
            'flow_measured': 0,             # bytes 44 - 47
            'volume_in_measured': 0,        # bytes 48 - 51
            'volume_out_measured': 0,       # bytes 52 - 55
            'volume_rate_measured': 0,      # bytes 56 - 59
            'control_state': 0,              # byte 60       - rpi unsigned char
            'battery_level': 0,              # byte 61
            'reserved': 0,                  # bytes 62 - 63 - rpi unsigned int
            'alarm_bits': 0,                # bytes 64 - 67
            'crc': 0 } 
    ser.reset_input_buffer()
    while 1:
        byteData = read_all(ser, 70)
        #byteData = ser.read(70)
        #2 ways to print
        #print (byteData)  #raw will show ascii if can be decoded
        #hex only -- byte order is reversed
        print(''.join(r'\x'+hex(letter)[2:] for letter in byteData))
        #Workaround
        # try:
        #     in_pkt['ver']=byteData[4]
        # except:
        #     pass
        # finally:
        #     pass
        
        in_pkt['sequence_count']=int.from_bytes(byteData[0:2], byteorder='little')
        in_pkt['packet_version']=byteData[4]
        in_pkt['mode_value']=byteData[3]
        in_pkt['crc']=int.from_bytes(byteData[72:None], byteorder='little')
        
        print (in_pkt)
process_in_serial()