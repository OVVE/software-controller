import serial
import sys


#Global Variables
ser = 0
dicA = {'ver': '', 'seq': '' }

#Function to Initialize the Serial Port
def init_serial():
    
    global ser          #Must be declared in Each Function
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = "/dev/ttyACM0" 
    #a little less than polling spped from arduino
    ser.timeout = 0.9
    ser.open()          #Opens SerialPort

    # print port open or closed
    if ser.isOpen():
        print ('Open: ' + ser.portstr)
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

ser.flush()
count = 0
while 1:
    byteData = read_all(ser, 70)
    #byteData = ser.read(70)
    #2 ways to print
    print (byteData)  #raw will show ascii if can be decoded
    #hex only -- byte order is reversed
    print(''.join(r'\x'+hex(letter)[2:] for letter in byteData))
    # Workaround
    try:
        dicA['ver']=byteData[4]
    except:
        print()
    finally:
        print()
    #
    dicA['seq']=int.from_bytes(byteData[0:2], byteorder='little')
    end = None
    dicA['crc']=int.from_bytes(byteData[72:None], byteorder='little')
    
    print (dicA)