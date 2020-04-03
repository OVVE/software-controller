import serial
import sys
from time import sleep
ser = 0
BAUD = 115200
PORT = "/dev/ttyACM0"
SER_TIMEOUT = .9

class vent_monitor():

    # Implement access and throughput
    #   between Arduino controller for the ventilator
    #   and the UI to the monitor.

    
    def __init__(self, init_setting):
        setting_curr = init_setting
        setting_new = None
        prev_seq_no = None
        init_serial()

    def process_in_serial():
        global ser
        in_pkt={'sequence_count': 0,            # bytes 0 - 1 - rpi unsigned short
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
                'control_state': 0,              # byte 60       - rpi unsigned ch
                'battery_level': 0,              # byte 61
                'reserved': 0,                  # bytes 62 - 63 - rpi unsigned int
                'alarm_bits': 0,                # bytes 64 - 67
                'crc': 0 } 
        while 1:
            byteData = read_all(ser, 70)
            #byteData = ser.read(70)
            #2 ways to print
            #print (byteData)  #raw will show ascii if can be decoded
            #hex only -- byte order is reversed
            #print(''.join(r'\x'+hex(letter)[2:] for letter in byteData))
            # Workaround
            try:
                in_pkt['ver']=byteData[4]
            except:
                pass
            finally:
                pass
            #
            in_pkt['seq']=int.from_bytes(byteData[0:2], byteorder='little')
            end = None
            in_pkt['crc']=int.from_bytes(byteData[72:None], byteorder='little')
            in_pkt['mode']=byteData[3]
            print (in_pkt)

    def recv_arduino_packet(self, arduino_packet):
        """
        Purpose:
            Call upon receiving a new Arduino packet
            Convert packet data to a dict; feed to this method.
            ALTERNATIVE: add code here to convert byts data to dict.

        Operation:
            Extract data dict into command dict
            Convert to 22-byte command packet
            Check sequence and timing
            Ack receipt with command packet
            Send command packet to monitor

        Parameters:
            arduino_packet
                A dict of the data packet that comes from the Arduino device
                every 10ms
        """

        # Check sequence number; if not consecutive, raise alarm.
        # Extract reportable data (see data format data strcuture.
        # Copy command data to command structure.
        # Send command data to UI monitor.
        # Check for settings updates;
        #   if found, update command structure.
        # Send command structure back to arduino

        # RACE CONDITION NOTE:
        #    It is theoretically possible for a reset order to intervene
        # between updating the command dict with an existing reset,
        # and clearing the setting_new attribute.  This would lose the second
        # reset data.
        # This problem would require the following sequence:
        # Receive & process data packet #1
        # Receive reset A
        # Receive data packet #2
        # Extract data packet #2 into command dict
        # Update command dict from reset dict A
        # Receive reset B
        # Clear reset dict, losing reset B
        # Finish processing data packet #2
        # Receive and process data packet #3 (which should handle reset B)
        # -----
        #   This requires receiving resets A & B at very restricted time
        # intervals between receiving packet 1 and completing packet 2.
        # The total elapsed time is barely over 10 milliseconds, which
        # presumes programmatic resets -- these are not part of the flow.
        # CONCLUSION: the race condition cannot arise from human control.

        # NOEL: INSERT CODE TO EXTRACT DATA DICT FROM PACKET

        # Extract command dict
        extract_field = [
            "seq_no",
            "packet_ver",
            "mode",
            "resp_rate",
            "tidal_vol",
            "IE_ratio",
            "alarm_bitmap",
            "CRC",
        ]
        command = {field: arduino_packet[field]
                   for field in extract_field}

        # Check sequence number; update previous
        curr_seq_no = command["seq_no"]
        if prev_seq_no is not None and \
           curr_seq_no != prev_seq_no + 1:
            # Raise alarm for lost packet(s)
            # THIS FUNCTION IS NOT YET IMPLEMENTED
            report_missing_packets(prev_seq_no, curr_seq_no)
        prev_seq_no = curr_seq_no

        # Send command data to UI monitor
        # THIS FUNCTION IS NOT YET IMPLEMENTED
        send_command_to_UI(command)
        
        # Check for settings updates;
        #   if found, update command structure.
        # Send command structure back to arduino
        if setting_new is not None:
            command.update(setting_new)
            # Race condition (see comment above)
            setting_new = None
        # THIS FUNCTION IS NOT YET IMPLEMENTED
        send_command_to_arduino(command)
        

    def update_setting(resp_rate, tidal_vol, IE_ratio):
        """
        Save the given settings into the "drop box" variable.
        If, by some chance, there is already data in the settings,
          there is no alert.
        """

        setting_new = locals()


    def report_missing_packets(seq1, seq2):
        msg = "Missing packets between %d and %d, exclusive." % (seq1, seq2)
        # HOW DO WE PUSH THIS ALERT TO THE PROPER PLACE?
        pass


    # NOEL: These are better moved to a class that deals with the
    # data and command packets.
    
    def send_command_to_UI(command):
        # HOW DO WE PUSH THIS COMMAND TO THE UI MONITOR?
        pass


    def send_command_to_arduino(command):
        # HOW DO WE PUSH THIS COMMAND ACK TO THE ARDUINO?
        pass
    
    #CRC implmentation

    def crc_check_in(in_packet)
        # NOEL: to implement CCIT CRC16
        pass
    
    def crc_create_out(out_packet)
        # NOEL: to implement CCIT CRC16
        pass
    
    def init_serial():
    
    global ser          #Must be declared in Each Function
    ser = serial.Serial()
    ser.baudrate = BAUD
    ser.port = PORT 
    #a little less than polling speed from arduino
    ser.timeout = SER_TIMEOUT
    ser.open()          #Opens SerialPort

    # print port open or closed
    if ser.isOpen():
        print ('Open: ' + ser.portstr)
    sleep(1)
    #Function Ends Here
        

    #Call the Serial Initilization Function
    #init_serial()



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

