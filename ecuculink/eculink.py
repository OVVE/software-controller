import os
import sys
import getopt
import serial
import time
import datetime
import re
import pickle


VERSION = (0, 1, 1)

try:
    import thread
except ImportError:
    import _thread as thread
class ecuCuLink:

    def connectSerial(self):
        return True

    def device_exists(self,device):
        try:
            from serial.tools import list_ports

            for port in list_ports.comports():
                if port[0] == device:
                    return True

            return False
        except serial.SerialException:
            return os.path.exists(device)
