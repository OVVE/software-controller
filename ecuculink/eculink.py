import os
import sys
import getopt
import serial
import time
import datetime
import re
import pickle
try:
    import thread
except ImportError:
    import _thread as thread


VERSION = (0, 1, 1)


class ecuCuLink():

    def connectSerial(self) -> True:
        return True

    def device_exists(self, device: str) -> bool:
        try:
            for port in serial.tools.list_ports.comports():
                if port[0] == device:
                    return True

            return False

        except serial.SerialException:
            return os.path.exists(device)
