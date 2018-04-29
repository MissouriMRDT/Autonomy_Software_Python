import serial
import pynmea2
import threading
import time
from collections import namedtuple

Coordinate = namedtuple('Coordinate', ['lat', 'lon'])

class GPS:
    def __init__(self, serial_port="/dev/ttyS0"):
        self.ser = serial.Serial(serial_port, 9600) 
        self.ser.flushInput()
        
        # Will block until a fix is available
        self._location = self._getReading()
        
        self.updateThread = threading.Thread(target=self._updateThreadFxn) 
        self.updateThread.setDaemon(True) # Automatically kill the thread when the program finishes
        self.updateThread.start()
        
    def location(self):
        """ Get current location
        
            Returns
            -------
            location : Coordinate
                (latitude, longitude)
                longitude and latitude are floats specified in decimal degrees, WGS84 Datum
                """
        return self._location
        
    def _getReading(self):    
        # Blocks until a message is read
        msg = ""
        while not hasattr(msg, 'latitude'):
            try:
                nmea_string = self.ser.readline()
                msg = pynmea2.parse(nmea_string)
            except:
                pass
        self.lastFixTime = time.time()    
        return Coordinate(msg.latitude, msg.longitude)
        
    def _updateThreadFxn(self):
        while True:
            self._location = self._getReading()
        
if __name__ == "__main__":
    import time
    gps = GPS()
    while True:
        print("Location: ", gps.location())
        time.sleep(0.1)
