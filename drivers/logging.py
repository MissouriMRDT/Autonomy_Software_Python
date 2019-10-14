import struct
import constants
import time
from drivers.rovecomm import RoveCommPacket

class LogWriter:
    def __init__(self,filename):
        self.filename = filename
        with open(self.filename, 'w') as f:
            f.write(time.strftime("%Y%m%d-%H%M%S") + "\n")
    
        
    def __del__(self):
        with open(self.filename, 'a') as f:
            f.write("Quitting\n")
    
    def write_line(self, line):
        with open(self.filename, 'a') as f:
            f.write(line + "\n")
        
        
if __name__ == '__main__':
    logggg = LogWriter("/testlog.txt")
    logggg.write_log("testytest")
