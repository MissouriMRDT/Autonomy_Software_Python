
import struct
import socket
import binascii
import threading
import time

DRIVE_BOARD_IP   = "192.168.1.130"
DRIVE_DATA_ID    = {'left':100, 'right':101}
ROVECOMM_VERSION = 1
ROVECOMM_PORT    = 11000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Todo: Extract
        
def roveComm_SendMsgTo(dataID, data, dest_ip, port=ROVECOMM_PORT, seqNum=0x0F49, flags=0x00):
    """ Send a RoveComm formatted message
    
    Parameters
    -----------
        dataID: Short unsigned integer
        data: bytearray or bytestring 
        dest_ip: IP address to send to. String formatted like "192.168.1.1"
        port: Port number. 2 byte unsigned integer
        seqNum: Short unsigned integer. Not used in current revision of RoveComm
        flags: Unused in current version of RoveComm"""
    
    headerFormat = ">BHBHH"

    packetsize = len(data)
    
    header = struct.pack(headerFormat, 
                            ROVECOMM_VERSION,
                            seqNum,
                            flags,
                            dataID,
                            packetsize)
    msgbuffer = header + data
    #print "Message Buffer: ", binascii.hexlify(msgbuffer), "\n"    
    sock.sendto(msgbuffer, (dest_ip, port))

def sendMotorCommand(speed_left, speed_right):
    #print "Moving motors: Left ", speed_left, " Right", speed_right

    drive_packet_format = "<i" # Signed Int16
    
    assert(-1000 < speed_left < 1000)
    assert(-1000 < speed_right < 1000)
    if(speed_left == 0 and speed_right == 0):
        print "Zero'd"
    
    left_packet = struct.pack(drive_packet_format, speed_left)
    right_packet = struct.pack(drive_packet_format, speed_right)
    
    roveComm_SendMsgTo(DRIVE_DATA_ID['left'], left_packet, DRIVE_BOARD_IP)
    roveComm_SendMsgTo(DRIVE_DATA_ID['right'], right_packet, DRIVE_BOARD_IP)

def _clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
    
class Motors:
    def __init__(self):
        
        self._targetSpdLeft  = 0
        self._targetSpdRight = 0
        self._actualSpdLeft  = 0
        self._actualSpdRight = 0
        
        self.updateThread = threading.Thread(target=self._updateThreadFxn) 
        self.updateThread.setDaemon(True) # Automatically kill the thread when the program finishes
        self.updateThread.start()
            
    def __del__(self):
        self.disable()
        
    def move(self, speed, angle):
        """ Speed: -100 to 100
        Angle: -180 = turn in place left, 0 = straight, 180 = turn in place right """
        
        # Map speed and angle to (-1000, +1000) for each wheel
        speed_left = speed_right = speed / 100.0
        if(angle > 0):
            speed_right = speed_right * (1 - (angle / 180.0))
        elif(angle < 0):
            speed_left = speed_left * (1 + (angle / 180.0))
        
        self._targetSpdLeft  = _clamp(1000.0 * speed_left, -700, 700)
        self._targetSpdRight = _clamp(1000.0 * speed_right, -700, 700)
        
    def _updateThreadFxn(self):
        while 1:
            r = 4 # Response speed.
            
            if(self._actualSpdLeft < (self._targetSpdLeft - 2*r)):
                self._actualSpdLeft += r
            elif(self._actualSpdLeft > (self._targetSpdRight + 2*r)):
                self._actualSpdLeft -= r
            
            if(self._actualSpdRight < (self._targetSpdRight - 2*r)):
                self._actualSpdRight += r
            elif(self._actualSpdRight > (self._targetSpdRight + 2*r)):
                self._actualSpdRight -= r
            
            sendMotorCommand(self._actualSpdLeft, self._actualSpdRight)
            time.sleep(0.01)
    
    def disable(self):
        self.move(0, 0)
        
############################################
# Interactive testing mode
############################################    
    
import sys,tty,termios

class _Getch:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def get(motors):
    inkey = _Getch()
    prev = False
    while(1):
        print "Waiting for key"
        k=inkey()
        if k == '\x1b': # Arrow Key
            k=inkey() # Arrow keys are thee characters
            k=inkey() # Clear out buffer
            speed = 25
            if k=='A':
                print "up"
                motors.move(speed, 0)
            elif k=='B':
                print "down"
                motors.move(-speed, 0)
            elif k=='C':
                print "right"
                motors.move(speed, 180)
            elif k=='D':
                print "left"
                motors.move(speed, -180)
        elif k == ' ': # Space
            print "space"
            motors.move(0,0)
        elif k == '\x03': # Ctrl-C
            motors.disable()
            quit()
        else:
            print "Unexpected key ", k

if __name__=='__main__':
    motors = Motors()
    print "Starting motor tester"
    while True:
        get(motors)
    motors.disable()
        
        
        
        
