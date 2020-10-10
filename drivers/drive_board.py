
import struct
from drivers.rovecomm import RoveCommPacket
import constants

DRIVE_BOARD_IP = "134"
DRIVE_DATA_ID = 1000
#left/right between -1000,1000

def clamp(n, min_n, max_n):
    return max(min(max_n, n), min_n)


class DriveBoard:
    def __init__(self):
        
        self._targetSpdLeft = 0
        self._targetSpdRight = 0
        self.enabled = False
        
    def __del__(self):
        self.disable()
        
    def calculate_move(self, speed, angle):
        """ Speed: -1000 to 1000
        Angle: -360 = turn in place left, 0 = straight, 360 = turn in place right """

        speed_left = speed_right = speed

        if(angle > 0):
            speed_right = speed_right * (1 - (angle / 180.0))
        elif(angle < 0):
            speed_left = speed_left * (1 + (angle / 180.0))
        
        self._targetSpdLeft = int(clamp(speed_left, -constants.DRIVE_POWER, constants.DRIVE_POWER))
        self._targetSpdRight = int(clamp(speed_right, -constants.DRIVE_POWER, constants.DRIVE_POWER))

        if self.enabled:
            return self._targetSpdLeft, self._targetSpdRight
        return 0, 0

    def send_drive(self, target_left, target_right):
        return RoveCommPacket(DRIVE_DATA_ID, 'h', (target_left, target_right), ip_octet_4=DRIVE_BOARD_IP)

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True


if __name__ == '__main__':
    motors = DriveBoard("testytest.txt")
    motors.enable()
    for i in range(-180, 181):
        left, right = motors.calculate_move(100, i)
        motors.send_drive(left, right)
        print(left, right)


