import struct
from drivers.rovecomm import RoveComm

SPEED_LIMIT = 300
DRIVE_BOARD_IP = "192.168.1.130"
DRIVE_DATA_ID = 528


def clamp(n, min_n, max_n):
    return max(min(max_n, n), min_n)


class DriveBoard:
    def __init__(self, rove_comm):

        self.rove_comm_node = rove_comm
        
        self._targetSpdLeft = 0
        self._targetSpdRight = 0
        self.enabled = False
        
    def __del__(self):
        self.disable()
        
    def calculate_move(self, speed, angle):
        """ Speed: -100 to 100
        Angle: -180 = turn in place left, 0 = straight, 180 = turn in place right """
        
        # Map speed and angle to (-1000, +1000) for each wheel
        speed_left = speed_right = speed / 100.0
        if angle > 0:
            speed_right = speed_right * (1 + (angle / 180.0))
        elif angle < 0:
            speed_left = speed_left * (1 - (angle / 180.0))
        
        self._targetSpdLeft = int(clamp(1000.0 * speed_left, -SPEED_LIMIT, SPEED_LIMIT))
        self._targetSpdRight = int(clamp(1000.0 * speed_right, -SPEED_LIMIT, SPEED_LIMIT))

        if self.enabled:
            return self._targetSpdLeft, self._targetSpdRight
        return 0, 0

    def send_drive(self, target_left, target_right):
        data = struct.pack("<hh", target_left, target_right)
        self.rove_comm_node.sendTo(DRIVE_DATA_ID, data, DRIVE_BOARD_IP)

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True


if __name__ == '__main__':
    rove_comm_node = RoveComm()
    motors = DriveBoard(rove_comm_node)
    motors.enable()
    for i in range(-180, 180):
        left, right = motors.calculate_move(100, 0)
        motors.send_drive(left, right)
        print(left, right)

