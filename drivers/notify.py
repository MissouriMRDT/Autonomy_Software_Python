import time
import struct

from drivers.rovecomm import RoveComm


DRIVE_BOARD_IP = "192.168.1.130"
NOTIFY_ID = -1


class Notify:

    def __init__(self, rove_comm):
        self.rove_comm_node = rove_comm

    def _notify(self, val):
        data = struct.pack("<h", val)
        self.rove_comm_node.sendTo(NOTIFY_ID, data, DRIVE_BOARD_IP)

    def notify_finish(self):
        self._notify(1)
        time.sleep(2)
        self._notify(0)


if __name__ == '__main__':
    rove_comm_node = RoveComm()
    notify = Notify(rove_comm_node)

    notify.notify_finish()
