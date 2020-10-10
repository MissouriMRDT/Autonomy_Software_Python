import time
import struct

from drivers.rovecomm import RoveCommEthernetUdp, RoveCommPacket


DRIVE_BOARD_IP = "192.168.1.142"
NOTIFY_ID = 7000


class Notify:

    def __init__(self, rove_comm):
        self.rove_comm_node = rove_comm

    def _notify(self, val):
        self.rove_comm_node.write(RoveCommPacket(NOTIFY_ID, 'b', (val,0), ip_octet_4='142'))

    def notify_finish(self):
        self._notify(50)
        time.sleep(1)
        self._notify(0)
        time.sleep(1)
        self._notify(50)
        time.sleep(1)
        self._notify(0)
        time.sleep(1)
        self._notify(50)
        time.sleep(5)
        self._notify(0)


if __name__ == '__main__':
    rove_comm_node = RoveCommEthernetUdp("oof.txt")
    notify = Notify(rove_comm_node)

    notify.notify_finish()

