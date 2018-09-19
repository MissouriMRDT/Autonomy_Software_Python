import time

from drivers.rovecomm import RoveComm

DRIVE_BOARD_IP = "192.168.1.130"
NOTIFY_ID = -1

class Notify:

    def __init__(self, rovecomm_node):

        self.rovecomm_node = rovecomm_node

    def notifyFinish(self):
        self.rovecomm_node.sendNotify(NOTIFY_ID, 1)
        time.sleep(2)
        self.rovecomm_node.sendNotify(NOTIFY_ID, 0)
