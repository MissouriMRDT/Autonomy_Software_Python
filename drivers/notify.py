import time

from drivers.rovecomm import RoveComm

DRIVE_BOARD_IP = "192.168.1.130"
NOTIFY_ID = -1

class Notify:

    def __init__(self, rovecomm_node):

        self.rovecomm_node = rovecomm_node

    def notifyFinish():
        self.rovecomm_node.sendTo(NOTIFY_ID, 1, DRIVE_BOARD_IP)
        time.sleep(10)
        self.rovecomm_node.sendTo(NOTIFY_ID, 0, DRIVE_BOARD_IP)
