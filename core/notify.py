import time
import core

# TODO: Wrap this in a Lighting/Media Board interface
class Notify:

    def __init__(self, rove_comm):
        self.rove_comm_node = rove_comm

    def _notify(self, val):
        self.rove_comm_node.write(core.RoveCommPacket(core.NOTIFY_ID, 'b', (val, 0), ip_octet_4='142'))

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

