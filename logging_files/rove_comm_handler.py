from core.rovecomm import RoveCommPacket
import core
import logging


class RoveCommHandler(logging.Handler):

    def __init__(self, target_host, target_port):
        """
        Initializes the handler with given network variables
        """
        logging.Handler.__init__(self)
        self.target_host = target_host
        self.target_port = target_port

    def emit(self, s):
        """
        Encodes and sends the log message over RoveComm
        """
        if core.rovecomm_node is None:
            logging.warning('UDP Handler called with no socket')
            return

        msg = self.format(s)
        # Max string size is 255 characters, truncate the rest and flag it
        if len(msg) > 255:
            msg = msg[:252] + "..."

        packet = RoveCommPacket(
            4242,
            's',
            tuple([char.encode('utf-8') for char in msg]),
            "",
            self.target_port
        )
        packet.SetIp(self.target_host)
        core.rovecomm_node.write(packet)
