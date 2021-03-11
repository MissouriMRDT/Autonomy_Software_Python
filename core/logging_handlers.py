from core.rovecomm_module.rovecomm import RoveCommPacket
import core
import logging
from logging.handlers import WatchedFileHandler
from datetime import datetime
import os


class CsvHandler(WatchedFileHandler):
    def __init__(self, filename, header, encoding=None, delay=False, new_file=False):
        """
        Initializes the handler
        """

        if new_file:
            timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            if not os.path.exists("/".join(f"{filename[:-4]}-{timestamp}{filename[-4:]}".split("/")[:-1])):
                os.makedirs("/".join(f"{filename[:-4]}-{timestamp}{filename[-4:]}".split("/")[:-1]))
            f = open(f"{filename[:-4]}-{timestamp}{filename[-4:]}", "w")
            f.write(header + "\n")
            f.close()
            WatchedFileHandler.__init__(
                self,
                f"{filename[:-4]}-{timestamp}{filename[-4:]}",
                "a",
                encoding,
                delay,
            )
        else:
            if not os.path.exists(filename):
                if not os.path.exists("/".join(filename.split("/")[:-1])):
                    os.makedirs("/".join(filename.split("/")[:-1]))
                f = open(filename, "w")
                f.write(header + "\n")
                f.close()
            WatchedFileHandler.__init__(self, filename, "a", encoding, delay)


class RoveCommHandler(logging.Handler):
    def __init__(self, reliable, data_id):
        """
        Initializes the handler with given network variables
        """
        self.reliable = reliable
        self.data_id = data_id

        logging.Handler.__init__(self)

    def emit(self, s):
        """
        Encodes and sends the log message over RoveComm
        """
        msg = self.format(s)
        # Max string size is 255 characters, truncate the rest and flag it
        # 255 coming from uint8
        if len(msg) > 255:
            msg = msg[:252] + "..."

        packet = RoveCommPacket(self.data_id, "c", tuple([char.encode("utf-8") for char in msg]), "", 0)
        core.rovecomm_node.write(packet, self.reliable)
