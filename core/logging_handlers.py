from core.rovecomm import RoveCommPacket
import core
import logging
from datetime import datetime
from os import path


class CsvHandler(logging.handlers.WatchedFileHandler):

    def __init__(self, filename, format_string, encoding=None, delay=False, new_file=False):
        """
        Initializes the handler
        """

        if (new_file):
            timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            f = open(f'{filename[:-4]}-{timestamp}{filename[-4:]}', 'w')
            f.write(format_string + '\n')
            f.close()
            logging.handlers.WatchedFileHandler.__init__(self, f'{filename[:-4]}-{timestamp}{filename[-4:]}', 'a', encoding, delay)
        else:
            if (not path.exists(filename)):
                f = open(filename, 'w')
                f.write(format_string + '\n')
                f.close()
            logging.handlers.WatchedFileHandler.__init__(self, filename, 'a', encoding, delay)


class RoveCommHandler(logging.Handler):
    def __init__(self, reliable):
        """
        Initializes the handler with given network variables
        """
        self.reliable = reliable

        logging.Handler.__init__(self)

    def emit(self, s):
        """
        Encodes and sends the log message over RoveComm
        """
        msg = self.format(s)
        # Max string size is 255 characters, truncate the rest and flag it
        if len(msg) > 255:
            msg = msg[:252] + "..."

        packet = RoveCommPacket(
            4241,
            's',
            tuple([char.encode('utf-8') for char in msg]),
            ""
        )
        core.rovecomm.write(packet, self.reliable)
