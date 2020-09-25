import logging
from datetime import datetime


class CsvHandler(logging.handlers.WatchedFileHandler):

    def __init__(self, filename, format_string, encoding=None, delay=False):
        """
        Initializes the handler
        """
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        f = open(f'{filename[:-4]}-{timestamp}{filename[-4:]}', 'w')
        f.write(format_string + '\n')
        f.close()

        logging.handlers.WatchedFileHandler.__init__(self, f'{filename[:-4]}-{timestamp}{filename[-4:]}', 'a', encoding, delay)
