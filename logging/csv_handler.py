import logging
import os


class CsvHandler(logging.handlers.WatchedFileHandler):

    def __init__(self, filename, format_string, encoding=None, delay=False):
        """
        Initializes the handler
        """
        num = 0
        while os.path.exists(f'{filename[:-4]}-{num}{filename[-4:]}'):
            num += 1
        f = open(f'{filename[:-4]}-{num}{filename[-4:]}', 'w')
        f.write(format_string + '\n')
        f.close()

        logging.handlers.WatchedFileHandler.__init__(self, f'{filename[:-4]}-{num}{filename[-4:]}', 'a', encoding, delay)
