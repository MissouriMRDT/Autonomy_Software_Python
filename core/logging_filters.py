import logging


class DebugFilter(logging.Filter):
    def filter(self, record):
        return (record.levelno == logging.DEBUG)
