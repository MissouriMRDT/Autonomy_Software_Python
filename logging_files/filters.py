import logging


class NumericalDataFilter(logging.Filter):
    def filter(self, record):
        return (record.levelno == 21)


class DebugFilter(logging.Filter):
    def filter(self, record):
        return (record.levelno == logging.DEBUG)
