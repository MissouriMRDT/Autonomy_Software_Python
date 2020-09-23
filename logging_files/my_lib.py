import logging


def my_function():
    logger = logging.getLogger(__name__)
    testString = """I have successfully added strings into the RoveComm \
interface, and am now sending to send a very long message to see if it is \
going to send properly or not. The limit appears to be 255 characters and \
this string should go over it"""
    logger.info(testString)
    logger.log(21, "STATE_CHANGE - IDLE - Other information in the message")
    logger.log(21, "WAYPOINT - (12, 27, 3, 4)")
