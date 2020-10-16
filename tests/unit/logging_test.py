import logging
import core
from core.basestation_send import basestation_send
import time

global packets
packets = []


def test_rovecomm_logging():

    global packets

    logger = logging.getLogger("Logging_Test")
    test_string = """I have successfully added strings into the RoveComm \
interface, and am now sending to send a very long message to see if it is \
going to send properly or not. The limit appears to be 255 characters and \
this string should go over it"""

    # Add incoming packets to the global packets list
    core.rovecomm.set_callback(4241, handle_packet)
    core.rovecomm.set_callback(4242, handle_packet)
    core.rovecomm.set_callback(4243, handle_packet)

    # Run logging events

    logger.debug(test_string)
    assert basestation_send("STATE_CHANGE", "IDLE", "Other information in the message")
    assert basestation_send("WAYPOINT", (12, 27, 3, 4), "Hello")

    # read in RoveComm logs
    i = 0
    while(len(packets) < 6 and i < 50):
        time.sleep(.01)
        i += 1

    packet_data = []
    for packet in packets:
        if packet.data_type == 's':
            # Reconstruct char[] into a string
            data = packet.data
            data_string = ""
            for s in data:
                data_string += s.decode('utf-8')
            packet_data.append(data_string)
        else:
            pass
            packet_data.append(packet.data)

    assert packet_data[0] == """logging_test, test_rovecomm_logging, DEBUG, I have successfully added \
strings into the RoveComm interface, and am now sending to send a very long message \
to see if it is going to send properly or not. The limit appears to be 255 characters \
and this str..."""
    assert packet_data[1] == """logging_test, test_rovecomm_logging, DEBUG, I have successfully added \
strings into the RoveComm interface, and am now sending to send a very long message \
to see if it is going to send properly or not. The limit appears to be 255 characters \
and this str..."""
    assert packet_data[2] == """basestation_send, basestation_send, INFO, STATE_CHANGE - \
IDLE - Other information in the message"""
    assert packet_data[3] == (2,)
    assert packet_data[4] == """basestation_send, basestation_send, INFO, WAYPOINT - \
(12, 27, 3, 4) - Hello"""
    assert packet_data[5] == (12, 27, 3, 4)


def handle_packet(packet):
    global packets
    packets.append(packet)
