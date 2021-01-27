import logging
import core
from core.telemetry_handler import telemetry_handler
import time
from core.rovecomm import RoveCommPacket
from core.rovecomm import ROVECOMM_SUBSCRIBE_REQUEST

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
    core.rovecomm_node.set_callback(4241, handle_packet)
    core.rovecomm_node.set_callback(4242, handle_packet)
    core.rovecomm_node.set_callback(4243, handle_packet)

    # Subscribe to our own packets
    packet = RoveCommPacket(ROVECOMM_SUBSCRIBE_REQUEST, "b", (), "127.0.0.1", 11000)
    core.rovecomm_node.write(packet, False)

    core.rovecomm_node.tcp_node.connect(("127.0.0.1", 11111))

    # Give some time for the subscription to go through
    time.sleep(0.01)

    # Run logging events

    logger.info(test_string)
    assert telemetry_handler("STATE_CHANGE", "IDLE", "Other information in the message")
    assert telemetry_handler("WAYPOINT", (12, 27, 3, 4), "Hello")

    # read in RoveComm logs
    i = 0
    while len(packets) < 6 and i < 50:
        time.sleep(0.01)
        i += 1

    packet_data = []
    for packet in packets:
        if packet.data_type == "c":
            # Reconstruct char[] into a string
            data = packet.data
            data_string = ""
            for s in data:
                data_string += s.decode("utf-8")
            packet_data.append(data_string)
        else:
            pass
            packet_data.append(packet.data)

    # Connecting to ourself creates two different sockets that are both considered subscribed,
    # so we need to remove the duplicate packets that are received in that manner
    packet_data = list(dict.fromkeys(packet_data))

    assert (
        packet_data[0]
        == """Logging_Test, test_rovecomm_logging, INFO, I have successfully added \
strings into the RoveComm interface, and am now sending to send a very long message \
to see if it is going to send properly or not. The limit appears to be 255 characters \
and this stri..."""
    )
    assert packet_data[1] == (2,)
    assert packet_data[2] == (12, 27, 3, 4)


def handle_packet(packet):
    global packets
    packets.append(packet)
