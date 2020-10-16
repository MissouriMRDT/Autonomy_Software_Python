import logging
import core
from core.basestation_send import basestation_send

global packets
packets = []


def main() -> None:
    global packets

    logger = logging.getLogger("Logging_Test")
    testString = """I have successfully added strings into the RoveComm \
interface, and am now sending to send a very long message to see if it is \
going to send properly or not. The limit appears to be 255 characters and \
this string should go over it"""

    # Add incoming packets to the global packets list
    core.rovecomm.set_callback(4241, handle_packet)
    core.rovecomm.set_callback(4242, handle_packet)
    core.rovecomm.set_callback(4243, handle_packet)

    # Run logging events
    logger.debug(testString)
    basestation_send("STATE_CHANGE", "IDLE", "Other information in the message")
    basestation_send("WAYPOINT", (12, 27, 3, 4), "Hello")

    # read in RoveComm logs
    while(len(packets) < 5):
        pass

    print("----------------Sent through RoveComm----------------")

    for packet in packets:
        if packet.data_type == 's':
            # Reconstruct char[] into a string
            data = packet.data
            data_string = ""
            for s in data:
                data_string += s.decode('utf-8')
            print(data_string)
        else:
            packet.print()


def handle_packet(packet):
    global packets
    packets.append(packet)
