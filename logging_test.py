import logging
# from logging_files.RoveComm_Python import RoveCommEthernetUDP
import core
from core.basestation_send import basestation_send

# Use of the UDP handler will require the 's' data type to be added to core.rovecomm
global packets
packets = []


def main() -> None:
    global packets
    # some output
    logger = logging.getLogger("Logging_Test")
    testString = """I have successfully added strings into the RoveComm \
interface, and am now sending to send a very long message to see if it is \
going to send properly or not. The limit appears to be 255 characters and \
this string should go over it"""

    core.rovecomm.set_callback(4241, handle_packet)
    core.rovecomm.set_callback(4242, handle_packet)
    core.rovecomm.set_callback(4243, handle_packet)

    logger.debug(testString)

    basestation_send("STATE_CHANGE", "IDLE", "Other information in the message")
    basestation_send("WAYPOINT", (12, 27, 3, 4), "Hello")

    # read in RoveComm logs
    print("----------------Sent through RoveComm----------------")
    # for i in range(3):
    #     packets.append(RoveCommUDP.read())

    while(len(packets) < 5):
        pass

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
