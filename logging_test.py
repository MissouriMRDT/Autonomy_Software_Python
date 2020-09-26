import logging
# from logging_files.RoveComm_Python import RoveCommEthernetUDP
import core
from core.rovecomm_TCP import RoveCommEthernetTCP
from core.rovecomm_caller import rovecomm_send

# Use of the UDP handler will require the 's' data type to be added to core.rovecomm


def main() -> None:
    # set up RoveComm readers
    # RoveCommUDP = RoveCommEthernetUdp(port=10999)
    RoveCommTCP = RoveCommEthernetTCP(HOST='127.0.0.1', PORT=11112)

    # sets up TCP handler
    core.rovecomm_node_tcp = RoveCommEthernetTCP(HOST='127.0.0.1', PORT=11111)

    # some output
    logger = logging.getLogger("Logging_Test")
    testString = """I have successfully added strings into the RoveComm \
interface, and am now sending to send a very long message to see if it is \
going to send properly or not. The limit appears to be 255 characters and \
this string should go over it"""
    logger.debug(testString)
    rovecomm_send("STATE_CHANGE", "IDLE", "Other information in the message")
    rovecomm_send("WAYPOINT", (12, 27, 3, 4), "Hello")

    # read in RoveComm logs
    print("----------------Sent through RoveComm----------------")
    packets = []
    # for i in range(3):
    #     packets.append(RoveCommUDP.read())
    for i in range(5):
        packets.extend(RoveCommTCP.read())

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

    # clean up
    RoveCommTCP.close_sockets()
    core.rovecomm_node_tcp.close_sockets()
    # RoveCommUDP.RoveCommSocket.close()
