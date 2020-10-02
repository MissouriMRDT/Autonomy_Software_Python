from core.rovecomm import RoveCommPacket
import core
import rovecomm_test
import time
import threading
import struct
import socket

rovecomm_test.variable = 0


def main() -> None:
    core.rovecomm.set_callback(4242, handlePacket)
    packet = RoveCommPacket(4242, 'b', (1, 3), "", 11000)
    packet.SetIp('127.0.0.1')
    core.rovecomm.write(packet, False)

    packet2 = RoveCommPacket(4242, 'b', (1, 4), "", 11111)
    packet2.SetIp('127.0.0.1')
    core.rovecomm.write(packet2, True)

    # Test socket to try to send to RoveComm
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("127.0.0.1", 11111))
    rovecomm_packet = struct.pack(">BHBB", 2, 4242, 2, 0)
    for i in (1, 5):
        rovecomm_packet = rovecomm_packet + struct.pack('>b', i)

    for i in range(5):
        s.send(rovecomm_packet)
        time.sleep(.5)
        print(rovecomm_test.variable)
    # Closes the RoveComm listener thread, it closes itself when
    #   the program terminates
    # core.rovecomm.close_thread()
    print(threading.enumerate())


def handlePacket(packet):
    packet.print()
    rovecomm_test.variable += 1
