from core.rovecomm import RoveCommPacket
from core.rovecomm import RoveCommEthernetTcp
import core
import RoveCommTest
import time
import threading

RoveCommTest.variable = 0


def main() -> None:
    core.rovecomm_node.callbacks[4242] = print_packet
    packet = RoveCommPacket(4242, 'b', (1, 3), "", 11000)
    packet.SetIp('127.0.0.1')
    core.rovecomm_node.write(packet)

    RoveCommTCP = RoveCommEthernetTcp(HOST='127.0.0.1', PORT=11113)
    RoveCommTCP2 = RoveCommEthernetTcp(HOST='127.0.0.1', PORT=11112)
    RoveCommTCP2.callbacks[4242] = print_packet
    packet2 = RoveCommPacket(4242, 'b', (1, 3), "", 11112)
    packet2.SetIp('127.0.0.1')
    RoveCommTCP.write(packet2)

    print(threading.enumerate())
    time.sleep(1)
    print(RoveCommTest.variable)


def print_packet(packet):
    RoveCommTest.variable += 1
