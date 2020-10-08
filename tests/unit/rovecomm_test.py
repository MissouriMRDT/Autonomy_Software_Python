from core.rovecomm import RoveCommPacket
import core
import time
import struct
import socket

global response


def test_udp():
    global response
    core.rovecomm.set_callback(4242, handlePacket)

    packet = RoveCommPacket(4242, 'b', (1, 3), "", 11000)
    packet.SetIp('127.0.0.1')
    assert core.rovecomm.write(packet, False) == 1

    # Give the listener thread a moment to catch the packet
    time.sleep(.05)
    assert response.data == packet.data
    assert response.data_type == packet.data_type
    assert response.data_count == packet.data_count
    assert response.data_id == packet.data_id


def test_tcp():
    global response
    core.rovecomm.set_callback(4241, handlePacket)

    # The RoveCommTcp class includes both a server socket and
    # a dictionary of connection sockets, allowing it to create
    # a TCP connection between its server and one of the dictionary
    # sockets
    packet = RoveCommPacket(4241, 'b', (1, 4), "", 11111)
    packet.SetIp('127.0.0.1')
    assert core.rovecomm.write(packet, True) == 1

    # Give the listener thread a moment to catch the packet
    time.sleep(.05)
    assert response.data == packet.data
    assert response.data_type == packet.data_type
    assert response.data_count == packet.data_count
    assert response.data_id == packet.data_id


def test_udp_external():
    global response
    core.rovecomm.set_callback(4240, handlePacket)

    # Test socket to try to send to RoveComm over UDP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 11001))

    rovecomm_packet = struct.pack(">BHBB", 2, 4240, 2, 0)
    data = (1, 6)
    for i in data:
        rovecomm_packet = rovecomm_packet + struct.pack('>b', i)

    s.sendto(rovecomm_packet, ('127.0.0.1', 11000))

    # Give the listener thread a moment to catch the packet
    time.sleep(.05)
    assert response.data == data
    assert response.data_type == 'b'
    assert response.data_count == len(data)
    assert response.data_id == 4240


def test_tcp_external():
    global response
    core.rovecomm.set_callback(4239, handlePacket)

    # Test socket to try to send to RoveComm over TCP
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("127.0.0.1", 11111))
    rovecomm_packet = struct.pack(">BHBB", 2, 4239, 2, 0)
    data = (1, 5)
    for i in data:
        rovecomm_packet = rovecomm_packet + struct.pack('>b', i)

    s.send(rovecomm_packet)

    # Give the listener thread a moment to catch the packet
    time.sleep(.05)
    assert response.data == data
    assert response.data_type == 'b'
    assert response.data_count == len(data)
    assert response.data_id == 4239


def test_invalid_target():
    # Sends to a port that won't be available
    packet = RoveCommPacket(4242, 'b', (0, ), "", 99999)
    packet.SetIp('127.0.0.1')
    assert core.rovecomm.write(packet, True) == 0
    assert core.rovecomm.write(packet, False) == 0


def test_listener_shutdown():
    assert core.rovecomm.thread.is_alive()
    core.rovecomm.close_thread()
    assert not core.rovecomm.thread.is_alive()


def handlePacket(packet):
    global response
    response = packet
