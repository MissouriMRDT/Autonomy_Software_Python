from core.rovecomm import RoveCommPacket
import core
import time
import struct
import socket
import sys
from contextlib import contextmanager
from io import StringIO
from unittest.mock import Mock

# Dict of packets, each test will use a different data_id so they don't interfere
global responses
responses = {}


def test_udp():
    global responses
    core.rovecomm_node.set_callback(4242, handle_packet)

    packet = RoveCommPacket(4242, "b", (1, 3), "127.0.0.1", 11000)
    assert core.rovecomm_node.write(packet, False) == 1

    # Give the listener thread a moment to catch the packet
    time.sleep(0.05)
    assert responses[4242].data == packet.data
    assert responses[4242].data_type == packet.data_type
    assert responses[4242].data_count == packet.data_count
    assert responses[4242].data_id == packet.data_id


def test_tcp():
    global responses
    core.rovecomm_node.set_callback(4241, handle_packet)

    # The RoveCommTcp class includes both a server socket and
    # a dictionary of connection sockets, allowing it to create
    # a TCP connection between its server and one of the dictionary
    # sockets
    packet = RoveCommPacket(4241, "b", (1, 4), "127.0.0.1", 11111)
    assert core.rovecomm_node.write(packet, True) == 1

    # Give the listener thread a moment to catch the packet
    time.sleep(0.05)
    assert responses[4241].data == packet.data
    assert responses[4241].data_type == packet.data_type
    assert responses[4241].data_count == packet.data_count
    assert responses[4241].data_id == packet.data_id


def test_tcp_subscribers():
    global responses
    core.rovecomm_node.set_callback(4243, handle_packet)

    # The RoveCommTcp class includes both a server socket and
    # a dictionary of connection sockets, allowing it to create
    # a TCP connection between its server and one of the dictionary
    # sockets
    packet = RoveCommPacket(4243, "b", (1, 4), "127.0.0.1", 11111)

    core.rovecomm_node.tcp_node.connect(("127.0.0.1", 11111))

    assert core.rovecomm_node.write(packet, True) == 1

    # Give the listener thread a moment to catch the packet
    time.sleep(0.05)
    assert responses[4243].data == packet.data
    assert responses[4243].data_type == packet.data_type
    assert responses[4243].data_count == packet.data_count
    assert responses[4243].data_id == packet.data_id


def test_udp_external():
    global responses
    core.rovecomm_node.set_callback(4240, handle_packet)

    # Test socket to try to send to RoveComm over UDP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 11001))

    rovecomm_packet = struct.pack(">BHBB", 2, 4240, 2, 0)
    data = (1, 6)
    for i in data:
        rovecomm_packet = rovecomm_packet + struct.pack(">b", i)

    s.sendto(rovecomm_packet, ("127.0.0.1", 11000))

    # Give the listener thread a moment to catch the packet
    time.sleep(0.05)
    assert responses[4240].data == data
    assert responses[4240].data_type == "b"
    assert responses[4240].data_count == len(data)
    assert responses[4240].data_id == 4240
    s.close()


def test_tcp_external():
    global responses
    core.rovecomm_node.set_callback(4239, handle_packet)

    # Test socket to try to send to RoveComm over TCP
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("127.0.0.1", 11111))
    rovecomm_packet = struct.pack(">BHBB", 2, 4239, 2, 0)
    data = (1, 5)
    for i in data:
        rovecomm_packet = rovecomm_packet + struct.pack(">b", i)

    s.send(rovecomm_packet)

    # Give the listener thread a moment to catch the packet
    time.sleep(0.05)
    assert responses[4239].data == data
    assert responses[4239].data_type == "b"
    assert responses[4239].data_count == len(data)
    assert responses[4239].data_id == 4239


def test_invalid_target():
    # Sends to a port that won't be available
    packet = RoveCommPacket(4238, "b", (0,), "127.0.0.1", 99999)
    assert core.rovecomm_node.write(packet, True) == 0
    assert core.rovecomm_node.write(packet, False) == 0


def test_callback_exception():
    global response
    core.rovecomm_node.set_callback(4237, handle_packet_exception)

    packet = RoveCommPacket(4237, "b", (1, 3), "127.0.0.1", 11000)
    assert core.rovecomm_node.write(packet, False) == 1

    # RoveComm passes on callback exception, nothing else to do


def test_print_packet():
    packet = RoveCommPacket(4236, "b", (1, 4), "127.0.0.1", 11111)
    # Temporarily changes stdout to write into an easily accessible output
    with captured_output() as (out, err):
        packet.print()
        output = out.getvalue().strip()

    assert (
        output
        == """----------
ID:    4236
Type:  b
Count: 2
IP:    ('127.0.0.1', 11111)
Data:  (1, 4)
----------"""
    )


def test_invalid_write_udp():
    # Sends non-tuple data
    packet = RoveCommPacket(4235, "b", "a", "127.0.0.1", 11000)
    assert core.rovecomm_node.write(packet, False) == 0


def test_invalid_write_tcp():
    # Sends non-tuple data
    packet = RoveCommPacket(4235, "b", "a", "127.0.0.1", 11111)
    assert core.rovecomm_node.write(packet, True) == 0


def test_udp_subscribe():
    assert core.rovecomm_node.udp_node.subscribe("0") == 1
    # Couldn't test receipt of packet because it sends to external IP

    global responses
    core.rovecomm_node.set_callback(3, handle_packet)

    packet = RoveCommPacket(3, "b", (), "127.0.0.1", 11000)
    assert core.rovecomm_node.write(packet, False) == 1

    time.sleep(0.05)
    assert responses[3].data_id == packet.data_id
    assert len(core.rovecomm_node.udp_node.subscribers) == 1

    # Main target should be invalid
    core.rovecomm_node.set_callback(4234, handle_packet)
    packet2 = RoveCommPacket(4234, "b", (), "", 0)
    assert core.rovecomm_node.write(packet2, False) == 1

    # Packet should still be recieved because we're subscribed
    time.sleep(0.05)
    assert responses[4234].data == packet2.data
    assert responses[4234].data_type == packet2.data_type
    assert responses[4234].data_count == packet2.data_count
    assert responses[4234].data_id == packet2.data_id


def test_udp_unsubscribe():
    global responses
    core.rovecomm_node.set_callback(4, handle_packet)

    packet = RoveCommPacket(4, "b", (), "127.0.0.1", 11000)
    assert core.rovecomm_node.write(packet, False) == 1

    time.sleep(0.05)
    assert responses[4].data_id == packet.data_id
    assert len(core.rovecomm_node.udp_node.subscribers) == 0

    # Main target should be invalid
    core.rovecomm_node.set_callback(4233, handle_packet)
    packet2 = RoveCommPacket(4233, "b", (), "", 0)
    assert core.rovecomm_node.write(packet2, False) == 1

    # Packet should not still be recieved because we unsubscribed
    time.sleep(0.05)
    assert responses.get(4233) is None


def test_invalid_rovecomm_version_tcp():
    global responses
    # 5 is the data id for the invalid version return packet
    core.rovecomm_node.set_callback(5, handle_packet)

    # Test socket to try to send to RoveComm over TCP
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("127.0.0.1", 11111))
    rovecomm_packet = struct.pack(">BHBB", 1, 4232, 2, 0)
    data = (1, 7)
    for i in data:
        rovecomm_packet = rovecomm_packet + struct.pack(">b", i)

    s.send(rovecomm_packet)

    # Give the listener thread a moment to catch the packet
    time.sleep(0.05)
    assert responses[5].data == (1,)
    assert responses[5].data_type == "b"
    assert responses[5].data_count == 1
    assert responses[5].data_id == 5


def test_invalid_rovecomm_version_udp():
    global responses
    responses.pop(5, None)
    # 5 is the data id for the invalid version return packet
    core.rovecomm_node.set_callback(5, handle_packet)

    # Test socket to try to send to RoveComm over UDP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 11001))

    rovecomm_packet = struct.pack(">BHBB", 1, 4231, 2, 0)
    data = (1, 6)
    for i in data:
        rovecomm_packet = rovecomm_packet + struct.pack(">b", i)

    s.sendto(rovecomm_packet, ("127.0.0.1", 11000))

    # Give the listener thread a moment to catch the packet
    time.sleep(0.05)
    assert responses[5].data == (1,)
    assert responses[5].data_type == "b"
    assert responses[5].data_count == 1
    assert responses[5].data_id == 5
    s.close()


def test_read_exception_udp():
    global responses
    # 0 is the data id for the default packet, sent when exception is raised
    core.rovecomm_node.set_callback(0, handle_packet)

    # A mock socket to throw an exception when a read occurs
    mock = Mock()
    mock.recvfrom.side_effect = Exception
    temp = core.rovecomm_node.udp_node.RoveCommSocket
    mock.fileno.return_value = temp.fileno()
    core.rovecomm_node.udp_node.RoveCommSocket = mock

    # Send a packet so recv can be triggered
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 11001))

    rovecomm_packet = struct.pack(">BHBB", 1, 4230, 2, 0)
    data = (1, 6)
    for i in data:
        rovecomm_packet = rovecomm_packet + struct.pack(">b", i)

    s.sendto(rovecomm_packet, ("127.0.0.1", 11000))

    # Give the listener thread a moment to catch the packet
    time.sleep(0.05)
    # Match the packet to the blank packet
    assert responses[0].data == ()
    assert responses[0].data_type == "b"
    assert responses[0].data_count == 0
    assert responses[0].data_id == 0

    # Gives the udp node its socket back
    core.rovecomm_node.udp_node.RoveCommSocket = temp
    s.close()


def test_listener_shutdown():
    # Store data to recreate the instance afterwards
    callbacks = core.rovecomm_node.callbacks
    udp_port = core.rovecomm_node.udp_node.rove_comm_port
    tcp_addr = core.rovecomm_node.tcp_node.server.getsockname()

    # Run the test
    assert core.rovecomm_node.thread.is_alive()
    core.rovecomm_node.close_thread()
    assert not core.rovecomm_node.thread.is_alive()

    # Reopen the thread to avoid any problems with other tests
    core.rovecomm_node = core.RoveComm(udp_port, tcp_addr)
    core.rovecomm_node.callbacks = callbacks
    assert core.rovecomm_node.thread.is_alive()


def handle_packet(packet):
    global responses
    responses[packet.data_id] = packet


def handle_packet_exception(packet):
    raise Exception


# Redirects sys.stdout to be captured more easily for testing
@contextmanager
def captured_output():
    new_out, new_err = StringIO(), StringIO()
    old_out, old_err = sys.stdout, sys.stderr
    try:
        sys.stdout, sys.stderr = new_out, new_err
        yield sys.stdout, sys.stderr
    finally:
        sys.stdout, sys.stderr = old_out, old_err
