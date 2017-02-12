import socket
import struct
import threading
import logging

PORT          = 11000
VERSION       = 1
HEADER_FORMAT = ">BHBHH"

ACK_FLAG      = 0x01
INTERNAL_DATA_IDS = {
    "Ping"             : 1,
    "Ping Reply"       : 2,
    "Subscribe"        : 3,
    "Unsubcribe"       : 4,
    "Force Unsubcribe" : 5,
    "ACK"              : 6,
}


class RoveComm(object):
    """
    RoveComm message sender and receiver

    Example:

        from rovecomm import RoveComm
        import struct
        import time
        import random

        def set_speed_handler(contents):
            # Contents are typically C style structs
            # ">HH" is a format code for two uint16_t
            # See python's documentation for struct.unpack
            # for more info about format codes
            # And working with C style structs from python
            speed_left, speed_right = struct.unpack(">HH", contents)
            print "Speed set to %d, %d" % (speed_left, speed_right)

        def add_waypoint_handler(contents):
            latitude, longitude = struct.unpack(">dd")
            print "Added waypoint (%f, %f) " % (latitude, longitude)

        rovecomm_node = RoveComm()

        # use RoveComm.callbacks to define what code should
        # run when a message is received.
        # Here we assign data id 138 to set_speed_handler
        # and data id 267 to add_waypoint_handler

        rovecomm_node.callbacks[138] = set_speed_handler
        rovecomm_node.callbacks[267] = add_waypoint_handler

        # Now you can do the rest of your program
        while True:
            my_telemetry = random.random()
            rovecomm_node.send(data_id = 138,
                               data = struct.pack(my_telemetry, 'f'),
                               destination_ip = "192.168.1.20")
            time.sleep(5)
    """

    def __init__(self):
        self.callbacks = {}

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.bind(("127.0.0.1", PORT))
        except socket.error:
            raise Exception("Error: Could not claim port. "
                            "Either another program or another copy or rovecomm"
                            "is using port %d " % (PORT,))

        # start a thread to get messages in the background
        self._monitoring_thread = threading.Thread(target=self._listen_thread)
        self._monitoring_thread.daemon = True
        self._monitoring_thread.start()

    def send(self, data_id, data, destination_ip, seq_num=0x0F49, flags=0x00, port=PORT):
        """
        Send a RoveComm formatted message

        Parameters
        -----------
            data_id        : RoveComm Data ID. 16-bit integer.
            data           : bytes to send
            destination_ip : IP address to send to. String formatted like "192.168.1.1"

            The following parameters are optional and not commonly used:

            seq_num        : 16 bit sequence number. Not commonly used.
            port           : Port number. 2 byte unsigned integer
            flags          : Bit addressable field. OR all ack flags together
        """

        packet_size = len(data)
        header = struct.pack(self.header_format,
                             RoveComm.version,
                             seq_num,
                             flags,
                             data_id,
                             packet_size)
        msgbuffer = header + data
        self._socket.sendto(msgbuffer, (destination_ip, port))

    def _listen_thread(self):
        while True:
            packet, addr = self._socket.recvfrom(1024)
            # TODO: Upgrade to logging
            print "DEBUG: Packet received: ", packet

            data_id, content_bytes = self._parse_header(packet)
            try:
                self.callbacks[data_id](packet)
            except KeyError:
                print "Warning: no callback assigned for data id %d", data_id

    def _parse_header(self, packet):
        header_bytes = packet[0:struct.calcsize(self.header_format)]
        content_bytes = packet[struct.calcsize(self.header_format):]

        (header_format, version, seq_num, flags, data_id, size) = struct.unpack(self.header_format, header_bytes)
        return data_id, content_bytes

    def _ping_reply(self, contents):
        