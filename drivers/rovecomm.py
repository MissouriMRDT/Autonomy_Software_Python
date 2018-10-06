import socket
import struct
import threading
import logging

PORT = 11000
VERSION = 1
HEADER_FORMAT = ">BHBHH"

# Flag bit fields
ACK_FLAG = 0x01

# Special Data IDs
PING = 1
PING_REPLY = 2
SUBSCRIBE = 3
UNSUBSCRIBE = 4
FORCE_UNSUBSCRIBE = 5
ACK = 6


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
                               contents = struct.pack(my_telemetry, 'f'),
                               destination_ip = "192.168.1.20")
            time.sleep(5)
    """

    def __init__(self):
        self.callbacks = {}
        self.subscribers = []

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.bind(("", PORT))
        except socket.error:
            raise Exception("Error: Could not claim port. "
                            "Either another program or another copy or rovecomm"
                            "is using port %d " % PORT)

        # start a thread to get messages in the background
        self._monitoring_thread = threading.Thread(target=self._listen_thread)
        self._monitoring_thread.daemon = True
        self._monitoring_thread.start()

    def send(self, data_id, contents):
        """
        Send a RoveComm formatted message to everyone subscribed
        Parameters
        -----------
            data_id        : RoveComm Data ID. 16-bit integer.
            contents       : bytes to send
        """
        for subscriber in self.subscribers:
            self.send_to(data_id, contents, subscriber)

    def subscribe(self, destination_ip):
        """
        Ask to receive messages from another device
        """
        self.send_to(SUBSCRIBE, "", destination_ip)

    def unsubscribe(self, destination_ip):
        """
        Stop receiving messages from another device
        """
        self.send_to(UNSUBSCRIBE, "", destination_ip)

    def send_to(self, data_id, contents, destination_ip, seq_num=0x0F49, flags=0x00, port=PORT):
        """
        Send a RoveComm formatted message to a specific address
        Used internally
        Parameters
        -----------
            data_id        : RoveComm Data ID. 16-bit integer.
            contents           : bytes to send
            destination_ip : IP address to send to. String formatted like "192.168.1.1"
            The following parameters are optional and not commonly used:
            seq_num        : 16 bit sequence number. Not commonly used.
            port           : Port number. 2 byte unsigned integer
            flags          : Bit addressable field. OR all ack flags together
        """
        
        packet_size = len(contents)
        header = self._header(data_id, packet_size, seq_num, flags)
        data = header + contents
        self._send_to_bytes(data, destination_ip, port)

    @staticmethod
    def _header(data_id, packet_size, seq_num=0x0F49, flags=0x00):
        return struct.pack(HEADER_FORMAT,
                           VERSION,
                           seq_num,
                           flags,
                           data_id,
                           packet_size)

    def _send_to_bytes(self, data, destination_ip, port=PORT):
        self._socket.sendto(data, (destination_ip, port))

    def _listen_thread(self):
        while True:
            packet, sender = self._socket.recvfrom(1024)
            # logging.debug("Packet received: %s" % packet)

            # Parse the message header
            header_length = struct.calcsize(HEADER_FORMAT)
            header_bytes = packet[0:header_length]
            content_bytes = packet[header_length:]
            (version, seq_num, flags, data_id, size) = struct.unpack(HEADER_FORMAT, header_bytes)

            # Check for special features, like pings and acknowledgements
            if flags & ACK_FLAG:
                self.send_to(ACK, contents=data_id, destination_ip=sender)

            if data_id == PING:
                self.send_to(PING_REPLY, contents=struct.pack(">H", seq_num), destination_ip=sender)
            elif data_id == SUBSCRIBE:
                self.subscribers.append(sender[0])
            elif data_id == UNSUBSCRIBE:
                try:
                    self.subscribers.remove(sender)
                except ValueError:
                    logging.warning("%s tried to unsubscribe, but wasn't subscribed in the first place" % sender)

            # No special features needed? Good. Just a normal packet.
            else:
                try:
                    self.callbacks[data_id](content_bytes)
                except KeyError:
                    pass
                    # logging.debug("No callback assigned for data id %d" % data_id)
