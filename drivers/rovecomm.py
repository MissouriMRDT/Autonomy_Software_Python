import socket
import struct
import threading

PORT = 11000

class RoveComm(object):
    header_format = ">BHBHH"
    version = 1

    def __init__(self):
        self.callbacks = {}

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(("127.0.0.1", RoveComm.port))
        except:
            print "Error: Could not claim port." \
                  " You can only have one RoveComm listener at a time " \
                  " Are you sure you didn't accidentally make a copy? "
            raise Exception()

        #start a thread / async thingy
        self.monitoring_thread = threading.Thread(target=self._listen_thread)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

    def send(self, data, data_id, dest_ip, PORT, seq_num=0x0F49, flags=0x00):
        """ Send a RoveComm formatted message

        Parameters
        -----------
            dataID: Short unsigned integer
            data: bytearray or bytestring
            dest_ip: IP address to send to. String formatted like "192.168.1.1"
            port: Port number. 2 byte unsigned integer
            seqNum: Short unsigned integer. Not used in current revision of RoveComm
            flags: Unused in current version of RoveComm
        """

        packet_size = len(data)
        header = struct.pack(self.header_format,
                             RoveComm.version,
                             seq_num,
                             flags,
                             data_id,
                             packet_size)
        msgbuffer = header + data
        # print "Message Buffer: ", binascii.hexlify(msgbuffer), "\n"
        self.socket.sendto(msgbuffer, (dest_ip, port))

    def _listen_thread(self):
        while True:
            packet, addr = self.socket.recvfrom(1024)
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
