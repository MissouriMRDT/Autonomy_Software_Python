import struct
import threading

PORT = 11000
VERSION = 1


class RoveCommSocket(object):
    def __init__(self):
        self.callbacks = {}

        #start a thread / async thingy
        self.monitoring_thread = threading.Thread(target=self._listen_thread, args=self)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

    def _listen_thread(self):
        while True:
            packet = self.socket.get_packet()

            header = parse_header(packet)

            self.callbacks[data_type](packet)


class RoveCommMessage(object):
    header_format = ">BHBHH"

    def __init__(self, data_id, format_code):
        self.data_id = data_id
        self.format_code = format_code

    def send(self, data, dest_ip, port=PORT, seq_num=0x0F49, flags=0x00):
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

        packetsize = len(data)
        header = struct.pack(self.header_format,
                             VERSION,
                             seq_num,
                             flags,
                             self.data_id,
                             packetsize)
        msgbuffer = header + data
        # print "Message Buffer: ", binascii.hexlify(msgbuffer), "\n"
        sock.sendto(msgbuffer, (dest_ip, port))

    def parse(self, contents):
        header_bytes = contents[0:struct.calcsize(self.header_format)]
        content_bytes = contents[struct.calcsize(self.header_format):]

        header = struct.unpack(self.header_format, header_bytes)
        content = struct.unpack(self.format, content_bytes)
        data_id = header[3]

