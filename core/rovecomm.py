import socket
import struct
import threading
import core
import select
import logging

ROVECOMM_PORT           = 11000
ROVECOMM_VERSION        = 2
ROVECOMM_HEADER_FORMAT  = ">BHBB"

ROVECOMM_PING_REQUEST           = 1
ROVECOMM_PING_REPLY             = 2
ROVECOMM_SUBSCRIBE_REQUEST      = 3
ROVECOMM_UNSUBSCRIBE_REQUEST    = 4
ROVECOMM_INCOMPATIBLE_VERSION   = 5

types_int_to_byte = {
    0: 'b',
    1: 'B',
    2: 'h',
    3: 'H',
    4: 'l',
    5: 'L',
    6: 'q',  # int64, this needs to stay here to not break RED. Leave this comment here Skelton.
    7: 'd',  # double
}

types_byte_to_int = {
    'b': 0,
    'B': 1,
    'h': 2,
    'H': 3,
    'l': 4,
    'L': 5,
    'q': 6,  # int64
    'd': 7,  # double
}

types_byte_to_size = {
    'b': 1,
    'B': 1,
    'h': 2,
    'H': 2,
    'l': 4,
    'L': 4,
    'q': 8,
    'd': 8,
}


class RoveCommPacket:
    '''
    The RoveComm packet is the encapsulation of a message sent across the rover
    network that can be parsed by all rover computing systems.

    A RoveComm Packet contains:
        - A data id
        - A data type
        - The number of data entries (data count)
        - The data itself

    The autonomy implementation also includes the remote ip of the sender.
    '''
    def __init__(self, data_id=0, data_type='b', data=(), ip_octet_4='', port=ROVECOMM_PORT):
        self.data_id = data_id
        self.data_type = data_type
        self.data_count = len(data)
        self.data = data
        if ip_octet_4 != '':
            self.ip_address = ('192.168.1.' + ip_octet_4, port)
        else:
            self.ip_address = ('0.0.0.0', port)
        return

    def SetIp(self, address):
        self.ip_address = (address, self.ip_address[1])

    def print(self):
        print('----------')
        print('{0:6s} {1}'.format('ID:', self.data_id))
        print('{0:6s} {1}'.format('Type:', self.data_type))
        print('{0:6s} {1}'.format('Count:', self.data_count))
        print('{0:6s} {1}'.format('IP:', self.ip_address))
        print('{0:6s} {1}'.format('Data:', self.data))
        print('----------')


class RoveCommCollection:
    '''
    Creates a separate thread to read all RoveComm connections
    '''
    connections = []
    shutdown_event = None

    def __init__(self):
        self.shutdown_event = threading.Event()
        self.thread = threading.Thread(target=self.listen)
        self.thread.start()

    def listen(self):
        while threading.main_thread().is_alive() and not self.shutdown_event.isSet():
            for connection in self.connections:
                result = connection.read()
                if not isinstance(result, list):
                    result = [result, ]
                for packet in result:
                    if packet is not None and packet.data_id != 0:
                        try:
                            connection.callbacks[packet.data_id](packet)
                        except Exception:
                            pass
        for connection in self.connections:
            if isinstance(connection, RoveCommEthernetTcp):
                connection.close_sockets()

    def close_thread(self):
        self.shutdown_event.set()
        self.thread.join()


class RoveCommEthernetUdp:
    '''
    The UDP implementation for RoveComm. UDP is a fast connectionless transport
    protocol that guarantees no data corruption but does not guarantee delivery,
    and if it delivers does not guarantee it being in the same order it was
    sent.

    Implements:
        - Write (to both the target ip and all subscribers)
        - Read
    '''
    def __init__(self, port=ROVECOMM_PORT):
        self.rove_comm_port = port
        self.subscribers = []

        self.callbacks = {}

        self.RoveCommSocket = socket.socket(type=socket.SOCK_DGRAM)
        self.RoveCommSocket.setblocking(True)
        self.RoveCommSocket.bind(("", self.rove_comm_port))

        if core.rovecomm.collection is None:
            core.rovecomm.collection = RoveCommCollection()
        core.rovecomm.collection.connections.append(self)

    def subscribe(self, ip_octet):

        self.write(RoveCommPacket(data_id=3, ip_octet_4=ip_octet))

    def write(self, packet):
        '''
        Transmits a packet to the destination IP (if there is one) and all active
        subscribers.

        Parameters:
            packet (RoveCommPacket): A packet containing the data and header info
            to be transmitted over the rover network
        Returns:
            success (int): An integer, either 0 or 1 depending on whether or not
            an exception occured during writing
        '''
        try:
            if not isinstance(packet.data, tuple):
                raise ValueError('Must pass data as a list, Data: ' + str(packet.data))

            rovecomm_packet = struct.pack(ROVECOMM_HEADER_FORMAT, ROVECOMM_VERSION, packet.data_id, packet.data_count,
                                          types_byte_to_int[packet.data_type])
            for i in packet.data:
                rovecomm_packet = rovecomm_packet + struct.pack('>' + packet.data_type, i)

            for subscriber in self.subscribers:
                self.RoveCommSocket.sendto(rovecomm_packet, subscriber)

            if packet.ip_address != ('0.0.0.0', 0):
                self.RoveCommSocket.sendto(rovecomm_packet, packet.ip_address)
                return 1
        except Exception:
            return 0

    def read(self):
        '''
        Unpacks the UDP packet and packs it into a RoveComm Packet for easy
        parsing in other code.

        Returns:
            return_packet (RoveCommPacket): A RoveCommPacket that contains a
            RoveComm message received over the network
        '''
        ready = select.select([self.RoveCommSocket], [], [], 0)
        if ready[0]:
            try:
                packet, remote_ip = self.RoveCommSocket.recvfrom(1024)
                header_size = struct.calcsize(ROVECOMM_HEADER_FORMAT)

                rovecomm_version, data_id, data_count, data_type = struct.unpack(
                    ROVECOMM_HEADER_FORMAT, packet[0:header_size])
                data = packet[header_size:]

                if rovecomm_version != 2:
                    return_packet = RoveCommPacket(ROVECOMM_INCOMPATIBLE_VERSION, 'b', (1,), '')
                    return_packet.ip_address = remote_ip
                    return return_packet

                if data_id == ROVECOMM_SUBSCRIBE_REQUEST:
                    if self.subscribers.count(remote_ip == 0):
                        self.subscribers.append(remote_ip)
                elif data_id == ROVECOMM_UNSUBSCRIBE_REQUEST:
                    if self.subscribers.count(remote_ip) != 0:
                        self.subscribers.remove(remote_ip)

                data_type = types_int_to_byte[data_type]
                data = struct.unpack('>' + data_type * data_count, data)

                return_packet = RoveCommPacket(data_id, data_type, data, '')
                return_packet.ip_address = remote_ip
                return return_packet

            except Exception:
                return_packet = RoveCommPacket()
                return return_packet


class RoveCommEthernetTcp:
    '''
    The TCP implementation for RoveComm.

    Implements:
        - Write
        - Read
    '''
    def __init__(self, HOST=socket.gethostbyname(socket.gethostname()), PORT=11111):
        self.callbacks = {}
        self.open_sockets = {}
        # create a semaphore to ensure we don't iterate through our socket dictionary while simulatenously modifying it
        self.sem = threading.Semaphore()
        # configure a TCP socket
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # bind the socket to the current machines local network IP by default (can be specified as well)
        self.server.bind((HOST, PORT))
        # accept up to 5 simulataneous connections, before we start discarding them
        self.server.listen(5)

        if core.rovecomm.collection is None:
            core.rovecomm.collection = RoveCommCollection()
        core.rovecomm.collection.connections.append(self)

    def close_sockets(self):
        self.sem.acquire()
        for open_socket in self.open_sockets:
            # notifies other end that we are terminating the connection
            self.open_sockets[open_socket].shutdown(1)
            self.open_sockets[open_socket].close()
        self.sem.release()

    def write(self, packet):
        try:
            if not isinstance(packet.data, tuple):
                raise ValueError('Must pass data as a tuple, Data: ' + str(packet.data))

            rovecomm_packet = struct.pack(
                ROVECOMM_HEADER_FORMAT,
                ROVECOMM_VERSION,
                packet.data_id,
                packet.data_count,
                types_byte_to_int[packet.data_type])
            for i in packet.data:
                rovecomm_packet = rovecomm_packet + struct.pack('>' + packet.data_type, i)

            # establish a new connection if the destination has not yet been connected to yet
            self.connect(packet.ip_address)

            if (packet.ip_address != ('0.0.0.0', 0)):
                self.open_sockets[packet.ip_address].send(rovecomm_packet)
                return 1
        except Exception:
            return 0

    def connect(self, address):
        self.sem.acquire()
        if address not in self.open_sockets:
            TCPSocket = socket.socket(type=socket.SOCK_STREAM)
            try:
                TCPSocket.connect(address)
            except Exception as e:
                logging.error("Something's wrong. Exception is %s" % (e))
            self.open_sockets[address] = TCPSocket
        self.sem.release()

    def read(self):
        self.sem.acquire()

        packets = []

        available_sockets = [self.server]
        for key, value in self.open_sockets.items():
            available_sockets.append(value)

        available_sockets = select.select(available_sockets, [], [], 0)

        for open_socket in available_sockets[0]:
            if open_socket is self.server:
                conn, addr = self.server.accept()
                self.open_sockets[addr[0]] = conn
            else:
                try:
                    header = open_socket.recv(5)
                    rovecomm_version, data_id, data_count, data_type = struct.unpack(ROVECOMM_HEADER_FORMAT, header)
                    data_type_byte = types_int_to_byte[data_type]
                    data = open_socket.recv(data_count * types_byte_to_size[data_type_byte])

                    if(rovecomm_version != 2):
                        returnPacket = RoveCommPacket(ROVECOMM_INCOMPATIBLE_VERSION, 'b', (1,), '')
                        returnPacket.SetIp(open_socket.getpeername())
                        return returnPacket

                    data_type = types_int_to_byte[data_type]
                    data = struct.unpack('>' + data_type * data_count, data)

                    returnPacket = RoveCommPacket(data_id, data_type, data, '')
                    returnPacket.SetIp(open_socket.getpeername())
                    packets.append(returnPacket)
                except Exception:
                    returnPacket = RoveCommPacket()
                    packets.append(returnPacket)

        self.sem.release()
        return packets


collection = None
