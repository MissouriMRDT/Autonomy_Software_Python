import socket
import struct
import threading
import select
import logging

ROVECOMM_UDP_PORT = 11000
ROVECOMM_TCP_PORT = 11111
ROVECOMM_VERSION = 2
ROVECOMM_HEADER_FORMAT = ">BHBB"

ROVECOMM_PING_REQUEST = 1
ROVECOMM_PING_REPLY = 2
ROVECOMM_SUBSCRIBE_REQUEST = 3
ROVECOMM_UNSUBSCRIBE_REQUEST = 4
ROVECOMM_INCOMPATIBLE_VERSION = 5

types_int_to_byte = {
    0: "b",
    1: "B",
    2: "h",
    3: "H",
    4: "l",
    5: "L",
    6: "f",
    7: "q",  # int64, this needs to stay here to not break RED. Leave this comment here Skelton.
    8: "d",  # double
    9: "c",
}

types_byte_to_int = {
    "b": 0,
    "B": 1,
    "h": 2,
    "H": 3,
    "l": 4,
    "L": 5,
    "f": 6,
    "q": 7,  # int64
    "d": 8,  # double
    "c": 9,  # char
}

types_byte_to_size = {
    "b": 1,
    "B": 1,
    "h": 2,
    "H": 2,
    "l": 4,
    "L": 4,
    "f": 4,
    "q": 8,
    "d": 8,
    "c": 1,
}


class RoveCommPacket:
    """
    The RoveComm packet is the encapsulation of a message sent across the rover
    network that can be parsed by all rover computing systems.

    A RoveComm Packet contains:
        - A data id
        - A data type
        - The number of data entries (data count)
        - The data itself

    The autonomy implementation also includes the remote ip of the sender.

    Methods:
        SetIp(ip, port):
            Sets packet's IP to address parameter
        print():
            Prints the packet'c contents
    """

    def __init__(
        self, data_id=0, data_type="b", data=(), ip_octet_4="", port=ROVECOMM_UDP_PORT
    ):
        self.data_id = data_id
        self.data_type = data_type
        self.data_count = len(data)
        self.data = data
        if ip_octet_4 != "":
            self.ip_address = ("192.168.1." + ip_octet_4, port)
        else:
            self.ip_address = ("0.0.0.0", port)
        return

    def SetIp(self, ip, port=None):
        """
        Sets packet's IP to address parameter

        Parameters:
            ip (String)
            port (Integer)
        """
        if port is None:
            port = self.ip_address[1]
        self.ip_address = (ip, port)

    def print(self):
        """
        Format:
            ----------
            ID:    _
            Type:  _
            Count: _
            IP:    _
            Data:  _
            ----------
        """
        print("----------")
        print("{0:6s} {1}".format("ID:", self.data_id))
        print("{0:6s} {1}".format("Type:", self.data_type))
        print("{0:6s} {1}".format("Count:", self.data_count))
        print("{0:6s} {1}".format("IP:", self.ip_address))
        print("{0:6s} {1}".format("Data:", self.data))
        print("----------")


class RoveComm:
    """
    Creates a separate thread to read all RoveComm connections

    Methods:
        write(packet, reliable):
            Writes the given packet to its destination address
        set_callback(data_id, func):
            Sets the callback function for any incoming packets with the given data id
        close_thread():
            Shuts down the listener thread
    """

    def __init__(
        self,
        udp_port=ROVECOMM_UDP_PORT,
        tcp_addr=(socket.gethostbyname(socket.gethostname()), ROVECOMM_TCP_PORT),
    ):
        self.callbacks = {}

        self.udp_node = RoveCommEthernetUdp(udp_port)
        self.tcp_node = RoveCommEthernetTcp(*tcp_addr)

        self.shutdown_event = threading.Event()
        self.thread = threading.Thread(target=self.listen)
        self.thread.start()

    def listen(self):
        """
        Loops over RoveComm connections to read packets and execute callbacks;
        closes when main thread closes or close_thread() is called
        """
        while threading.main_thread().is_alive() and not self.shutdown_event.isSet():
            self.tcp_node.handle_incoming_connection()
            packets = self.tcp_node.read()
            packets.append(self.udp_node.read())

            for packet in packets:
                if packet is not None:
                    try:
                        self.callbacks[packet.data_id](packet)
                    except Exception:
                        pass

        self.udp_node.close_socket()
        self.tcp_node.close_sockets()
        # Logger throws an error when logging to console with main thread closed
        if threading.main_thread().is_alive():
            logging.getLogger(__name__).debug("Rovecomm sockets closed")
        return

    def set_callback(self, data_id, func):
        """
        Sets the callback function for any incoming packets with the given data id

        Parameters:
            data_id (Integer): Data id to call the function for
            func (Function): The function to be called
        """
        self.callbacks[data_id] = func

    def write(self, packet, reliable=False):
        """
        Writes the given packet to its destination address

        Parameters:
            packet (RoveCommPacket): The packet to send
            reliable (Bool): Whether to send over TCP or UDP

        Returns:
            success (int): An integer, either 0 or 1 depending on whether or not
            an exception occured during writing
        """
        if reliable:
            return self.tcp_node.write(packet)
        else:
            return self.udp_node.write(packet)

    def close_thread(self):
        """
        Shuts down the listener thread
        """
        self.shutdown_event.set()
        self.thread.join()


class RoveCommEthernetUdp:
    """
    The UDP implementation for RoveComm. UDP is a fast connectionless transport
    protocol that guarantees no data corruption but does not guarantee delivery,
    and if it delivers does not guarantee it being in the same order it was
    sent.

    Methods:
        write(packet):
            Transmits a packet to the destination IP and all active subscribers.
        read():
            Unpacks the UDP packet and packs it into a RoveComm Packet for easy
            parsing in other code.
        subscribe(ip_octet):
            Subscribes to UDP packets from the given ip
        close_socket():
            Closes the UDP socket
    """

    def __init__(self, port=ROVECOMM_UDP_PORT):
        self.rove_comm_port = port
        self.subscribers = []

        self.RoveCommSocket = socket.socket(type=socket.SOCK_DGRAM)
        self.RoveCommSocket.setblocking(True)
        self.RoveCommSocket.bind(("", self.rove_comm_port))

    def subscribe(self, ip_octet):
        """
        Parameters:
            ip_octet (String): The ip to subscribe to
        Returns:
            success (int): An integer, either 0 or 1 depending on whether or not
            an exception occured during writing
        """
        return self.write(RoveCommPacket(data_id=3, ip_octet_4=ip_octet))

    def write(self, packet):
        """
        Transmits a packet to the destination IP (if there is one) and all active
        subscribers.

        Parameters:
            packet (RoveCommPacket): A packet containing the data and header info
            to be transmitted over the rover network
        Returns:
            success (int): An integer, either 0 or 1 depending on whether or not
            an exception occured during writing
        """
        try:
            if not isinstance(packet.data, tuple):
                raise ValueError("Must pass data as a list, Data: " + str(packet.data))

            rovecomm_packet = struct.pack(
                ROVECOMM_HEADER_FORMAT,
                ROVECOMM_VERSION,
                packet.data_id,
                packet.data_count,
                types_byte_to_int[packet.data_type],
            )
            for i in packet.data:
                rovecomm_packet = rovecomm_packet + struct.pack(
                    ">" + packet.data_type, i
                )

            for subscriber in self.subscribers:
                self.RoveCommSocket.sendto(rovecomm_packet, subscriber)

            if packet.ip_address != ("0.0.0.0", 0):
                self.RoveCommSocket.sendto(rovecomm_packet, packet.ip_address)
            return 1
        except Exception:
            return 0

    def read(self):
        """
        Unpacks the UDP packet and packs it into a RoveComm Packet for easy
        parsing in other code.

        Returns:
            return_packet (RoveCommPacket): A RoveCommPacket that contains a
            RoveComm message received over the network
        """
        # The select function is used to poll the socket and check whether
        # there is data available to be read, preventing the read from
        # blocking the thread while waiting for a packet
        available_sockets = select.select([self.RoveCommSocket], [], [], 0)[0]
        if len(available_sockets) > 0:
            try:
                packet, remote_ip = self.RoveCommSocket.recvfrom(1024)
                header_size = struct.calcsize(ROVECOMM_HEADER_FORMAT)

                rovecomm_version, data_id, data_count, data_type = struct.unpack(
                    ROVECOMM_HEADER_FORMAT, packet[0:header_size]
                )
                data = packet[header_size:]

                if rovecomm_version != 2:
                    return_packet = RoveCommPacket(
                        ROVECOMM_INCOMPATIBLE_VERSION, "b", (1,), ""
                    )
                    return_packet.ip_address = remote_ip
                    return return_packet

                if data_id == ROVECOMM_SUBSCRIBE_REQUEST:
                    if self.subscribers.count(remote_ip) == 0:
                        self.subscribers.append(remote_ip)
                elif data_id == ROVECOMM_UNSUBSCRIBE_REQUEST:
                    if self.subscribers.count(remote_ip) != 0:
                        self.subscribers.remove(remote_ip)

                data_type = types_int_to_byte[data_type]
                data = struct.unpack(">" + data_type * data_count, data)

                return_packet = RoveCommPacket(data_id, data_type, data, "")
                return_packet.ip_address = remote_ip
                return return_packet

            except Exception:
                return_packet = RoveCommPacket()
                return return_packet

    def close_socket(self):
        """
        Closes the UDP socket
        """
        self.RoveCommSocket.close()


class RoveCommEthernetTcp:
    """
    The TCP implementation for RoveComm.

    Methods:
        write(packet):
            Transmits a packet to the destination IP and all active subscribers.
        read():
            Receives all TCP packets from open sockets and packs data into RoveCommPacket instances
        connect(ip_octet):
            Opens a socket connection to the given address
        close_sockets():
            Closes the server socket and all open sockets
        handle_incoming_connections():
            Accepts socket connection requests
    """

    def __init__(
        self, HOST=socket.gethostbyname(socket.gethostname()), PORT=ROVECOMM_TCP_PORT
    ):
        self.open_sockets = {}
        self.incoming_sockets = {}
        # configure a TCP socket
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Allows the socket address to be reused after being closed
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Fixes an error on linux with opening the socket again too soon
        try:
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        # bind the socket to the current machines local network IP by default (can be specified as well)
        self.server.bind((HOST, PORT))
        # accept up to 5 simulataneous connections, before we start discarding them
        self.server.listen(5)

    def close_sockets(self):
        """
        Closes all active TCP connections
        """
        for open_socket in self.open_sockets:
            # notifies other end that we are terminating the connection
            self.open_sockets[open_socket].shutdown(1)
            self.open_sockets[open_socket].close()
        self.server.close()

    def write(self, packet):
        """
        Transmits a packet to the destination IP (if there is one)

        Parameters:
            packet (RoveCommPacket): A packet containing the data and header info
            to be transmitted over the rover network
        Returns:
            success (int): An integer, either 0 or 1 depending on whether or not
            an exception occured during writing
        """
        try:
            if not isinstance(packet.data, tuple):
                raise ValueError("Must pass data as a tuple, Data: " + str(packet.data))

            rovecomm_packet = struct.pack(
                ROVECOMM_HEADER_FORMAT,
                ROVECOMM_VERSION,
                packet.data_id,
                packet.data_count,
                types_byte_to_int[packet.data_type],
            )
            for i in packet.data:
                rovecomm_packet = rovecomm_packet + struct.pack(
                    ">" + packet.data_type, i
                )

            for address in self.incoming_sockets:
                self.incoming_sockets[address].send(rovecomm_packet)

            if packet.ip_address != ("0.0.0.0", 0):
                # establish a new connection if the destination has not yet been connected to yet
                if self.connect(packet.ip_address) == 0:
                    return 0

                self.open_sockets[packet.ip_address].send(rovecomm_packet)

            return 1
        except Exception:
            return 0

    def connect(self, address):
        """
        Opens a socket connection to the address given as a parameter
        """
        if address not in self.open_sockets:
            TCPSocket = socket.socket(type=socket.SOCK_STREAM)
            try:
                TCPSocket.connect(address)
            except Exception as e:
                logging.getLogger(__name__).error(
                    "Something's wrong. Exception is %s" % (e)
                )
                return 0
            self.open_sockets[address] = TCPSocket
        return 1

    def handle_incoming_connection(self):
        """
        Polls for an incoming connection, accepts it if one exists
        """
        # The select function is used to poll the socket and check whether
        # there is an incoming connection to accept, preventing the read
        # from blocking the thread while waiting for a request
        if len(select.select([self.server], [], [], 0)[0]) > 0:
            conn, addr = self.server.accept()
            self.open_sockets[addr[0]] = conn
            self.incoming_sockets[addr[0]] = conn

    def read(self):
        """
        Unpacks the UDP packet and packs it into a RoveComm Packet for easy
        parsing in other code.

        Returns:
            packets (tuple of RoveCommPacket instances): RoveCommPackets
            that contain RoveComm messages received over the network
        """

        packets = []

        available_sockets = []
        for key, value in self.open_sockets.items():
            available_sockets.append(value)

        if len(available_sockets) > 0:
            # The select function is used to poll the socket and check whether
            # there is data available to be read, preventing the read from
            # blocking the thread while waiting for a packet
            available_sockets = select.select(available_sockets, [], [], 0)[0]

        for open_socket in available_sockets:
            try:
                header = open_socket.recv(5)
                rovecomm_version, data_id, data_count, data_type = struct.unpack(
                    ROVECOMM_HEADER_FORMAT, header
                )
                data_type_byte = types_int_to_byte[data_type]
                data = open_socket.recv(data_count * types_byte_to_size[data_type_byte])

                if rovecomm_version != 2:
                    returnPacket = RoveCommPacket(
                        ROVECOMM_INCOMPATIBLE_VERSION, "b", (1,), ""
                    )
                    returnPacket.SetIp(*open_socket.getpeername())
                    packets.append(returnPacket)

                else:
                    data_type = types_int_to_byte[data_type]
                    data = struct.unpack(">" + data_type * data_count, data)

                    returnPacket = RoveCommPacket(data_id, data_type, data, "")
                    returnPacket.SetIp(*open_socket.getpeername())
                    packets.append(returnPacket)
            except Exception:
                returnPacket = RoveCommPacket()
                packets.append(returnPacket)

        return packets
