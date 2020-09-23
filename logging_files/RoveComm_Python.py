import socket
import socketserver
import struct
import threading
import time

ROVECOMM_PORT 			= 11000
ROVECOMM_VERSION 		= 2
ROVECOMM_HEADER_FORMAT 	= ">BHBB"

ROVECOMM_PING_REQUEST 			= 1
ROVECOMM_PING_REPLY				= 2
ROVECOMM_SUBSCRIBE_REQUEST 		= 3
ROVECOMM_UNSUBSCRIBE_REQUEST	= 4
ROVECOMM_INCOMPATIBLE_VERSION 	= 5

types_int_to_byte = {
    0: 'b',
    1: 'B',
    2: 'h',
    3: 'H',
    4: 'l',
    5: 'L',
    6: 'f',
    7: 's'
}

types_byte_to_int = {
    'b': 0,
    'B': 1,
    'h': 2,
    'H': 3,
    'l': 4,
    'L': 5,
    'f': 6,
    's': 7
}

types_byte_to_size = {
    'b': 1,
    'B': 1,
    'h': 2,
    'H': 2,
    'l': 4,
    'L': 4,
    'f': 4,
    's': 1
}


class RoveCommPacket:
    def __init__(self, data_id=0, data_type='b', data=(), ip_octet_4='', port=ROVECOMM_PORT):
        self.data_id = data_id
        self.data_type = data_type
        self.data_count = len(data)
        self.data = data
        if (ip_octet_4 != ''):
            self.address = ('192.168.1.' + ip_octet_4, port)
        else:
            self.address = ('0.0.0.0', port)
        return
    
    def SetIp(self, ip_address):
        self.address = (ip_address, self.address[1])
    
    def print(self):
        print('----------')
        print('{0:6s} {1}'.format('ID:', self.data_id))
        print('{0:6s} {1}'.format('Type:', self.data_type))
        print('{0:6s} {1}'.format('Count:', self.data_count))
        print('{0:6s} {1}'.format('IP:', self.address[0]))
        print('{0:6s} {1}'.format('Port:', self.address[1]))
        print('{0:6s} {1}'.format('Data:', self.data))
        print('----------')
    
    
class RoveCommEthernetUdp:
    def __init__(self, port=ROVECOMM_PORT):
        self.rove_comm_port = port
        self.subscribers = []

        self.RoveCommSocket = socket.socket(type=socket.SOCK_DGRAM)
        self.RoveCommSocket.setblocking(False)
        self.RoveCommSocket.bind(("", self.rove_comm_port))

    def write(self, packet):
        try:
            # packet.print()
            if not isinstance(packet.data, tuple):
                raise ValueError('Must pass data as a tuple, Data: ' + str(packet.data))

            rovecomm_packet = struct.pack(ROVECOMM_HEADER_FORMAT, ROVECOMM_VERSION, packet.data_id, packet.data_count,
                                            types_byte_to_int[packet.data_type])
            for i in packet.data:
                rovecomm_packet = rovecomm_packet + struct.pack('>' + packet.data_type, i)

            for subscriber in self.subscribers:
                    self.RoveCommSocket.sendto(rovecomm_packet, (subscriber))
            
            if (packet.address != ('0.0.0.0', 0) and not (packet.address in self.subscribers)):
                self.RoveCommSocket.sendto(rovecomm_packet, packet.address)

            return 1
        except:
            return 0

    def read(self):
        try:
            packet, remote_ip = self.RoveCommSocket.recvfrom(1024)
            header_size = struct.calcsize(ROVECOMM_HEADER_FORMAT)

            rovecomm_version, data_id, data_count, data_type = struct.unpack(ROVECOMM_HEADER_FORMAT, packet[0:header_size])
            data = packet[header_size:]

            if(rovecomm_version != 2):
                returnPacket = RoveCommPacket(ROVECOMM_INCOMPATIBLE_VERSION, 'b', (1,), '')
                returnPacket.SetIp(remote_ip)
                return returnPacket

            if (data_id == ROVECOMM_SUBSCRIBE_REQUEST):
                if (self.subscribers.count(remote_ip) == 0):
                    self.subscribers.append(remote_ip)
            elif (data_id == ROVECOMM_UNSUBSCRIBE_REQUEST):
                if (self.subscribers.count(remote_ip) != 0):
                    self.subscribers.remove(remote_ip)

            data_type = types_int_to_byte[data_type]
            data = struct.unpack('>' + data_type * data_count, data)
            returnPacket = RoveCommPacket(data_id, data_type, data, '')
            returnPacket.SetIp(remote_ip)
            return returnPacket

        except:
            returnPacket = RoveCommPacket()
            return (returnPacket)

class RoveCommEthernetTCP:

    def __init__(self, HOST=socket.gethostbyname(socket.gethostname()), PORT=11111):
        self.open_sockets = {}
        #create a semaphore to ensure we don't iterate through our socket dictionary while simulatenously modifyingit
        self.sem = threading.Semaphore()
        #configure a TCP socket
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #bind the socket to the current machines local network IP by default (can be specified as well)
        self.server.bind((HOST, PORT))
        #accept up to 5 simulataneous connections, before we start discarding them
        self.server.listen(5)
        #start up a seperate thread to handle connections
        server_thread = threading.Thread(target=self.handle_incoming_connection)
        server_thread.daemon = True
        server_thread.start()

    def close_sockets(self):
        self.sem.acquire()
        for open_socket in self.open_sockets:
            #notifies other end that we are terminating the connection
            self.open_sockets[open_socket].shutdown(1)
            self.open_sockets[open_socket].close()
        self.sem.release()

    def handle_incoming_connection(self):
        #keep adding sockets to our dictionary as clients connect
        while True:
            conn, addr = self.server.accept()
            self.sem.acquire()
            self.open_sockets[addr[0]] = conn
            self.sem.release()

    def write(self, packet):
        try:
            # packet.print()
            if not isinstance(packet.data, tuple):
                raise ValueError('Must pass data as a tuple, Data: ' + str(data))

            rovecomm_packet = struct.pack(ROVECOMM_HEADER_FORMAT, ROVECOMM_VERSION, packet.data_id, packet.data_count,
                                            types_byte_to_int[packet.data_type])
            for i in packet.data:
                rovecomm_packet = rovecomm_packet + struct.pack('>' + packet.data_type, i)
            
            #establish a new connection if the destination has not yet been connected to yet
            self.connect(packet.address)

            if (packet.address != ('0.0.0.0', 0)):
                self.open_sockets[packet.address].send(rovecomm_packet)
                return 1
        except:
            return 0

    def connect(self, address):
        self.sem.acquire()
        if not address in self.open_sockets:
            TCPSocket = socket.socket(type=socket.SOCK_STREAM)
            print("Connecting to: " + address[0])
            try:
                TCPSocket.connect(address)
            except Exception as e: 
                print("Something's wrong. Exception is %s" % (e))
            self.open_sockets[address] = TCPSocket
        self.sem.release()

    def read(self):
        self.sem.acquire()
        packets = []
        for socket in self.open_sockets:
            try:
                header = self.open_sockets[socket].recv(5)
                rovecomm_version, data_id, data_count, data_type = struct.unpack(ROVECOMM_HEADER_FORMAT, header)
                data_type_byte = types_int_to_byte[data_type]
                data = self.open_sockets[socket].recv(data_count*types_byte_to_size[data_type_byte])
            
                if(rovecomm_version != 2):
                    returnPacket = RoveCommPacket(ROVECOMM_INCOMPATIBLE_VERSION, 'b', (1,), '')
                    returnPacket.SetIp(remote_ip)
                    return returnPacket
        
                data_type = types_int_to_byte[data_type]
                data = struct.unpack('>' + data_type * data_count, data)
        
                returnPacket = RoveCommPacket(data_id, data_type, data, '')
                returnPacket.SetIp(socket[0])
                packets.append(returnPacket)
        
            except Exception as e: 
                returnPacket = RoveCommPacket()
                packets.append(returnPacket)

        self.sem.release()
        return packets
