from core.rovecomm import RoveCommPacket
from core.rovecomm_TCP import RoveCommPacket as RoveCommPacketTCP
import core
import logging
import yaml
from yaml import CLoader
from datetime import datetime


class CsvHandler(logging.handlers.WatchedFileHandler):

    def __init__(self, filename, format_string, encoding=None, delay=False):
        """
        Initializes the handler
        """
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        f = open(f'{filename[:-4]}-{timestamp}{filename[-4:]}', 'w')
        f.write(format_string + '\n')
        f.close()

        logging.handlers.WatchedFileHandler.__init__(self, f'{filename[:-4]}-{timestamp}{filename[-4:]}', 'a', encoding, delay)


class RoveCommHandlerUDP(logging.Handler):

    def __init__(self, target_host, target_port):
        """
        Initializes the handler with given network variables
        """
        logging.Handler.__init__(self)
        self.target_host = target_host
        self.target_port = target_port

    def emit(self, s):
        """
        Encodes and sends the log message over RoveComm
        """
        if core.rovecomm_node is None:
            logging.warning('UDP Handler called with no socket')
            return

        msg = self.format(s)
        # Max string size is 255 characters, truncate the rest and flag it
        if len(msg) > 255:
            msg = msg[:252] + "..."

        packet = RoveCommPacket(
            4242,
            's',
            tuple([char.encode('utf-8') for char in msg]),
            "",
            self.target_port
        )
        packet.SetIp(self.target_host)
        core.rovecomm_node.write(packet)


class RoveCommHandlerTCP(logging.Handler):
    sock = None

    def __init__(self, target_host, target_port):
        """
        Initializes the handler
        """
        logging.Handler.__init__(self)
        self.target_host = target_host
        self.target_port = target_port

    def emit(self, s):
        """
        Encodes and sends the log message over RoveComm
        """
        if RoveCommHandlerTCP.sock is None:
            logging.warning('TCP Handler called with no socket')
            return

        msg = self.format(s)
        # Max string size is 255 characters, truncate the rest and flag it
        if len(msg) > 255:
            msg = msg[:252] + "..."

        packet = RoveCommPacketTCP(
            4242,
            's',
            tuple([char.encode('utf-8') for char in msg]),
            "",
            self.target_port
        )
        packet.SetIp(self.target_host)
        self.sock.write(packet)


class RoveCommHandlerNumerical(RoveCommHandlerTCP):
    def __init__(self, target_host, target_port):
        """
        Initializes the handler
        """
        RoveCommHandlerTCP.__init__(self, target_host, target_port)
        self.event_list = open('core/RoveCommValues.yaml', 'r').read()
        self.event_list = yaml.load(self.event_list, Loader=CLoader)

    def emit(self, s):
        """
        Sends some data over the socket based on log message
        """
        if RoveCommHandlerTCP.sock is None:
            logging.warning('TCP Handler called with no socket')
            return

        data = s.msg.split(" - ")
        # Matches log 'event' to event from predefined dict
        if data[0] in self.event_list:
            event = self.event_list[data[0]]
            data_id = event['data_id']
            data_type = event['data_type']
            # If the event uses preset values, match the correct one
            if 'values' in event:
                if data[1] in event['values']:
                    value = tuple(event['values'][data[1]])
                else:
                    logging.warning(
                        f'{data[1]} is not a valid value for {data[0]}')
                    value = ()
            # Otherwise evaluate the string as the tuple it should be
            else:
                value = eval(data[1])

            # Pack up and send the data
            packet = RoveCommPacketTCP(
                data_id,
                data_type,
                value,
                "",
                self.target_port
            )
            packet.SetIp(self.target_host)
            packet.print()
            self.sock.write(packet)
        else:
            logging.warning(f'{data[0]} is not a valid event')
