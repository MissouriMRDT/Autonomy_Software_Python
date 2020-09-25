from logging_files.RoveComm_Python import RoveCommPacket
from logging_files.rove_comm_handler_tcp import RoveCommHandlerTCP
import logging
import yaml
from yaml import CLoader


class RoveCommHandlerNumerical(RoveCommHandlerTCP):
    def __init__(self, target_host, target_port):
        """
        Initializes the handler
        """
        RoveCommHandlerTCP.__init__(self, target_host, target_port)
        self.event_list = open('logging_files/values.yaml', 'r').read()
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
            packet = RoveCommPacket(
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
