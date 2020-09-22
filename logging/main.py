import logging
import my_lib
# from RoveComm_Python import RoveCommEthernetUdp
# from rove_comm_handler import RoveCommHandler
from RoveComm_Python import RoveCommEthernetTCP
from rove_comm_handler_tcp import RoveCommHandlerTCP
import logging.config
import yaml
from yaml import CLoader

# set up RoveComm
# RoveCommUDP = RoveCommEthernetUdp(port=10999)
RoveCommTCP = RoveCommEthernetTCP(HOST='127.0.0.1', PORT=11112)

# Setup Logging
# RoveCommHandler.sock = RoveCommEthernetUdp(port=11000)
RoveCommHandlerTCP.sock = RoveCommEthernetTCP(HOST='127.0.0.1', PORT=11111)
yaml_conf = yaml.load(open('logging.yaml', 'r').read(), Loader=CLoader)
logging.config.dictConfig(yaml_conf)
logging.addLevelName(21, "NUMERICAL_INFO")

# some output
logging.debug('debug message')
logging.info('info message')
logging.warning('warn message')
logging.error('error message')
logging.critical('critical message')
my_lib.my_function()

# read in RoveComm logs
print("----------------Sent through RoveComm----------------")
packets = []
# for i in range(3):
#     packets.append(RoveCommUDP.read())
for i in range(5):
    packets.extend(RoveCommTCP.read())

for packet in packets:
    if packet.data_type == 's':
        # Reconstruct char[] into a string
        data = packet.data
        data_string = ""
        for s in data:
            data_string += s.decode('utf-8')
        print(data_string)
    else:
        packet.print()

# clean up
RoveCommTCP.close_sockets()
RoveCommHandlerTCP.sock.close_sockets()
# RoveCommUDP.RoveCommSocket.close()
# RoveCommHandler.sock.RoveCommSocket.close()
