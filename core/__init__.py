from core.rovecomm import RoveCommEthernetUdp, RoveCommPacket
from core.rovecomm_TCP import RoveCommEthernetTCP
import core.rovecomm_send
import core.constants
import core.logging
import core.notify
import core.rover_states
import core.state

# RoveComm node, must be initialized before it can be used.
rovecomm_node: RoveCommEthernetUdp = None
rovecomm_node_tcp: RoveCommEthernetTCP = None
