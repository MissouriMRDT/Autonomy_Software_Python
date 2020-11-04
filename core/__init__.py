from core.rovecomm import RoveComm, RoveCommPacket
import core.constants
import core.logging
import core.notify
import core.rover_states
import core.state
from core.video_handler import VideoHandler

# RoveComm node, must be initialized before it can be used.
rovecomm: RoveComm

# Video handler, used to keep track of video feeds we want to stream/record
video_handler: VideoHandler
