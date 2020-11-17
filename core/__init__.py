from core.rovecomm import RoveComm, RoveCommPacket
import core.constants
import core.logging
import core.notify
import core.rover_states
import core.state
from core.feed_handler import FeedHandler
from core.zed_handler import ZedHandler

# RoveComm node, must be initialized before it can be used.
rovecomm: RoveComm

# Feed handler, used to keep track of video feeds we want to stream/record
feed_handler: FeedHandler

# Zed Handler, used to setup ZED and grab frames/point cloud data
zed: ZedHandler
