import logging
from core.vision.feed_handler import FeedHandler
import threading
import socket, cv2, pickle, struct


class SimCamHandler:
    def __init__(self):
        """
        Sets up the simulator camera with the specified parameters
        """

        # Create socket to receive incoming frames streamed from Simulator
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host_ip = "127.0.0.1"
        self.port = 9999
        self.client_socket.connect((self.host_ip, self.port))

        self.feed_handler = FeedHandler()
        self.logger = logging.getLogger(__name__)

        # Add the desired feeds
        self.feed_handler.add_feed(2, "regular")

        # Create initial frames
        self.reg_img = None
        self.depth_img = None

        # Create thread to constantly grab frames, and pass them to other processes to stream/save
        self._stop = threading.Event()

        self.thread = threading.Thread(target=self.frame_grabber, args=())

    # Should this be a generator or a thread? Generator might help cuz I could schedule this in the ASYNC calls
    def frame_grabber(self):
        """
        Function to be executed as a thread, grabs latest depth/regular images
        from network and then passes them to the respective feed handlers
        """

        while not self._stop.is_set():
            data = b""
            payload_size = struct.calcsize("Q")
            while True:
                # First read the message size, we know it's a 8 byte number
                while len(data) < payload_size:
                    packet = self.client_socket.recv(4 * 1024)
                    if not packet:
                        break
                    data += packet

                # Grab only the payload size
                packed_msg_size = data[:payload_size]
                # Store rest of data
                data = data[payload_size:]
                # Calculate the message size
                msg_size = struct.unpack("Q", packed_msg_size)[0]

                # Now keep reading the payload until we have read in all expected data
                while len(data) < msg_size:
                    data += self.client_socket.recv(4 * 1024)
                frame_data = data[:msg_size]
                data = data[msg_size:]
                self.reg_img = pickle.loads(frame_data)

                # Now let the feed_handler stream/save the frames
                self.feed_handler.handle_frame("regular", self.reg_img)

    def grab_regular(self):
        """
        Returns the latest regular frame captured from the simulator
        """
        return self.reg_img

    def grab_depth(self):
        """
        Returns the latest depth frame captured from the simulator
        """
        self.logger.error("Tried calling grap_depth() for simulator! Not supported currently")
        return None

    def start(self):
        """
        Starts up the frame grabber thread, which constantly polls the simulator camera
        for new frames
        """
        self.thread.start()
        self.logger.info("Starting Simulator capture")

    def close(self):
        """
        Closes the simulator camera as well as feed handler
        """
        # Set the threading event so we kill the thread
        self._stop.set()
        # Wait for the thread to join
        self.thread.join()

        # Now close the socket we were reading in from
        self.client_socket.close()

        # Close the feed handler as well
        self.feed_handler.close()

        self.logger.info("Closing Simulator capture")

    def grab_point_cloud(self):
        """
        Returns 3D point cloud data captured with simulator
        """
        self.logger.error("Tried calling grab_point_cloud() for simulator! Not supported currently")
        return None
