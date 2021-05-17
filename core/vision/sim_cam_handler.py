from core.vision.camera import Camera
import logging
from core.vision import feed_handler
import threading
import socket, cv2, pickle, struct
import gzip
import numpy as np


class SimCamHandler(Camera):
    def __init__(self):
        """
        Sets up the simulator camera with the specified parameters
        """

        # Create socket to receive incoming frames streamed from Simulator
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host_ip = "127.0.0.1"
        self.port = 9999
        self.client_socket.connect((self.host_ip, self.port))
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        self.feed_handler = feed_handler
        self.logger = logging.getLogger(__name__)
        self.r_lock = threading.RLock()

        # Define the camera resolutions
        self.depth_res_x = 640
        self.depth_res_y = 360
        self.reg_res_x = 1280
        self.reg_res_y = 720
        self.hfov = 85

        # Desired FPS
        self.fps = 30

        # Add the desired feeds
        self.feed_handler.add_feed(10, "regular")

        # Create initial frames
        self.reg_img = None
        self.depth_img = None

        # Create thread to constantly grab frames, and pass them to other processes to stream/save
        self._stop = threading.Event()

        self.thread = threading.Thread(target=self.frame_grabber, args=())

    def frame_grabber(self):
        """
        Function to be executed as a thread, grabs latest depth/regular images
        from network and then passes them to the respective feed handlers
        """
        data = b""

        # Size of the number that specifies the size of frame
        data_length_size = struct.calcsize("Q")
        # Size of the character that indicates frame type
        type_size = struct.calcsize("c")

        # An image frame message streamed from the simulator contains the following info:
        # +---------------------+--------------------------------+------------------------+
        # |     uint32t (Q)     |            char (C)            |  message (bytearray)   |
        # +---------------------+--------------------------------+------------------------+
        # | The number of bytes | The type of image frame:       | The bytes in the image |
        # | in the image frame  | "r" for regular, "d" for depth | frame itself           |
        # +---------------------+--------------------------------+------------------------+

        while not self._stop.is_set():
            # First read the in the data length specifier (should be 8 bits)
            while len(data) < data_length_size:
                packet = self.client_socket.recv(4 * 1024)
                if not packet:
                    break
                data += packet

            # Grab only the data length size
            packed_msg_size = data[:data_length_size]
            data = data[data_length_size:]

            # The first element in data is the type of frame encoded as a single byte
            # Either "r" or "d" for regular or depth
            while len(data) < type_size:
                data += self.client_socket.recv(4 * 1024)
            type = data[:type_size]
            data = data[type_size:]
            msg_type = struct.unpack("c", type)[0]

            # Calculate the message size
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            # Now keep reading the payload until we have read in all expected data in
            # the frame
            while len(data) < msg_size:
                data += self.client_socket.recv(4 * 1024)

            frame_data = data[:msg_size]
            data = data[msg_size:]

            # For regular images or depth data we have different decompression techniques
            # this is due to the type of data we are sending
            self.r_lock.acquire()
            if msg_type == b"r":
                self.encoded_img = pickle.loads(frame_data)
                self.reg_img = cv2.imdecode(self.encoded_img, 1)
            elif msg_type == b"d":
                self.encoded_img = gzip.decompress(frame_data)
                self.encoded_img = pickle.loads(self.encoded_img)
                self.depth_data = struct.unpack(str(int(len(self.encoded_img) / 4)) + "f", self.encoded_img)
            self.r_lock.release()

            # Now let the feed_handler stream/save the frames
            self.feed_handler.handle_frame("regular", self.reg_img)

    def grab_regular(self):
        """
        Returns the latest regular frame captured from the simulator
        """
        self.r_lock.acquire()
        reg = self.reg_img.copy()
        self.r_lock.release()
        return reg

    def grab_depth(self):
        """
        Returns the latest depth frame captured from the simulator
        """
        self.logger.error("Tried calling grap_depth() for simulator! Not supported currently")
        return None

    def grab_depth_data(self):
        """
        Returns the depth matrix (in meters) ahead of the current rover
        """
        self.r_lock.acquire()
        # Convert depth data to numpy array
        self.depth_data = np.asarray(self.depth_data)
        # Resize current data (in list form) to matrix with expected dimensions
        self.depth_data = self.depth_data.reshape((self.depth_res_y, self.depth_res_x, 1))
        depth_data = self.depth_data.copy()
        self.r_lock.release()
        return depth_data

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
