from core.vision.cameras.camera import Camera
import logging
from core.vision import feed_handler
import threading
import socket, cv2, pickle, struct
import gzip
import numpy as np


class WebCam(Camera):
    def __init__(self):
        """
        Sets up the simulator camera with the specified parameters
        """

        # Set up cv2 webcam capture
        self.cap = cv2.VideoCapture(0)

        self.feed_handler = feed_handler
        self.logger = logging.getLogger(__name__)

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

        # Create thread to constantly grab frames, and pass them to other processes to stream/save
        self._stop = threading.Event()

        self.thread = threading.Thread(target=self.frame_grabber, args=())

    def frame_grabber(self):
        """
        Function to be executed as a thread, grabs latest depth/regular images
        from webcam
        """
        while not self._stop.is_set():

            # Capture frame-by-frame
            ret, frame = self.cap.read()

            frame = cv2.resize(frame, (self.reg_res_x, self.reg_res_y))
            self.reg_img = frame

            # Wait for the desired framerate between frames
            if cv2.waitKey(int(1000 / self.fps)):
                break

        # Now let the feed_handler stream/save the frames
        self.feed_handler.handle_frame("regular", self.reg_img)

    def grab_regular(self):
        """
        Returns the latest regular frame captured from the simulator
        """
        return self.reg_img

    def start(self):
        """
        Starts up the frame grabber thread, which constantly polls the simulator camera
        for new frames
        """
        self.thread.start()
        self.logger.info("Starting Webcam capture")

    def close(self):
        """
        Closes the simulator camera as well as feed handler
        """
        # Set the threading event so we kill the thread
        self._stop.set()

        # Close cv2 capture
        self.cap.close()

        # Wait for the thread to join
        self.thread.join()

        # Close the feed handler as well
        self.feed_handler.close()

        self.logger.info("Closing Webcam capture")
