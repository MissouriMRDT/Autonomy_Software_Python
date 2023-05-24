#
# Mars Rover Design Team
# feed_handler.py
#
# Created on Jan 09, 2021
# Updated on Aug 21, 2022
#

import cv2
import time
import sys
import multiprocessing as mp
import os

if sys.platform == "linux":
    # Pyfakewebcam requires linux
    from pyfakewebcam import FakeWebcam


def feed_process(
    pipe,
    num,
    feed_id,
    fourcc,
    frame_rate,
    resolution_x,
    resolution_y,
    _event,
    save_video=True,
    stream_video=True,
):
    """
    Function to be run as a process, configures streaming and recording of frames.
    Uses pipe to receive incoming frames and stream/save

    :param pipe:
    :param num:
    :param feed_id:
    :param fourcc:
    :param frame_rate:
    :param resolution_x:
    :param resolution_y:
    :param _event:
    :param save_video: defaults to True
    :param stream_video: defaults to True
    """

    # Only attempt to stream video if on Linux (due to package dependencies)
    if stream_video and sys.platform == "linux":
        streamer: FakeWebcam = FakeWebcam(
            f"/dev/video{num}", int(resolution_x / 2), int(resolution_y / 2)
        )  # append v4l output to list of cameras

    if save_video:
        if not os.path.isdir("logs/videos/"):
            os.mkdir("logs/videos/")
        video_filename = f"logs/videos/stream_{feed_id}_" + time.strftime(
            "%Y%m%d-%H%M%S"
        )  # save videos to unique files
        video_writer = cv2.VideoWriter(
            video_filename + "_left.avi",
            fourcc,
            frame_rate,
            (resolution_x, resolution_y),
        )  # append video writer to list of video writers

    p_output, p_input = pipe
    p_input.close()  # We are only reading

    while not _event.is_set():
        try:
            if p_output.poll(0):
                data = p_output.recv()
                data = cv2.imdecode(data, 1)
                # OpenCV video writer expects BGR color channels
                save_img = cv2.cvtColor(data, cv2.COLOR_BGRA2BGR)
                # Motion expects RGB color channels
                stream_img = cv2.cvtColor(data, cv2.COLOR_BGRA2RGB)

                # Stream and record video if applicable
                if stream_video and sys.platform == "linux":
                    stream_img = cv2.resize(stream_img, (int(resolution_x / 2), int(resolution_y / 2)))
                    streamer.schedule_frame(stream_img)
                if save_video:
                    video_writer.write(save_img)
        except KeyboardInterrupt:
            p_output.close()
            exit(0)

        # We only need to handle frames at the desired frame rate (no need to be blocking)
        time.sleep(1 / 30)


class FeedHandler:
    def __init__(self, resolution_x=1280, resolution_y=720, frame_rate=30):
        """
        Configure the resolution and framerate of all feed handlers

        :param resolution_x: defaults to 1280
        :param resolution_y: defaults to 720
        :param frame_rate: defaults to 30
        """

        self.feeds = {}
        self.fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self.resolution_x = resolution_x
        self.resolution_y = resolution_y
        self.frame_rate = frame_rate

    def add_feed(self, camera_num, feed_id, save_video=True, stream_video=True):
        """
        Adds a new feed and corresponding process, takes care of configure process correctly and creating pipe

        :param camera_num:
        :param feed_id:
        :param save_video: defaults to True
        :param stream_video: defaults to True
        """
        # Create a process to send frames to, to be saved and scheduled to stream
        proc_output, proc_input = mp.Pipe()
        _event = mp.Event()

        proc = mp.Process(
            target=feed_process,
            args=(
                (proc_output, proc_input),
                camera_num,
                feed_id,
                self.fourcc,
                self.frame_rate,
                self.resolution_x,
                self.resolution_y,
                _event,
                save_video,
                stream_video,
            ),
        )

        proc.start()
        proc_output.close()  # We don't need output on our end

        # Add the process and pipe to the dictionary
        self.feeds[feed_id] = (proc, proc_input, _event)

    def close(self):
        """
        Closes all feeds and waits for processes to join
        """
        for feed_id, (process, pipe_in, _event) in self.feeds.items():
            # Terminate all the processes and wait to join
            _event.set()

            # Close the pipe
            pipe_in.close()

            process.terminate()
            process.join()

    def handle_frame(self, feed_id, img):
        """
        Passes the image to a corresponding process to stream/save the frame

        :param feed_id:
        :param img:
        """
        # Frames is a dictionary of (process, pipe_in)
        process, pipe_in, _event = self.feeds[feed_id]

        # Resize the image
        img = cv2.resize(img, (self.resolution_x, self.resolution_y))

        # Encode the image before pickling to speed up
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        result, encimg = cv2.imencode(".jpg", img, encode_param)

        # Try writing if pipe is still open
        try:
            pipe_in.send(encimg)
        except BrokenPipeError:
            return
