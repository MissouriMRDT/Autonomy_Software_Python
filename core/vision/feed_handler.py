import cv2
import time
import sys
import multiprocessing as mp

# Pyfakewebcam requires linux
if sys.platform == "linux":
    import pyfakewebcam


def feed_process(
    pipe,
    num,
    feed_id,
    fourcc,
    frame_rate,
    resolution_x,
    resolution_y,
    save_video=True,
    stream_video=True,
):
    """
    Function to be run as a process, configures streaming and recording of frames.
    Uses pipe to receive incoming frames and stream/save
    """

    # Only attempt to stream video if on Linux (due to package dependancies)
    if stream_video and sys.platform == "linux":
        streamer = pyfakewebcam.FakeWebcam(
            f"/dev/video{num}", resolution_x, resolution_y
        )  # append v4l output to list of cameras

    if save_video:
        video_filename = f"logs/stream_{feed_id}_" + time.strftime("%Y%m%d-%H%M%S")  # save videos to unique files
        video_writer = cv2.VideoWriter(
            video_filename + "_left.avi",
            fourcc,
            frame_rate,
            (resolution_x, resolution_y),
        )  # append video writer to list of video writers

    p_output, p_input = pipe
    p_input.close()  # We are only reading

    while True:
        data = p_output.recv()
        # Resize image to reduce bandwidth/size
        image = cv2.resize(data, (resolution_x, resolution_y))
        # OpenCV video writer expects BGR color channels
        save_img = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        # Motion expects RGB color channels
        stream_img = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)

        # Stream and record video if applicable
        if stream_video and sys.platform == "linux":
            streamer.schedule_frame(stream_img)
        if save_video:
            video_writer.write(save_img)


class FeedHandler:
    def __init__(self, resolution_x=640, resolution_y=480, frame_rate=30):
        """
        Configure the resolution and framerate of all feed handlers
        """

        self.feeds = {}
        self.fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self.resolution_x = resolution_x
        self.resolution_y = resolution_y
        self.frame_rate = frame_rate

    def add_feed(self, camera_num, feed_id, save_video=True, stream_video=True):
        """
        Adds a new feed and corresponding process, takes care of configure process correctly and creating pipe
        """
        # Create a process to send frames to, to be saved and scheduled to stream
        proc_output, proc_input = mp.Pipe()
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
                save_video,
                stream_video,
            ),
        )

        proc.start()
        proc_output.close()  # We don't need output on our end

        # Add the process and pipe to the dictionary
        self.feeds[feed_id] = (proc, proc_input)

    def close(self):
        """
        Closes all feeds and waits for processes to join
        """
        for feed_id, (process, pipe_in) in self.feeds.items():
            # Close the pipe
            pipe_in.close()
            # Terminate all the processes and wait to join
            process.terminate()
            process.join()

    def handle_frame(self, feed_id, img):
        """
        Passes the image to a corresponding process to stream/save the frame
        """
        # Frames is a dictionary of (process, pipe_in)
        process, pipe_in = self.feeds[feed_id]
        pipe_in.send(img)