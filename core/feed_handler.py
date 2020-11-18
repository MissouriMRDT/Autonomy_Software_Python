import cv2
import time
import threading
import time
import sys
import multiprocessing as mp

if sys.platform == "linux":
    import pyfakewebcam


class FeedHandler:
    def __init__(self, resolution_x=640, resolution_y=480, frame_rate=30):
        self.feeds = {}
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.resolution_x = resolution_x
        self.resolution_y = resolution_y
        self.frame_rate = frame_rate

    def __del__(self):
        # Call close so we can close all the video writers
        self.close()

    def feed_process(self, pipe, num, feed_id, save_video=True, stream_video=True):
        if stream_video and sys.platform == "linux":
            streamer = pyfakewebcam.FakeWebcam(f'/dev/video{num}', self.resolution_x, self.resolution_y)  # append v4l output to list of cameras        p_output, p_input = pipe

        if save_video:
            video_filename = f'stream_{feed_id}_' + time.strftime("%Y%m%d-%H%M%S")  # save videos to unique files
            video_writer = cv2.VideoWriter(video_filename + "_left.avi", self.fourcc, self.frame_rate, (self.resolution_x, self.resolution_y))  # append video writer to list of video writers

        p_output, p_input = pipe
        p_input.close()    # We are only reading

        while True:
            data = p_output.recv()
            # Terminate process if we received an end signal
            if str(data) == "END":
                break
            image = cv2.resize(data, (640, 480))
            img = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
            if stream_video and sys.platform == "linux":
                streamer.schedule_frame(img)
            if save_video:
                video_writer.write(img)

    def add_feed(self, camera_num, feed_id, save_video=True, stream_video=True):
        # Create a process to send frames to, to be saved and scheduled to stream
        proc_output, proc_input = mp.Pipe()
        proc = mp.Process(target=self.feed_process, args=((proc_output, proc_input), camera_num, feed_id, save_video, stream_video,))
        
        proc.start()
        proc_output.close()  # We don't need output on our end

        # Add the process and pipe to the dictionary
        self.feeds[feed_id] = (proc, proc_input)

    def close(self):
        for process, pipe_in in self.feeds.items():
            # Terminate the process by sending an END signal
            pipe_in.send("END")
            process.join()

    def handle_frame(self, feed_id, img):
        # Frames is a dictionary of (process, (pipe_out, pipe_in))
        process, pipe_in = self.frames_process[feed_id]
        pipe_in.send(img)

