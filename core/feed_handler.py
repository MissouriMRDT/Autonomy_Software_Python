import pyfakewebcam
import cv2
import time
import threading
import time


class Feed:
    def __init__(self, streamer=None, video_writer=None):
        self.streamer = streamer
        self.video_writer = video_writer
    

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

    def add_feed(self, camera_num, feed_id, save_video=True, stream_video=True):
        # Create a Feed        

        feed = Feed()

        if stream_video:
            feed.streamer = pyfakewebcam.FakeWebcam(f'/dev/video{camera_num}', self.resolution_x, self.resolution_y)  # append v4l output to list of cameras
        if save_video:
            video_filename = f'stream_{feed_id}_' + time.strftime("%Y%m%d-%H%M%S")  # save videos to unique files
            feed.video_writer = cv2.VideoWriter(video_filename + "_left.avi", self.fourcc, self.frame_rate, (self.resolution_x, self.resolution_y))  # append video writer to list of video writers

        # Add feed to our dictionary
        self.feeds[feed_id] = feed

    def close(self):
        for __, feed in self.feeds.items():
            feed.video_writer.release()

    def handle_frame(self, feed_id, img):
        # Stream and save frames
        img = cv2.resize(img, (640, 480))

        if self.feeds[feed_id].streamer is not None:
            image_resized = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            self.feeds[feed_id].streamer.schedule_frame(image_resized)

        if self.feeds[feed_id].video_writer is not None:
            image_resized = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            self.feeds[feed_id].video_writer.write(image_resized)
