import pyfakewebcam
import cv2
import time


class VideoHandler:
    def __init__(self, resolution_x=640, resolution_y=480, frame_rate=30):
        self.cameras = {}
        self.video_writers = {}
        self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        self.resolution_x = resolution_x
        self.resolution_y = resolution_y
        self.frame_rate = frame_rate

    def __del__(self):
        # Call close so we can close all the video writers
        self.close()

    def add_video(self, camera_num, camera_id, save_video=True, stream_video=True):
        if stream_video:
            self.cameras[camera_id] = pyfakewebcam.FakeWebcam(f'/dev/video{camera_num}', self.resolution_x, self.resolution_y)  # append v4l output to list of cameras

        if save_video:
            video_filename = f'stream_{camera_id}_' + time.strftime("%Y%m%d-%H%M%S")  # save videos to unique files
            self.video_writers[camera_id] = cv2.VideoWriter(video_filename + "_left.avi", self.fourcc, self.frame_rate, (self.resolution_x, self.resolution_y))  # append video writer to list of video writers

    def close(self):
        for key in self.video_writers:
            self.video_writers[key].release()

    def handle_frame(self, frame_id, frame):
        # Stream and save frames
        if frame_id in self.cameras:
            img = cv2.resize(frame, (640, 480))
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            self.cameras[frame_id].schedule_frame(img)

        if frame_id in self.video_writers:
            self.video_writers.write(frame)
