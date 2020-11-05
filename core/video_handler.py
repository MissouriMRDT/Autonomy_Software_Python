import pyfakewebcam
import cv2
import time
import pyzed.sl as sl
import threading
import time

class Feed:
    def __init__(self, mat, view, streamer=None, video_writer=None):
        self.streamer = streamer
        self.video_writer = video_writer
        self.mat = mat
        self.view = view
    


class VideoHandler:
    def __init__(self, resolution_x=640, resolution_y=480, frame_rate=30):
        self.feeds = {}
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.resolution_x = resolution_x
        self.resolution_y = resolution_y
        self.frame_rate = frame_rate
        self.stopped = False

        # Create a ZED camera object
        self.zed = sl.Camera()

        # Set configuration parameters
        input_type = sl.InputType()
    
        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = sl.RESOLUTION.HD720
        init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init.coordinate_units = sl.UNIT.MILLIMETER
        init.camera_fps = 30


        # Open the camera
        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS :
            print(repr(err))
            self.zed.close()
            exit(1)

        # Set runtime parameters after opening the camera
        self.runtime = sl.RuntimeParameters()
        self.runtime.sensing_mode = sl.SENSING_MODE.STANDARD

        # Prepare new image size to retrieve half-resolution images
        self.image_size = self.zed.get_camera_information().camera_resolution
        self.image_size.height = self.image_size.height / 2
        self.image_size.width = self.image_size.width /2 
        self.update_thread = threading.Thread(target=self.update)

    def __del__(self):
        # Call close so we can close all the video writers
        self.close()

    def add_video(self, zed_view, zed_mat, camera_num, feed_id, save_video=True, stream_video=True):
        # Create a Feed
        feed = Feed(sl.Mat(self.image_size.width, self.image_size.height, zed_mat), zed_view)

        if stream_video:
            feed.streamer = pyfakewebcam.FakeWebcam(f'/dev/video{camera_num}', self.resolution_x, self.resolution_y)  # append v4l output to list of cameras
        if save_video:
            video_filename = f'stream_{feed_id}_' + time.strftime("%Y%m%d-%H%M%S")  # save videos to unique files
            feed.video_writer = cv2.VideoWriter(video_filename + "_left.avi", self.fourcc, self.frame_rate, (self.resolution_x,self.resolution_y))  # append video writer to list of video writers
        
        # Add feed to our dictionary
        self.feeds[feed_id] = feed
    
    def start(self):
        self.update_thread.start()

    def close(self):
        self.stopped = True
        for __, feed in self.feeds.items():
            feed.video_writer.release()
    
    def update(self):
        # Go through the existing feeds and read in new images and handle their 
        err = self.zed.grab(self.runtime)
        if err == sl.ERROR_CODE.SUCCESS :
            for feed_id in self.feeds:
                self.zed.retrieve_image(self.feeds[feed_id].mat, self.feeds[feed_id].view, sl.MEM.CPU, self.image_size)
                self.handle_frames(self.feeds[feed_id])

    def grab_frame(self, feed_id):
        # Return the data for the latest frame in the specified feed
        return self.feeds[feed_id].mat.get_data()

    def handle_frames(self, feed):
        # Stream and save frames
        img = feed.mat.get_data()
        img = cv2.resize(img, (640, 480))
        if feed.streamer is not None:
            image_resized = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            feed.streamer.schedule_frame(image_resized)

        if feed.video_writer is not None:
            image_resized = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            feed.video_writer.write(image_resized)
