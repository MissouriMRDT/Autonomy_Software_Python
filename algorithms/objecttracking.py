import numpy as np
import cv2
import time
import threading
# import StereoVision


class ObjectTracker(object):
    def __init__(self):
        self.GREEN_LOWER = np.array((77, 92, 14))
        self.GREEN_UPPER = np.array((255, 204, 202))
        self.MIN_RADIUS = 20
        self.FRAME_RATE = 10
        self.ball_in_frame = None
        self.center = None
        self.radius = None

        try:
            self.left = 0
            self.right = 1
            self.camera = cv2.VideoCapture(self.left)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        except:
            raise Exception("Could not connect to camera")
        pass
        grabbed, self.framebuffer = self.camera.read()
        if not grabbed:
            print("Frame capture failed")

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_filename = "whatever.avi"  # "'logs/objtracker_%s.avi'" % time.strftime("%Y%m%d-%H%M%S")
        print(video_filename)
        self.video_out = cv2.VideoWriter(video_filename, fourcc, self.FRAME_RATE, (640, 480))
        assert(self.video_out.isOpened())
        self.recording_thread = threading.Thread(target=self._record_thread)

    def __del__(self):
        self.camera.release()
        self.video_out.release()

    def track_ball(self):
        self.ball_in_frame = False

        (grabbed, frame) = self.camera.read()
        if grabbed:
                cv2.waitKey()
        if not grabbed:
            print("Frame capture failed")
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, self.GREEN_LOWER, self.GREEN_UPPER)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(contour) > 0:
            self.ball_in_frame = True
            largest_contour = max(contour, key=cv2.contourArea)
            ((x, y), self.radius) = cv2.minEnclosingCircle(largest_contour)
            M = cv2.moments(largest_contour)
            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if self.radius > self.MIN_RADIUS:
                cv2.circle(frame, (int(x), int(y)), int(self.radius), (0, 255, 255), 2)
                cv2.circle(frame, self.center, 5, (0, 0, 255), -1)

            self.video_out.write(frame)

            return self.ball_in_frame, self.center, self.radius

    def _record_thread(self):
        pass
        # while(True):
        #    (grabbed, frame) = camera.read()
        #    if not grabbed:
        #        print "Frame capture failed"
        #    self.framebuffer = frame
        #    self.video_out.write(frame)


if __name__ == '__main__':
    tracker = ObjectTracker()
    while True:
        ball_in_frame, center, radius = tracker.track_ball()
        if ball_in_frame:
            print("Ball found at %s, distance %s" % (center, 1.0/radius))
        else:
            print("No ball found")
