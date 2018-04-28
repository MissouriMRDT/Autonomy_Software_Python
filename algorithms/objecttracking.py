import numpy as np
import cv2
import time
import threading

GREEN_LOWER = np.array(cv2.cv.Scalar(29, 86, 6))
GREEN_UPPER = np.array(cv2.cv.Scalar(64, 255, 255))
MIN_RADIUS  = 20
FRAME_RATE  = 10


class ObjectTracker(object):
    def __init__(self, camera=0):
        try:
            self.camera = cv2.VideoCapture(camera)
            self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
        except:
            raise Exception("Could not connect to camera")
        pass
        grabbed, self.framebuffer = self.camera.read()
        if not grabbed:
            print "Frame capture failed"
    
        fourcc = cv2.cv.CV_FOURCC(*'XVID')
        video_filename = 'logs/objtracker_%s.avi' % time.strftime("%Y%m%d-%H%M%S")
        print(video_filename)
        self.video_out = cv2.VideoWriter(video_filename, fourcc, FRAME_RATE, (640, 480))
        assert(self.video_out.isOpened())
        self.recording_thread = threading.Thread(target=self._record_thread)
    
    def __del__(self):
        self.camera.release()
        self.video_out.release()

    def track_ball(self):
        ball_in_frame = False
        center = None
        radius = None

        (grabbed, frame) = self.camera.read()
        if not grabbed:
            print ("Frame capture failed")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(contour) > 0:
            ball_in_frame = True
            largest_contour = max(contour, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            M = cv2.moments(largest_contour)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > MIN_RADIUS:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
        self.video_out.write(frame)

        return ball_in_frame, center, radius
    
    def _record_thread(self):
        pass
        #while(True):
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
            print ("Ball found at %s, distance %s" % (center, 1.0/radius))
        else:
            print ("No ball found")
