import numpy as np
import cv2
import time


# COLOR BOUNDS FOR OTHER COLORED TENNIS BALLS
# hot pink:
#     pinkUpper = np.array((250, 149, 165))
#     pinkLower = np.array((126, 41, 57))

# bad hot pink:
#     pinkUpper = np.array((227, 95, 135))
#     pinkLower = np.array((142, 8, 12))
# next one upper =  246, 125, 122
# next one lower = 175, 75, 75

# bad hot pink outside:
#     pinkUpper = np.array((246, 125, 122))
#     pinkLower = np.array((119, 28, 25))
#  new upper: 255, 164, 147

# old bounds
# tennis ball:
#     self.GREEN_LOWER = np.array((55, 65, 40))
#     self.GREEN_UPPER = np.array((135, 150, 85))

#outside
# upper = 200, 227, 115
# lower = 103, 115, 65

# 5/28 green outside overcast
# upper:138 207 41
# lower: 83, 92, 26

# 5/29 green outside sunny
# upper: 201 243 110
# lower: 145 165 55

# urc green
# upper: 205 209 100
# lower: 147 156 79

class ObjectTracker(object):
    def __init__(self, testing = False, video_filename=None):
        self.LOWER = np.array((147, 156, 79))
        self.UPPER = np.array((205, 209, 100))
        self.MIN_RADIUS = 10
        self.FRAME_RATE = 10
        self.ball_in_frame = None
        self.center = None
        self.radius = 0
        self.firstRun = 0
        self.testing = testing
        finalVideoOut = "logs/objtracker" + time.strftime("%Y%m%d-%H%M%S") + ".avi" # for outputting the video stream with tennis ball tracking
        # self.circle = np.array([[[375, 52]], [[232, 84]], [[138, 200]], [[162, 337]], [[226, 403]], [[286, 427]],
                                # [[415, 407]], [[475, 359]], [[511, 293]], [[503, 156]], [[453, 90]]])

        # connects cameras, creates file
        try:
            self.camera = cv2.VideoCapture(1)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        except Exception:
            raise Exception("Could not connect to camera")
        pass
        grabbed, self.framebuffer = self.camera.read()
        if not grabbed:
            print("Frame capture failed")

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        if video_filename is None:
            video_filename = "logs/objtracker" + time.strftime("%Y%m%d-%H%M%S") + ".avi" # save videos to unique files.
        
        print(video_filename)
        self.video_out = cv2.VideoWriter(video_filename, fourcc, self.FRAME_RATE, (640, 480))
        assert(self.video_out.isOpened())
        self.finalVideoOut = cv2.VideoWriter(finalVideoOut, fourcc, self.FRAME_RATE, (640, 480))

    def __del__(self):
        self.camera.release()
        self.video_out.release()

    def track_ball(self):
        self.ball_in_frame = False

        (grabbed, frame) = self.camera.read()

        if grabbed:
            cv2.waitKey(1)
            self.video_out.write(frame) # write raw video frame to the logging file for later examination.
        else:
            print("Frame capture failed")

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) # this was necessary to move from RGB to BGR color space for the underlying CSV code, but then we realized theres a BGR2HSV conversion thingy
        #cv2.imshow("hsv", hsv)
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if self.testing:
            cv2.imshow("hsv", hsv) # don't really need to display the hsv image in the things.
        mask = cv2.inRange(hsv, self.LOWER, self.UPPER)
        if self.testing:
            cv2.imshow("mask1", mask)
        mask = cv2.dilate(mask, None, iterations=2)
        if self.testing:
            cv2.imshow("mask2", mask)
        mask = cv2.erode(mask, None, iterations=2)
        if self.testing:
            cv2.imshow("mask3", mask)
        # mask = cv2.dilate(mask, None, iterations=2) # testing with masks determined Dilate then Erode is better for usage in the SDELC, Utah might be different
        if self.testing:
            cv2.imshow("mask4", mask)
        contour = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

        radius = 0
        if len(contour) > 0:
            self.ball_in_frame = True
            largest_contour = max(contour, key=cv2.contourArea)
            ((x, y), self.radius) = cv2.minEnclosingCircle(largest_contour)
            radius = self.radius
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                return False, (0,0), 0 # sometimes M["m00"] is 0 when we have the ball in frame, this is to prevent crashing due to div by 0 errors.
            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > self.MIN_RADIUS:
            cv2.circle(frame, (int(x), int(y)), int(self.radius), (0, 255, 255), 2)
            cv2.circle(frame, self.center, 5, (0, 0, 255), -1)

            #if self.testing:
            #    cv2.imshow("Auto", frame)

            # self.video_out.write(frame) # leave this commented for early testing so we can get the raw video feed to run through later for more testing.

            return self.ball_in_frame, self.center, self.radius
        if not self.firstRun: # using this to pull a sample image to grab RGB values for the tennis ball from gimp.
            self.firstRun = 1
            cv2.imwrite('test.png', frame)
        #if self.testing:
        #    cv2.imshow("Auto", frame)
        self.finalVideoOut.write(frame)
        return self.ball_in_frame, (0, 0), 0


if __name__ == '__main__':
    tracker = ObjectTracker(True, "../logs/objtracker" + time.strftime("%Y%m%d-%H%M%S") + ".avi")
    tracker.video_filename = "../logs/objtracker" + time.strftime("%Y%m%d-%H%M%S") + ".avi" # save videos to unique files.
    while True:
        ball_in_frame, center, radius = tracker.track_ball()
        if ball_in_frame:
            if radius == 0:
                radius = 1
            print("Ball found at %s, distance %s" % (center, 1.0/radius))
        else:
            print("No ball found")
