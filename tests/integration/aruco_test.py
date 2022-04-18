import cv2
from cv2 import aruco

class Tag:
    def __init__(self, id, gps, center):
        self.id = id
        self.lat, self.long = gps
        self.cX, self.cY = center
        self.detected = 1

    def checkTag(self, id, gps):
        if id == self.id:
            self.tagSpotted()
            self.gps = gps
            return True
        else:
            return False

    def tagSpotted(self):
        self.detected += 1

    def print(self):
        print(output.format(id = self.id, detected = self.detected))

FRAMES_DETECTED = 5
cap = cv2.VideoCapture(0)
output = "Ids: {id:02d}   |   Detected: {detected:02d}"
frame_found = False
detected_tags = []

# Add Tag Function
def addTag(id, corners, gps):
    latitude, longitude = gps

    x1 = corners[0][0][0][0]  # top left x coord
    y1 = corners[0][0][0][1]  # top left y coord
    x2 = corners[0][0][1][0]  # top right x coord
    y2 = corners[0][0][3][1]  # bottom left y coord

    # Calculate the center points of the AR Tag
    cX = (x1 + x2) / 2
    cY = (y1 + y2) / 2

    detected_tags.append(Tag(id, (latitude, longitude), (cX, cY)))

while True:
    ret, reg_img = cap.read()

    # Frame Adjustments
    gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    # Capture Tags
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Add Tags to Tag Class Object
    if ids is not None:
        reg_img = aruco.drawDetectedMarkers(reg_img, corners)

        for i in ids:
            for j in i:
                if len(detected_tags) == 0:
                    addTag(j, corners, (37.951500, -91.772552))
                else:
                    found = False
                    for t in detected_tags:
                        if t.checkTag(j, (37.951500, -91.772552)):
                            found = True
                    if not found:
                        addTag(j, corners, (37.951500, -91.772552))
        
    # Print Frames Detected 5 or more times
    for t in detected_tags:
        if t.detected >= FRAMES_DETECTED:
            frame_found = True
            print (output.format(id = t.id, detected = t.detected))

    # Frame Bounding Line
    if (frame_found):
        print ("--------------------------------------------------------------------------")

    # Show Frame
    cv2.imshow('frame', reg_img)

    # Exit Test
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release Frames
cap.release()
cv2.destroyAllWindows()