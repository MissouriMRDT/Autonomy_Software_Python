import cv2
from cv2 import aruco
import datetime;

class Tag:
    def __init__(self, id, gps, center):
        """
        Creates an object of Tag
        Parameters:
        -----------
            id - the tag that was found
            gps - the coordinates of the tag found
            center - the center points of the tag
        Returns:
        --------
            None
        """

        self.id = id
        self.lat, self.long = gps
        self.cX, self.cY = center
        self.detected = 1

    def checkTag(self, id, gps):
        """
        Checks to see if the tag that was spotted has already been added to the array.
        Parameters:
        -----------
            id - the tag that is being checked
            gps - the coordinates of the tag being checked
        Returns:
        --------
            bool - True if detected, False if not detected
        """

        if id == self.id:
            self.tagSpotted(gps)
            return True
        else:
            return False

    def tagSpotted(self, gps):
        """
        Increments the detected variable and updates GPS
        Parameters:
        -----------
            gps - the coordinates of the tag
        Returns:
        --------
            None
        """

        self.detected += 1
        self.gps = gps

    def resetSpotted(self):
        """
        Resets the tags detected counter
        Parameters:
        -----------
            None
        Returns:
        --------
            None
        """

        self.detected = 0

    def print(self):
        """
        Prints the id and the number of times it was detected
        Parameters:
        -----------
            None
        Returns:
        --------
            None
        """

        print(output.format(id = self.id, detected = self.detected))

cap = cv2.VideoCapture(0)
output = "Ids: {id:02d}   |   Detected: {detected:02d}"
frame_found = False
FRAMES_DETECTED = 5
detected_tags = []
blank_frames = 0

def getGPS():
    """
    Finds the GPS Cordinates for the Aruco Tag
    Parameters:
    -----------
        None
    Returns:
    --------
        latitude - the latitude of the tag
        longitude - the longitude of the tag
    """

    latitude = 37.951500
    longitude = -91.772552

    return latitude, longitude

def addTag(id, corners):
    """
    Creates a new Object of Tag in the detected_tags list
    Parameters:
    -----------
        id - the id of the aruco tag
        corners - the four corner points of the aruco tag for distance calculations
    Returns:
    --------
        None
    """

    latitude, longitude = getGPS()

    x1 = corners[0][0][0][0]  # top left x coord
    y1 = corners[0][0][0][1]  # top left y coord
    x2 = corners[0][0][1][0]  # top right x coord
    y2 = corners[0][0][3][1]  # bottom left y coord

    # Calculate the center points of the AR Tag
    cX = (x1 + x2) / 2
    cY = (y1 + y2) / 2

    # Adds an object of Tag to detected_tags
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
        blank_frames = 0
        reg_img = aruco.drawDetectedMarkers(reg_img, corners)

        # Loops through ids found and checks adds them to the list accordingly.
        for i in ids:
            for j in i:
                if len(detected_tags) == 0:
                    addTag(j, corners)
                else:
                    found = False
                    for t in detected_tags:
                        if t.checkTag(j, getGPS()):
                            found = True
                    if not found:
                        addTag(j, corners)
    
    # Checks for Blank Frames and resets counter if >= FRAMES_DETECTED
    else:
        blank_frames += 1
        if blank_frames >= FRAMES_DETECTED:
            for t in detected_tags:
                t.resetSpotted()
        
    # Frame Bounding Line
    if frame_found and ids is not None:
        ct = datetime.datetime.now()
        print("Timestamp: ", ct)
        print ("--------------------------------------------------------------------------")
    
    # Print Frames Detected 5 or more times
    for t in detected_tags:
        if t.detected >= FRAMES_DETECTED and ids is not None:
            frame_found = True
            print (output.format(id = t.id, detected = t.detected))

    # Frame Bounding Line
    if frame_found and ids is not None:
        print ("--------------------------------------------------------------------------")
        print()

    # Show Frame
    cv2.imshow('frame', reg_img)

    # Exit Test
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release Frames
cap.release()
cv2.destroyAllWindows()