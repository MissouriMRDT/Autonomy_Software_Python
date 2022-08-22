import cv2
from cv2 import aruco
import datetime


class Tag:
    def __init__(self, tag, gps, center):
        """
        Creates an object of Tag
        Parameters:
        -----------
            tag - the id that was found
            gps - the coordinates of the tag (lat, long)
            center - the center points of the tag
        Returns:
        --------
            None
        """

        self.id = tag
        self.lat, self.long = gps
        self.cX, self.cY = center
        self.empty = 0
        self.detected = 1

    def check_tag(self, tag, gps):
        """
        Checks to see if the tag that was spotted has already been added to the array.
        Parameters:
        -----------
            tag - the id that is being checked
            gps - the coordinates of the tag (lat, long)
        Returns:
        --------
            bool - True if detected, False if not detected
        """

        if self.id == tag:
            self.tag_spotted(gps)
            return True
        else:
            return False

    def tag_spotted(self, gps):
        """
        Increments the detected variable and updates GPS
        Parameters:
        -----------
            gps - the coordinates of the tag (lat, long)
        Returns:
        --------
            None
        """

        self.detected += 1
        self.lat, self.long = gps

    def reset_spotted(self):
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

        print(output.format(id=self.id, detected=self.detected))


cap = cv2.imread(r'../../resources/artags.png')
frame_found = False
FRAMES_DETECTED = 5
detected_tags = []
blank_frames = 0
output = "Ids: {id:02d}   |   Detected: {detected:02d}"


def get_gps():
    """
    Finds the GPS Coordinates for the Aruco Tag
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


def add_tag(tag, corner):
    """
    Creates a new Object of Tag in the detected_tags list
    Parameters:
    -----------
        tag - the id of the aruco tag
        corner - the four corner points of the aruco tag for distance calculations
    Returns:
    --------
        None
    """

    latitude, longitude = get_gps()

    x1 = corner[0][0][0][0]  # top left x coord
    y1 = corner[0][0][0][1]  # top left y coord
    x2 = corner[0][0][1][0]  # top right x coord
    y2 = corner[0][0][3][1]  # bottom left y coord

    # Calculate the center points of the AR Tag
    cX = (x1 + x2) / 2
    cY = (y1 + y2) / 2

    # Adds an object of Tag to detected_tags
    detected_tags.append(Tag(tag, (latitude, longitude), (cX, cY)))


while True:
    # Webcam Frame
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

        # Create a list of all ids seen in frame
        frameTags = []
        for i in ids:
            for j in i:
                frameTags.append(j)

        # Loops through ids found and checks adds them to the list accordingly.
        for i in ids:
            for j in i:
                if len(detected_tags) == 0:
                    add_tag(j, corners)
                else:
                    found = False
                    for t in detected_tags:
                        if t.checkTag(j, get_gps()):
                            found = True
                    if not found:
                        add_tag(j, corners)

        # Loops through ids found on current leg and checks if they were in current frame.
        for t in detected_tags:
            if t.id not in frameTags:
                t.empty += 1

    # Checks for Blank Frames and resets counter if >= FRAMES_DETECTED
    else:
        for t in detected_tags:
            t.empty += 1
            # if t.empty >= FRAMES_DETECTED:
            #     t.resetSpotted()

    # Frame Bounding Line
    if frame_found and ids is not None:
        ct = datetime.datetime.now()
        print("Timestamp: ", ct)
        print("--------------------------------------------------------------------------")

    # Print Frames Detected 5 or more times
    for t in detected_tags:
        if t.detected >= FRAMES_DETECTED and ids is not None:
            frame_found = True
            print(output.format(id=t.id, detected=t.detected))

    # Frame Bounding Line
    if frame_found and ids is not None:
        print("--------------------------------------------------------------------------")
        print()

    # Show Frame
    cv2.imshow('frame', reg_img)

    # Exit Test
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release Frames
cap.release()
cv2.destroyAllWindows()
