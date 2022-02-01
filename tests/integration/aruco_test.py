import numpy as np
import cv2
from cv2 import aruco

tags = []

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, reg_img = cap.read()
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    #print(parameters)

    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # print(corners)

    #It's working.
    # my problem was that the cellphone put black all around it. The algorithm
    # depends very much upon finding rectangular black blobs

    reg_img = aruco.drawDetectedMarkers(reg_img, corners)

    if ids is not None:
        for i in range(0, len(ids) - 1):
            x1 = corners[i][0][0][0] #top left x coord
            y1 = corners[i][0][0][1] # top left y coord
            x2 = corners[i][0][1][0] # top right x coord
            y2 = corners[i][0][3][1] # bottom left y coord

            # Calculate the center points of the AR Tag
            cX = (x1 + x2) / 2
            cY = (y1 + y2) / 2

            # Find the distance/angle of said center pixels
            print("(" + cX + "," + cY + ")")

    if len(corners) != 0:
        x = corners[0][0][0][0]
        y = corners[0][0][0][1]
        w = corners[0][0][1][0] - corners[0][0][0][0]
        h = corners[0][0][3][1] - corners[0][0][0][1]

    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame', reg_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()