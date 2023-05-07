import cv2
from cv2 import aruco
import numpy as np
import glob

# Load previously saved data
with np.load('camera_calib.npz') as X:
    mtx, dist, rvecs, tvecs = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]


# define an empty custom dictionary with 
aruco_dict = aruco.custom_dictionary(2, 5, 1)
# add empty bytesList array to fill with 3 markers later
aruco_dict.bytesList = np.empty(shape = (2, 4, 4), dtype = np.uint8)

# add custom markers as defined by the ALVAR library
mybits = np.array([[1,1,0,1,1],[1,1,0,1,1],[1,0,1,0,1],[1,1,1,1,1],[1,1,1,1,1,]], dtype = np.uint8)
aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)

mybits_1 = np.array([[1,1,0,1,1],[1,1,0,1,1],[1,0,1,0,1],[0,0,1,1,0],[1,1,1,0,1,]], dtype = np.uint8)
aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits_1)

# open video capture from (first) webcam
# can also be used with video files
cap = cv2.VideoCapture(2)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret == True:
        #apply some effects to make image easy to process
        #NOTE: good chance Aruco already done this
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #for ALVAR tags the border is actually 2 black bits wide
        parameters =  aruco.DetectorParameters_create()
        parameters.markerBorderBits = 2
        #parameters.cornerRefinementMethod = 3
        #parameters.errorCorrectionRate = 0.2

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        #draw bounding box and ID on the markers
        frame = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.2, mtx, dist, None, None)

        #if some tags were detected, draw axis on them
        if rvecs is not None:
            for i in range(len(rvecs)):
                frame = aruco.drawAxis(frame.copy(), mtx, dist, rvecs[i], tvecs[i], 0.1)
                #also print out each translation vector, units should be in cm
                print(tvecs[i])
        
        # resize frame to show even on smaller screens
        frame = cv2.resize(frame, None, fx = 1.6, fy = 1.6)
        # Display the resulting frame
        cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

