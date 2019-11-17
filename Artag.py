import cv2

cap = cv2.VideoCapture(0)
#define the dictionary (matrix size) of each ar tag size
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
dictionary2 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
dictionary3 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
dictionary4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
dictionary5 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
dictionary6 = cv2.aruco.custom_dictionary(9, 9, 9)


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    res = cv2.aruco.detectMarkers(gray,dictionary)
    res2 = cv2.aruco.detectMarkers(gray,dictionary2)
    res3 = cv2.aruco.detectMarkers(gray,dictionary3)
    res4 = cv2.aruco.detectMarkers(gray,dictionary4)
    res5 = cv2.aruco.detectMarkers(gray,dictionary5)
    res6 = cv2.aruco.detectMarkers(gray,dictionary6)
    
#   print(res[0],res[1],len(res[2]))

    if len(res[0]) > 0:
        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])
    if len(res2[0]) > 0:
        cv2.aruco.drawDetectedMarkers(gray,res2[0],res2[1])
    if len(res3[0]) > 0:
        cv2.aruco.drawDetectedMarkers(gray,res3[0],res3[1])
    if len(res4[0]) > 0:    
        cv2.aruco.drawDetectedMarkers(gray,res4[0],res4[1])
    if len(res5[0]) > 0:
        cv2.aruco.drawDetectedMarkers(gray,res5[0],res5[1])
    if len(res6[0]) > 0:
        cv2.aruco.drawDetectedMarkers(gray,res6[0],res6[1])
        
    # Display the resulting frame - change this to read in an image (Nov 23rd)
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
# What to return? Heading?
