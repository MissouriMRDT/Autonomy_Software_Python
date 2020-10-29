import cv2
import numpy as np
import time


###################################
widthImg=1920
heightImg =1080

FRAME_RATE = 10
#####################################

 
def preProcessing(img):
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray,(5,5),1)
    imgCanny = cv2.Canny(imgBlur,200,200)
    kernel = np.ones((5,5))
    imgDial = cv2.dilate(imgCanny,kernel,iterations=2)
    imgThres = cv2.erode(imgDial,kernel,iterations=1)
    return imgThres
 
def getContours(img):
    biggest = np.array([])
    maxArea = 0
    contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area>5000:
            #cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
            if area >maxArea and len(approx) == 4:
                biggest = approx
                maxArea = area
    cv2.drawContours(imgContour, biggest, -1, (255, 0, 0), 20)
    return biggest
 
def reorder (myPoints):
    myPoints = myPoints.reshape((4,2))
    myPointsNew = np.zeros((4,1,2),np.int32)
    add = myPoints.sum(1)
    #print("add", add)
    myPointsNew[0] = myPoints[np.argmin(add)]
    myPointsNew[3] = myPoints[np.argmax(add)]
    diff = np.diff(myPoints,axis=1)
    myPointsNew[1]= myPoints[np.argmin(diff)]
    myPointsNew[2] = myPoints[np.argmax(diff)]
    #print("NewPoints",myPointsNew)
    return myPointsNew
 
def getWarp(img,biggest):
    biggest = reorder(biggest)
    pts1 = np.float32(biggest)
    pts2 = np.float32([[0, 0], [widthImg, 0], [0, heightImg], [widthImg, heightImg]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgOutput = cv2.warpPerspective(img, matrix, (widthImg, heightImg))
 
    imgCropped = imgOutput[20:imgOutput.shape[0]-20,20:imgOutput.shape[1]-20]
    imgCropped = cv2.resize(imgCropped,(widthImg,heightImg))
 
    return imgCropped
 
 
def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

usingZed = False

# Set video parameters
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
video_filename = "logs/objtracker" + time.strftime("%Y%m%d-%H%M%S") # save videos to unique files.
video_out_left = cv2.VideoWriter(video_filename + "_left.avi", fourcc, FRAME_RATE, (1920, 1080))

if usingZed:
    import pyzed.s1 as s1

    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    input_type = sl.InputType()
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)

    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width /2
    image_size.height = image_size.height /2

    # Declare your sl.Mat matrices
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    while True:
        key = ' '
        while key != 113 :
            err = zed.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS :
                # Retrieve the left image, depth image in the half-resolution
                zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
                # Retrieve the RGBA point cloud in half resolution

                # To recover data from sl.Mat to use it with opencv, use the get_data() method
                # It returns a numpy array that can be used as a matrix with opencv
                img = image_zed.get_data()

                img = cv2.resize(img,(widthImg,heightImg))
                imgContour = img.copy()
            
                imgThres = preProcessing(img)
                biggest = getContours(imgThres)
                if biggest.size !=0:
                    imgWarped=getWarp(img,biggest)
                    # imageArray = ([img,imgThres],
                    #           [imgContour,imgWarped])
                    imageArray = ([imgContour, imgWarped])
                    cv2.imshow("ImageWarped", imgWarped)
                else:
                    # imageArray = ([img, imgThres],
                    #               [img, img])
                    imageArray = ([imgContour, img])
            
                stackedImages = stackImages(0.6,imageArray)
                cv2.imshow("WorkFlow", stackedImages)
                video_out_left.write(stackedImages)
                
                key = cv2.waitKey(10)


        cv2.destroyAllWindows()
        zed.close()
else:
    cap = cv2.VideoCapture(1)
    

    while True:    
        success, img = cap.read()
        img = cv2.resize(img,(widthImg,heightImg))
        imgContour = img.copy()
    
        imgThres = preProcessing(img)
        biggest = getContours(imgThres)
        if biggest.size !=0:
            imgWarped=getWarp(img,biggest)
            # imageArray = ([img,imgThres],
            #           [imgContour,imgWarped])
            imageArray = ([imgContour, imgWarped])
            cv2.imshow("ImageWarped", imgWarped)
        else:
            # imageArray = ([img, imgThres],
            #               [img, img])
            imageArray = ([imgContour, img])
    
        stackedImages = stackImages(0.6,imageArray)
        cv2.imshow("WorkFlow", stackedImages)
        video_out_left.write(imageArray)

        c = cv2.waitKey(1) % 256

        if c == ord('a'):
            break
    video_out_left.release()
        
    