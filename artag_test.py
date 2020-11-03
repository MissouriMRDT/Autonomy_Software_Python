import numpy as np
import cv2
from cv2 import aruco
import time
import pyzed.sl as sl
import multiprocessing as mp
import pyfakewebcam

# define an empty custom dictionary with 
aruco_dict = aruco.custom_dictionary(2, 5, 1)
# add empty bytesList array to fill with 3 markers later
aruco_dict.bytesList = np.empty(shape = (2, 4, 4), dtype = np.uint8)

# add custom markers as defined by the ALVAR library
mybits = np.array([[1,1,0,1,1],[1,1,0,1,1],[1,0,1,0,1],[1,1,1,1,1],[1,1,1,1,1,]], dtype = np.uint8)
aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)

mybits_1 = np.array([[1,1,0,1,1],[1,1,0,1,1],[1,0,1,0,1],[0,0,1,1,0],[1,1,1,0,1,]], dtype = np.uint8)
aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits_1)

def stream_loop(pipe, num):
    camera = pyfakewebcam.FakeWebcam(f'/dev/video{num}', 640, 480)
    p_output, p_input = pipe
    p_input.close()    # We are only reading
    while True:
        image_ocv = p_output.recv()
        image_ocv = cv2.resize(image_ocv, (640, 480))
        img = cv2.cvtColor(image_ocv, cv2.COLOR_BGRA2RGB)
        camera.schedule_frame(img)

p1_output, p1_input = mp.Pipe()
p2_output, p2_input = mp.Pipe()
t1 = mp.Process(target=stream_loop, args=((p1_output,p1_input),2,))
t2 = mp.Process(target=stream_loop, args=((p2_output, p2_input),3,))
t1.daemon = True
t2.daemon = True
t1.start()
t2.start()

p1_output.close()       # We don't need output on our end
p2_output.close()       

# Create a ZED camera object
zed = sl.Camera()

# Set configuration parameters
input_type = sl.InputType()
init = sl.InitParameters(input_t=input_type)
init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init.coordinate_units = sl.UNIT.MILLIMETER
init.camera_fps = 30

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
depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

while(True):
    err = zed.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS :
        # Retrieve the left image, depth image in the half-resolution
        zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
        zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
        # Retrieve the RGBA point cloud in half resolution

        # To recover data from sl.Mat to use it with opencv, use the get_data() method
        # It returns a numpy array that can be used as a matrix with opencv
        frame = image_zed.get_data()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)


        #apply some effects to make image easy to process
        #NOTE: good chance Aruco already done this
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #for ALVAR tags the border is actually 2 black bits wide
        parameters =  aruco.DetectorParameters_create()
        parameters.markerBorderBits = 2
        parameters.cornerRefinementMethod = 3
        parameters.errorCorrectionRate = 0.2

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        #print(corners, ids)
        
        #draw bounding box and ID on the markers
        frame = aruco.drawDetectedMarkers(frame, corners, ids)
        
        # resize frame to show even on smaller screens
        #frame = cv2.resize(frame, None, fx = 1.6, fy = 1.6)
        # Display the resulting frame
        cv2.imshow('frame',frame)
        img = depth_image_zed.get_data()
        
        p1_input.send(frame)
        p2_input.send(img)

        #print("Hello")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
t1.join()