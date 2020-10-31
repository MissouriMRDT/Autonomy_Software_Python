import time
import pyfakewebcam
import numpy as np
import pyzed.sl as sl
import os
import cv2
from queue import Queue
import multiprocessing as mp
from threading import Thread

# Create two fake /dev/video/ devices
#os.system("modprobe v4l2loopback devices=2")
#modprobe v4l2loopback devices=2 video_nr=10 video_nr=2 card_label="OBS Cam" exclusive_caps=1

def stream_frames(q, num):
    camera = pyfakewebcam.FakeWebcam(f'/dev/video{num}', 640, 480)
    while True:
        if not q.empty():
            image_ocv = q.get()
            image_ocv = cv2.resize(image_ocv, (640, 480))
            img = cv2.cvtColor(image_ocv, cv2.COLOR_BGRA2RGB)
            camera.schedule_frame(img)
        #time.sleep(0.05)



def main() :

    # Create a ZED camera object
    zed = sl.Camera()

    #camera1 = pyfakewebcam.FakeWebcam('/dev/video2', 640, 480)
    #camera2 = pyfakewebcam.FakeWebcam('/dev/video10', 640, 480)

    # Set configuration parameters
    input_type = sl.InputType()
   
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD720
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER
   #init.camera_fps = 30


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

    # Declare your sl.Mat matrices
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    q1 = mp.Queue()
    q2 = mp.Queue()
    t1 = mp.Process(target=stream_frames, args=(q1,2,))
    t2 = mp.Process(target=stream_frames, args=(q2,10,))
    t1.daemon = True
    t2.daemon = True
    t1.start()
    t2.start()

    key = ' '
    while key != 113 :
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS :
            # Retrieve the left image, depth image in the half-resolution
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            image_ocv = image_zed.get_data()
            q1.put(image_ocv)
            # image_ocv = cv2.resize(image_ocv, (640, 480))
            # img = cv2.cvtColor(image_ocv, cv2.COLOR_BGRA2RGB)
            # camera1.schedule_frame(img)


            image_depth = depth_image_zed.get_data()
            q2.put(image_depth)
            #image_depth = cv2.resize(image_depth, (640, 480))
            # img = cv2.cvtColor(image_depth, cv2.COLOR_BGRA2RGB)
            # camera2.schedule_frame(img)
            #time.sleep(1/30)
            
    cv2.destroyAllWindows()
    zed.close()
    t1.join()
    t2.join()

    print("\nFINISH")

if __name__ == "__main__":
    main()