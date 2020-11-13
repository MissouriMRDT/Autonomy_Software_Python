import core
import logging
import pyzed.sl as sl
import time
import cv2
import threading, queue

q = queue.Queue()


def setup_zed():
    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    input_type = sl.InputType()
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD720
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)

    return zed


def thread_func(zed):
    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width /2
    image_size.height = image_size.height /2

    # Declare your sl.Mat matrices
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    while True:
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS :
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            img = image_zed.get_data()
            cv2.imshow("frame", img)
            q.put(img)
            key = cv2.waitKey(10)

def main() -> None:
    '''
    Main function for example script, tests geomath code
    '''
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")
    core.video_handler = core.VideoHandler()
    # Add video streams to handler
    core.video_handler.add_feed(2, "regular", True, False)
    core.video_handler.add_feed(3, "depth")
    
    zed = setup_zed()
    thread_obj = threading.Thread(target=thread_func, args=(zed,))
    thread_obj.start()
    
    try:
        while True:
            if not q.empty():
                data = q.get(False)  
                core.video_handler.handle_frame("regular", data)
    except KeyboardInterrupt:
        core.video_handler.close()
        
    thread_obj.join()
if __name__ == "__main__":
    # Run main()
    main()
