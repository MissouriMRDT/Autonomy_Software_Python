import pyzed.sl as sl
import numpy as np
import cv2
from ArTagDetection import detect_markers



if __name__ == '__main__':
    # add zed camera code
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    videoname = "./ArTest.avi"
    video_out = cv2.VideoWriter(videoname, fourcc, 10, (int(1920), int(1080)))
    grabbed = False
    runtime_params = sl.RuntimeParameters()
    camera = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD1080
    err = camera.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("FAILED TO OPEN CAMERA")

    image_size = camera.get_resolution()
    left = sl.Mat(image_size.width/2, image_size.height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
    if camera.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        camera.retrieve_image(left, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU, int(1920), int(1080))
        frame = left.get_data()
        grabbed = True
    
    while grabbed:
        markers = detect_markers(frame)
        for marker in markers:
            marker.highlite_marker(frame)
        cv2.imshow("IMAGE", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        video_out.write(frame)
        if camera.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            camera.retrieve_image(left, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU, int(1920), int(1080))
            frame = left.get_data()
            grabbed = True
        else:
            grabbed = False
        #read zed again



    cv2.destroyAllWindows()
    camera.close()
    video_out.release()
