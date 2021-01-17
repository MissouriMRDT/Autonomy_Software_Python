import vision
import logging
import cv2
import numpy as np
from cv2 import aruco

# define an empty custom dictionary with
aruco_dict = aruco.custom_dictionary(2, 5, 1)
# add empty bytesList array to fill with 3 markers later
aruco_dict.bytesList = np.empty(shape=(2, 4, 4), dtype=np.uint8)

# add custom markers as defined by the ALVAR library
mybits = np.array(
    [
        [1, 1, 0, 1, 1],
        [1, 1, 0, 1, 1],
        [1, 0, 1, 0, 1],
        [1, 1, 1, 1, 1],
        [
            1,
            1,
            1,
            1,
            1,
        ],
    ],
    dtype=np.uint8,
)
aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)

mybits_1 = np.array(
    [
        [1, 1, 0, 1, 1],
        [1, 1, 0, 1, 1],
        [1, 0, 1, 0, 1],
        [0, 0, 1, 1, 0],
        [
            1,
            1,
            1,
            0,
            1,
        ],
    ],
    dtype=np.uint8,
)
aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits_1)


def main() -> None:
    """
    Main function for video stream script, tests streaming/recording zed footage
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Add feed
    vision.camera_handler.feed_handler.add_feed(2, "ar")

    while True:
        # Test grabbing the latest camera frames
        img = vision.camera_handler.grab_regular()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # for ALVAR tags the border is actually 2 black bits wide
        parameters = aruco.DetectorParameters_create()
        parameters.markerBorderBits = 2
        parameters.cornerRefinementMethod = 3
        parameters.errorCorrectionRate = 0.2

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        print(corners, ids)

        # draw bounding box and ID on the markers
        img = aruco.drawDetectedMarkers(gray, corners, ids)

        # resize frame to show even on smaller screens
        # frame = cv2.resize(frame, None, fx = 1.6, fy = 1.6)
        # Display the resulting frame
        # cv2.imshow('frame',img)

        # Stream the AR Tag video feed
        vision.camera_handler.feed_handler.handle_frame("ar", img)

        # stackedImages = cv2.resize(stackedImages, (1920, 1080))
        # stackedImage = cv2.cvtColor(stackedImages, cv2.COLOR_RGBA2RGB)

        key = cv2.waitKey(10)

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        # cv2.imshow('img', img)
        # cv2.imshow('depth', depth_img)

        for i in range(10000):
            pass

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    # Run main()
    main()
