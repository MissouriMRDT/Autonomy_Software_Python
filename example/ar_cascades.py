import core.vision
import logging
import cv2
from cv2 import aruco
import numpy as np

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
    # for ALVAR tags the border is actually 2 black bits wide
    parameters = aruco.DetectorParameters_create()
    parameters.markerBorderBits = 2
    parameters.cornerRefinementMethod = 3
    parameters.errorCorrectionRate = 0.2
    while True:
        # Test grabbing the latest camera frames
        reg_img = core.vision.camera_handler.grab_regular()
        # depth_img = vision.camera_handler.grab_depth()
        tag_cascade = cv2.CascadeClassifier("cascade.xml")
        tags_imgs = list()

        gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)

        tags = tag_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in tags:
            tags_imgs.append((reg_img[y : y + h, x : x + w], w, h))
            reg_img = cv2.rectangle(reg_img, (x, y), (x + w, y + h), (255, 0, 0), 2)

        cv2.imshow("img", reg_img)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        img = aruco.drawDetectedMarkers(gray, corners, ids)
        cv2.imshow("ar", img)

        # Go through tags detected and insert them on a white background
        # run ARUCO detection on them to see if accuracy increases
        for i, tag in enumerate(tags_imgs):
            im, w, h = tag
            print(im)
            img_1 = np.zeros((1280, 720, 4), dtype=np.uint8)
            img_1.fill(255)
            margin_x = (1280 - w) // 2
            margin_y = (720 - h) // 2

            img_1[margin_y : margin_y + h, margin_x : margin_x + w] = im
            gray = cv2.cvtColor(img_1, cv2.COLOR_BGRA2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            img = aruco.drawDetectedMarkers(gray, corners, ids)
            cv2.imshow(f"img{i}", gray)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    # Run main()
    main()
