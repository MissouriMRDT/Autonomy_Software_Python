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
    parameters.polygonalApproxAccuracyRate = 0.08
    # cap = cv2.VideoCapture("algorithms/ar3.avi")
    while True:
        # Test grabbing the latest camera frames
        reg_img = core.vision.camera_handler.grab_regular()
        # ret, reg_img = cap.read()
        # depth_img = vision.camera_handler.grab_depth()
        tag_cascade = cv2.CascadeClassifier("cascade.xml")
        tags_imgs = list()
        print(reg_img.shape)
        gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)

        tags = tag_cascade.detectMultiScale(gray, 1.05, 5, 0, (30, 30))

        for (x, y, w, h) in tags:
            tags_imgs.append((reg_img.copy()[y : y + h, x : x + w], x, y, w, h))
            rect_img = cv2.rectangle(reg_img.copy(), (x, y), (x + w, y + h), (255, 0, 0), 2)

        cv2.imshow("img", rect_img)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        img = aruco.drawDetectedMarkers(gray, corners, ids)
        cv2.imshow("ar", img)

        # Go through tags detected and insert them on a white background
        # run ARUCO detection on them to see if accuracy increases
        for i, tag in enumerate(tags_imgs):
            im, x, y, w, h = tag
            print(w, h)
            img_1 = np.zeros((720, 1280), dtype=np.uint8)
            img_1.fill(255)

            im = cv2.cvtColor(im, cv2.COLOR_BGRA2GRAY)
            ret, im = cv2.threshold(im, 60, 255, cv2.THRESH_BINARY)
            img_1[y : y + h, x : x + w] = im
            corners, ids, rejectedImgPoints = aruco.detectMarkers(img_1, aruco_dict, parameters=parameters)
            img_1 = aruco.drawDetectedMarkers(img_1, corners, ids)
            cv2.imshow(f"detect{i}", img_1)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    # Run main()
    main()
