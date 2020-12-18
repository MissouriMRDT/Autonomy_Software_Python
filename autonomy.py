import algorithms
import core
import logging
import interfaces
import asyncio
import cv2
from cv2 import aruco
import numpy as np

logger = logging.getLogger(__name__)

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
        [0, 1, 1, 1, 0],
        [
            0,
            1,
            1,
            1,
            0,
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
    Main autonomy loop
    """
    logger.info("Running main autonomy loop")

    loop = asyncio.get_event_loop()
    loop.create_task(do_ar_tag())
    loop.run_until_complete(autonomy_state_loop())


async def autonomy_state_loop():
    while True:
        # Run the current state in the state machine (and handle enable/disable)
        await core.states.state_machine.run()

        logger.debug(f"Current State: {core.states.state_machine.state}")

        # Core state machine runs every X ms, to prevent unecessarily fast computation
        # sensor data is processed seperately, as that is the bulk of processing time
        await asyncio.sleep(core.constants.EVENT_LOOP_DELAY)


async def do_ar_tag():

    cap = cv2.VideoCapture("resources/ar.avi")

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret is True:
            # apply some effects to make image easy to process
            # NOTE: good chance Aruco already done this
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # for ALVAR tags the border is actually 2 black bits wide
            parameters = aruco.DetectorParameters_create()
            parameters.markerBorderBits = 2
            parameters.cornerRefinementMethod = 3
            parameters.errorCorrectionRate = 0.2

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # draw bounding box and ID on the markers
            frame = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

            # resize frame to show even on smaller screens
            frame = cv2.resize(frame, None, fx=1.6, fy=1.6)

            # Display the resulting frame
            cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        await asyncio.sleep(1 / 30)


if __name__ == "__main__":
    # Run main()
    main()
