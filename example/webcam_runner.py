import time
import core.vision
import logging
import cv2
<<<<<<< HEAD
import numpy as np
import pyzed.sl as sl
=======
import matplotlib.pyplot as plt
>>>>>>> feature/simulator_depth


def main() -> None:
    """
    Main function for video stream script, tests streaming/recording camera footage
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Give the system a second to set everything up, start reading in frames
    time.sleep(1)

    while True:
        # Test grabbing the latest camera frames
        reg_img = core.vision.camera_handler.grab_regular()
        depth_data = core.vision.camera_handler.grab_depth_data()

        # Plot the depth data as a grayscale image
        plt.imshow(depth_data, cmap="gray", vmin=0, vmax=20)
        plt.draw()
        plt.pause(0.01)

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        cv2.imshow("reg", reg_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    # Run main()
    main()
