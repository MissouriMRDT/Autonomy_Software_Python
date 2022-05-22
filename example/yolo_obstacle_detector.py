import cv2
import core
import numpy as np
import algorithms
import logging
import time
import os

# Define constants.
DEBUG = True
DISPLAY = True


def main():
    # Declare objects
    ObstacleIgnorance = algorithms.yolo_obstacle_detector.ObstacleDetector(
        weights=os.path.dirname(__file__) + "/../resources/yolo_models/2022-0511/weights/best.pt",
        net_img_size=640,
        min_confidence=0.4,
    )
    logger = logging.getLogger(__name__)

    # Setup camera stream.
    if not DISPLAY:
        core.vision.feed_handler.add_feed(2, "yolo", stream_video=core.vision.STREAM_FLAG)

    while True:
        # Get other info and image stream from ZED camera.
        reg_img = core.vision.camera_handler.grab_regular()
        # Get point cloud from zed cam.
        zed_point_cloud = core.vision.camera_handler.grab_point_cloud()

        # Detect obstacles.
        objects, pred = ObstacleIgnorance.detect_obstacles(reg_img)

        # Track a specific obstacle. (closest one)
        object_angle, object_distance, object_summary, inference_time, _ = ObstacleIgnorance.track_obstacle(
            zed_point_cloud=zed_point_cloud, reg_img=reg_img
        )

        # Print console info.
        if object_distance != -1:
            logger.info(
                f"Object tracked at a distance of {object_distance / 1000} meters and {object_angle} degrees from camera center!\nTotal Objects Detected: {object_summary}Done. ({inference_time:.3f}s)"
            )

        # Show regular image in seperate window.
        if DISPLAY:
            cv2.imshow("reg", reg_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            core.vision.feed_handler.handle_frame("yolo", reg_img)

    # Close all windows.
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
