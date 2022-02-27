import logging
import core
import algorithms
import cv2
import asyncio
import open3d as o3d


# Dict to hold the obstacle info
obstacle_dict = {"detected": False, "angle": None, "distance": None}

# Define the number of cores to use for each task.
CONVERSION_CORES = 1
DETECTION_CORES = 1
# Define multiprocessing toggle.
MULTIPROC_MODE = True
# Define a constant for toggling between the different methods of obstacle detection.
USE_NEW_METHOD = True
DISPLAY = False


async def async_obstacle_detector():
    """
    Async function to find obstacles
    """
    # Declare objects
    logger = logging.getLogger(__name__)
    ObstacleIgnorance = algorithms.new_obstacle_detector.ObstacleDetector()

    # Create visualizer for masking the object and floor onto the reg_img.
    if DISPLAY:
        vis2 = o3d.visualization.Visualizer()
        vis2.create_window("annotation mask", width=1386, height=752, visible=False)
    else:
        vis2 = None

    while True:
        reg_img = core.vision.camera_handler.grab_regular()

        # Determine which object detection method to use.
        if USE_NEW_METHOD:
            # Get point cloud from zed cam.
            zed_point_cloud = core.vision.camera_handler.grab_point_cloud()

            # Detect obstacles.
            (processed_pcd, inlier_clouds, object_bounding_boxes,) = ObstacleIgnorance.detect_obstacle(
                zed_point_cloud,
                conversion_procs=CONVERSION_CORES,
                detection_procs=DETECTION_CORES,
                multiproc_mode=MULTIPROC_MODE,
            )

            # Track a specific obstacle. (closest one)
            angle, distance, closest_box = ObstacleIgnorance.track_obstacle(
                object_bounding_boxes, reg_img, vis2, annotate_object=True, annotate_floor=True
            )

            # If obstacle has been detected store its info.
            if closest_box is not None:
                # Update the current obstacle info
                obstacle_dict["detected"] = True
                obstacle_dict["angle"] = angle
                obstacle_dict["distance"] = distance / 1000
            else:
                # Update the current obstacle info
                obstacle_dict["detected"] = False
                obstacle_dict["angle"] = None
                obstacle_dict["distance"] = None
        else:
            depth_matrix = core.vision.camera_handler.grab_depth_data()
            mask, lower = algorithms.obstacle_detector.get_floor_mask(
                reg_img, int(reg_img.shape[1] / 2), int(reg_img.shape[0] / 2)
            )
            depth_matrix = cv2.bitwise_and(depth_matrix, depth_matrix, mask=mask)
            obstacle = algorithms.obstacle_detector.detect_obstacle(depth_matrix, 1, 4)

            # Resize the image so it matches the dimensions of the depth data
            depth_img_x, depth_img_y = core.vision.camera_handler.get_depth_res()
            reg_img = cv2.resize(reg_img, (depth_img_x, depth_img_y))

            if len(obstacle) > 0:
                # Track the obstacle in the depth matrix
                angle, distance, _ = algorithms.obstacle_detector.track_obstacle(
                    depth_matrix, obstacle, True, reg_img, True
                )
                # Update the current obstacle info
                obstacle_dict["detected"] = True
                obstacle_dict["angle"] = angle
                obstacle_dict["distance"] = distance / 1000
            else:
                # Update the current obstacle info
                obstacle_dict["detected"] = False
                obstacle_dict["angle"] = None
                obstacle_dict["distance"] = None

        if obstacle_dict["detected"]:
            logger.info(
                f"Object detected at a distance of {obstacle_dict['distance']} meters and {obstacle_dict['angle']} degrees from camera center!"
            )
        core.vision.feed_handler.handle_frame("obstacle", reg_img)
        await asyncio.sleep(1 / core.vision.camera_handler.get_fps())


def is_obstacle():
    """
    Returns whether there is an obstacle

    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    return obstacle_dict["detected"]


def get_angle():
    return obstacle_dict["angle"]


def get_distance():
    return obstacle_dict["distance"]
