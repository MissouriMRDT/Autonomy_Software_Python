import cv2
import core
import numpy as np
import algorithms
import time
from core.vision import camera
import open3d as o3d


# Define depth img/data so we can use them for OpenCV window callbacks
depth_matrix = None
# Define the number of cores to use for each task.
conversion_cores = 5
detection_cores = 5


MULTIPROC_MODE = True
DISPLAY = True


def main():
    # Declare objects
    ObstacleIgnorance = algorithms.new_obstacle_detector.ObstacleDetector()
    # Declare point clouds.
    processed_pcd = o3d.geometry.PointCloud()
    object_bounding_boxes = []
    inlier_clouds = []
    # Declare other instance variables.
    camera_position = None

    # Setup Open3d and visualization.
    # o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    vis = o3d.visualization.Visualizer()
    vis.create_window("point cloud objects")
    # Create visualizer for masking the object and floor onto the reg_img.
    vis2 = o3d.visualization.Visualizer()
    vis2.create_window("annotation mask", width=1386, height=752)

    # Enable callbacks on depth window, can be used to display the depth at pixel clicked on
    if DISPLAY:
        cv2.namedWindow("reg")
    else:
        core.vision.feed_handler.add_feed(2, "regular", stream_video=core.vision.STREAM_FLAG)

    while True:
        # Get other info and image stream from ZED camera.
        reg_img = core.vision.camera_handler.grab_regular()
        # Get point cloud from zed cam.
        zed_point_cloud = core.vision.camera_handler.grab_point_cloud()
        # Resize the image so it matches the dimensions of the depth data
        depth_img_x, depth_img_y = core.vision.camera_handler.get_depth_res()
        reg_img = cv2.resize(reg_img, (depth_img_x, depth_img_y))

        # Detect obstacles.
        (processed_pcd, inlier_clouds, object_bounding_boxes,) = ObstacleIgnorance.detect_obstacle(
            zed_point_cloud,
            conversion_procs=conversion_cores,
            detection_procs=detection_cores,
            multiproc_mode=MULTIPROC_MODE,
        )

        # Track a specific obstacle. (closest one)
        angle, distance, closest_box = ObstacleIgnorance.track_obstacle(
            object_bounding_boxes, reg_img, vis2, annotate_object=True, remove_floor=True
        )

        # Print console info.
        print("Object angle", angle)
        print("Object distance", distance)

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        if DISPLAY:
            # Update visualizer point clouds.
            if len(np.asarray(processed_pcd.points)) != 0:
                # Add point clouds.
                vis.add_geometry(processed_pcd)
                for plane in inlier_clouds:
                    vis.add_geometry(plane)
                vis.add_geometry(closest_box)
                # Store current visualizer camera position. (Its not retained everytime we update geometries)
                if camera_position is not None:
                    vis.get_view_control().convert_from_pinhole_camera_parameters(camera_position)
                # Update window.
                vis.update_renderer()
                vis.poll_events()
                # Restore camera position.
                camera_position = vis.get_view_control().convert_to_pinhole_camera_parameters()
                # Remove old geometries.
                vis.remove_geometry(processed_pcd)
                for plane in inlier_clouds:
                    vis.remove_geometry(plane)
                vis.remove_geometry(closest_box)

            # Show regular image in seperate window.
            cv2.imshow("reg", reg_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            core.vision.feed_handler.handle_frame("regular", reg_img)
            time.sleep(1 / 30)


if __name__ == "__main__":
    main()
