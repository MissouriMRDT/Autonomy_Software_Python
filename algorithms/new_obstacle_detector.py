import numpy as np
import cv2
import core
import math
import heapq
import time
import open3d as o3d
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from multiprocessing import Queue, Process
from collections import deque


class DummyTask:
    """
    This is a container class for methods when multiprocessing is disabled.
    """

    def __init__(self, data):
        self.data = data

    def ready(self):
        return True

    def get(self):
        return self.data


def pickle_serialize(point_cloud):
    """
    This method is necessary when passing non-picklable point cloud objects
    to other python processes.

    Parameters:
    -----------
        point_cloud - The Open3D point cloud object to be unpacked.

    Returns:
    --------
        [points, colors] - A 3d numpy array containing the cloud's points at index 0
                        and its colors at index 1.
    """
    # Unpack points and colors from cloud
    points = np.asarray(point_cloud.points)
    colors = np.asarray(point_cloud.colors)

    return [points, colors]


def pickle_deserialize(cloud_data):
    """
    This method repacks the array from pickle_serialize into an Open3D point cloud.

    Parameters:
    -----------
        [points, colors] - A 3d numpy array containing the cloud's points at index 0
                        and its colors at index 1.

    Returns:
    --------
        point_cloud - The repacked Open3d point cloud object.
    """
    # Get points and colors
    points = cloud_data[0]
    colors = cloud_data[1]
    # Repack into Open3d PointCloud object.
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    return point_cloud


def convert_zed_cloud_to_o3d_cloud(zed_point_cloud):
    """
    This method provides an easy, hassle free way of converting the point cloud returned from the ZED
    API into something usable by the Open3D library.

    ZED CAM PCD STRUCTURE = [[[x, y, z, rgba]]]
    O3D PCD STRUCTURE = [[x, y, z]]

    Parameters:
    -----------
        zed_point_cloud - The point cloud numpy array object returned by the ZED API get_data() call.

    Returns:
    --------
        o3d.geometry.PointCloud() - The point cloud object for use with Open3D.
    """
    # Create instance variables.
    pcd = o3d.geometry.PointCloud()
    pts = zed_point_cloud
    # Converting everything to float64 has a HUGE FREAKING performance improvement. I guess silicon just likes 64-bit numbers.
    pts = pts.astype(np.float64)

    # Cut off color value from each point.
    pts = pts[:, :, :-1]
    # Cut off 2 dimensions from the array (x pixel, y pixel)
    pts = np.reshape(pts, (-1, 3))
    # Remove nans and infs.
    # pts = pts[np.isfinite(pts[:, 0])]

    # Assign the restructured and filtered pts array to the PointCloud.
    pcd.points = o3d.utility.Vector3dVector(pts)
    # Remove nans and infs using built in open3d function.
    pcd.remove_non_finite_points()

    return pcd


def down_sample_o3d_cloud(o3d_point_cloud, down_sample_factor=4):
    """
    This method provides an easy way to downscale an Open3D point cloud for easier/faster processing.

    Parameters:
    -----------
        o3d_point_cloud - The Open3D point cloud object.
        down_sample_factor - The factor at which to downscale. (2 = half, 4 = quarter)

    Returns:
    --------
        o3d.geometry.PointCloud() - The down sampled point cloud object for use with Open3D.
    """
    return o3d_point_cloud.uniform_down_sample(down_sample_factor)


def RANSAC_plane_detection(o3d_point_cloud, num_of_planes=1, distance_threshold=50, ransac_n=3, num_iterations=300):
    """
    Use RANdom SAmple Consensus algorithm to iteratively find planes based off of straight point trends.
    YOU WILL NEED TO ADJUST PARAMS TO PERFECT. (don't worry it's easy)

    Parameters:
    -----------
        o3d_point_cloud - The Open3D point cloud object.
        num_of_planes - The number of planes to detect and remove from the point cloud.
        distance_threshold - The distance threshold from the plane to consider a point an inlier or outlier.
        ransac_n - The number of sampled points selected. (3 because we want a plane)
        num_iterations - higher = more accurate, but slower.

    Returns:
    --------
        outlier_cloud - o3d point cloud with detected planes removed.
        inlier_clouds[] - an array of point clouds containing the detected planes.
    """
    # Initialize instance variables.
    inlier_clouds = []
    outlier_cloud = o3d.geometry.PointCloud()
    # Remove n number of planes.
    for i in range(num_of_planes):
        # Check if the point cloud is empty.
        if not (o3d_point_cloud.is_empty()) and o3d_point_cloud.has_points():
            # Use RANSAC method to estimate planes.
            plane_model, inliers = o3d_point_cloud.segment_plane(distance_threshold, ransac_n, num_iterations)
            # Get planes inliers and outliers.
            inlier_cloud = o3d_point_cloud.select_by_index(inliers)
            outlier_cloud = o3d_point_cloud.select_by_index(inliers, invert=True)
            # Color the different planes gray (RGB range from 0 to 1)
            inlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])
            # outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])

            # Store current detected plane.
            inlier_clouds.append(inlier_cloud)
            # Set outlier cloud (cloud without plane) to normal point cloud.
            o3d_point_cloud = outlier_cloud

    return outlier_cloud, inlier_clouds


def DBSCAN_euclidean_clustering(o3d_point_cloud, growing_radius=0.05, min_points=50, print_progress=False):
    """
    Use the Density-Based Spatial Clustering of Applications with Noise algorithm to guess potential objects
    by analyzing point patterns. This algorithm returns a list of points with a 4th value attached. The fourth
    value is the object identifier. It starts at 0 and goes up. Any points that don't fall within
    a cluster are marked with -1 and are considered noise.

    For your consideration: If this ends up sucking, then try the DBSCAN from scikit-learn.

    Parameters:
    -----------
        o3d_point_cloud - The Open3D point cloud object.
        growing_radius - The radius to search in for another point from a point.
        min_points - The minimum number of points before a group is considered a cluster.
        print_progress - Print a info bar of the algorithms progress to the console.

    Returns:
    --------
        o3d.geometry.PointCloud() - The point cloud with each detected cluster colored differently. (For your viewing convenience!)
        (DICTIONARY) objects - A dictionary of seperated point from the original point cloud, with each clusters
                                identifier being the dictionary key. ({"1": [[LIST], [OF], [POINTS]]})
    """
    # Create instance variables.
    objects = {}

    # Check if the point cloud is empty.
    if not (o3d_point_cloud.is_empty()) and o3d_point_cloud.has_points():
        # Use DBSCAN method to estimate and seperate objects in the scene.
        test = np.asarray(o3d_point_cloud.points)
        dbscan = DBSCAN(eps=growing_radius, min_samples=min_points).fit(test)
        labels = dbscan.labels_
        # labels = np.array(o3d_point_cloud.cluster_dbscan(growing_radius, min_points, print_progress))
        # Check if labels is empty.
        if len(labels) > 0:
            max_label = labels.max()
            # Print number of clusters found.
            if print_progress:
                print(f"point cloud has {max_label + 1} clusters")

            # Create a new array with colors values at the same index as its point in pcd array. (This array store only colors)
            clusters = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
            # Points at index values that the algorithm thinks are not important are marked -1.
            clusters[labels < 0] = 0
            # Set the point clouds colors.
            o3d_point_cloud.colors = o3d.utility.Vector3dVector(clusters[:, :-1])

        # Seperate each of the objects based on their labels value.
        for i, identifier in enumerate(labels):
            # Get corresponding point coordinates.
            x = np.asarray(o3d_point_cloud.points)[i][0]
            y = np.asarray(o3d_point_cloud.points)[i][1]
            z = np.asarray(o3d_point_cloud.points)[i][2]

            # Add the point and its identifier to the new dictionary.
            if identifier not in objects:
                objects[identifier] = []
            objects[identifier].append(np.array([x, y, z], dtype=np.float64))

    return o3d_point_cloud, objects


def find_cluster_bounding_boxes(objects):
    """
    Using the objects dictionary from the DBSCAN method, find the bounding box of each cluster.

    Parameters:
    -----------
        objects{} - The dictionary from DBSCAN containing the detected clusters.

    Returns:
    --------
        open3d.geometry.AxisAlignedBoundingBox[] - An array containing multiple AxisAlignedBoundingBox types. One for each cluster.
    """
    # Create instance variables.
    bounding_boxes = []

    # Find the bounding box of each object.
    for key, object_points in objects.items():
        # Calculate bounding box.
        points = o3d.utility.Vector3dVector(object_points)
        bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(points)
        # Add to array.
        bounding_boxes.append(bounding_box)

    return bounding_boxes


def find_closest_bounding_box(bounding_boxes):
    """
    Using the array of bounding box objects calculate the x,y,z center point of each box, and then
    find the one closest to the origin.

    Parameters:
    -----------
        bounding_boxes[] - The list of open3d.geometry.AxisAlignedBoundingBox objects.

    Returns:
    --------
        o3d.geometry.AxisAlignedBoundingBox - The bounding box of the closest object.
        double - The distance in zed_handler.coordinate_units that the object is away from the origin.
    """
    # Loop through each of the bounding boxes and find the one closest to the camera.
    min_distance = math.inf
    bounding_box = None
    for object in bounding_boxes:
        # Get bounding boxes center point.
        object_location = object.get_center()
        # Find distance from origin.
        distance = math.sqrt(
            object_location[0] * object_location[0]
            + object_location[1] * object_location[1]
            + object_location[2] * object_location[2]
        )
        # Determine if this is the closer object.
        if distance < min_distance:
            min_distance = distance
            bounding_box = object

    return bounding_box, min_distance


def find_angle_from_camera_center(center_point):
    """
    Use some simple trig to find the angle of the objects center point from the center of the
    camera.

    Parameters:
    -----------
        [x, y, z] - The center point of the object.

    Returns:
    --------
        angle - The angle of the object from center in degrees.
    """
    # Find the distance of the object from the camera.
    distance = math.sqrt(
        center_point[0] * center_point[0] + center_point[1] * center_point[1] + center_point[2] * center_point[2]
    )
    # Find the angle of object from camera center line.
    angle = math.degrees(math.acos(center_point[2] / distance))

    return angle


def rotation_matrix_to_euler_angles(rotation_matrix):
    """
    This function converts a 3x3 rotation matrix into three euler angles.

    Parameters:
    -----------
        rotation_matrix - The 3x3 numpy array containing the rotation matrix values.

    Returns:
    --------
        euler_angles - The 1x3 numpy array containing x, y, z angles in the same units as the rotation matrix.
    """
    # Create instance variables.
    is_valid_matrix = False
    euler_angles = [0, 0, 0]

    # Check to see if the rotation matrix is funny.
    Rt = np.transpose(rotation_matrix)
    should_be_identity = np.dot(Rt, rotation_matrix)
    i = np.identity(3, dtype=rotation_matrix.dtype)
    n = np.linalg.norm(i - should_be_identity)
    is_valid_matrix = n < 1e-6

    # Only do calculations if matrix seems correct.
    if is_valid_matrix:
        # Check to see how many values the matrix contains.
        sy = math.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            y = math.atan2(-rotation_matrix[2, 0], sy)
            z = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            euler_angles = [x, y, z]
        else:
            x = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y = math.atan2(-rotation_matrix[2, 0], sy)
            z = 0
            euler_angles = [x, y, z]

    return euler_angles


def open_point_cloud_visualizer(o3d_point_cloud, *args):
    """
    This method provides an easy way to visualize a Open3D point cloud.

    IMPORTANT: This method will cause the program to halt duing the time the visualizer is open.

    Parameters:
    -----------
        o3d_point_cloud - The Open3D point cloud object to view.
        *args - As many other point clouds as you want.

    Returns:
    --------
        Nothing
    """
    o3d.visualization.draw_geometries([o3d_point_cloud, *args])


def convert_cloud_proc(data_queue, point_cloud):
    """
    This method is ran in parallel in a different process.
    It converts and then downsamples the zed point cloud.

    Parameters:
    -----------
        data_queue - The queue object used to store and pass converted point
                    cloud data back to the main thread.
        point_cloud - The zed point cloud to be converted.

    Returns:
    --------
        Nothing - (Puts result into the shared queue accessable by main thread.)
    """
    # Catch any potential errors.
    try:
        # Convert point cloud.
        pcd = convert_zed_cloud_to_o3d_cloud(point_cloud)
        # Downsample the point cloud to improve processing speed.
        pcd = down_sample_o3d_cloud(pcd, down_sample_factor=4)
        # Make pcd object picklable. (I really hate python, this is awful.)
        pcd = pickle_serialize(pcd)

        # Store data in queue.
        data_queue.put(pcd)
    except Exception as e:
        print("Convert Process Exception:", e)


def detect_object_clusters_proc(data_queue, point_cloud):
    """
    This method is ran in parallel in a different process.
    It detects the floor plane, finds objects clusters, and then calculates bounding boxes.

    Parameters:
    -----------
        data_queue - The queue object used to store and pass converted point
                    cloud data back to the main thread.
        point_cloud - The o3d point cloud to be processed.
    Returns:
    --------
        Nothing - (Puts result into the shared queue accessable by main thread.)
    """
    # Catch any potential errors.
    try:
        # Deserialize data.
        point_cloud = pickle_deserialize(point_cloud)
        # Use the point cloud to detect and remove the floor plane.
        outlier_cloud, inliers = RANSAC_plane_detection(
            point_cloud, num_of_planes=1, distance_threshold=80, ransac_n=3, num_iterations=300
        )
        # Make the data picklable.
        inlier_clouds = []
        for cloud in inliers:
            inlier_clouds.append(pickle_serialize(cloud))

        # Use the outlier points to detect objects.
        pcd, objects = DBSCAN_euclidean_clustering(
            outlier_cloud, growing_radius=120, min_points=80, print_progress=False
        )
        # Make the data picklable.
        pcd = pickle_serialize(pcd)

        # Find the bounding boxes of each object.
        boxes = find_cluster_bounding_boxes(objects)
        # Make the data picklable.
        bounding_boxes = []
        for box in boxes:
            bounding_boxes.append(np.asarray(box.get_box_points()))

        # Store data in queue.
        data_queue.put([pcd, bounding_boxes, inlier_clouds])
    except Exception as e:
        print("Detect Process Exception:", e)


class ObstacleDetector:
    def __init__(self):
        # Declare point clouds.
        self.processed_pcd = o3d.geometry.PointCloud()
        self.object_bounding_boxes = []
        self.inlier_clouds = []
        self.zed_point_cloud = None
        # Create queue for AsyncResult objects returned by process pool.
        self.conversion_process_queue = deque()
        self.detection_process_queue = deque()
        # Create queues for results.
        self.conversion_data_queue = Queue()
        self.detection_data_queue = Queue()

        # Declare other instance variables.
        self.conversion_exceptions = 0
        self.detection_exceptions = 0

    def detect_obstacle(self, zed_point_cloud, conversion_procs=3, detection_procs=3, multiproc_mode=True):
        """
        Detects obstacles in the provided point cloud. This method should be called
        at every iteration in the main loop as this methods starts multiple processes
        for speedy detection.

        Parameters:
        -----------
            zed_point_cloud - The point cloud object from the zed cam.
            conversion_procs - The number of processes to spawn for point cloud conversion operations.
            detection_procs - The number of processes to spawn for detection operations.
            multiproc_mode - Whether or not this method should use multiprocessing for object detection.
                            If false, then conversion_procs and detection_procs are meaningless.

        Returns:
        --------
            processed_pcd - Point cloud with detected planes removed and detected objects randomly colored.
            inlier_clouds - An array containing the point clouds of the deteted planes from RANSAC.
            object_bounding_boxes - An array of Open3D bounding box objects of each detected object.
        """
        # If there are converted clouds in queue, check if the next process is done calculating, then pull data from it.
        if len(self.conversion_process_queue) > 0:
            # Attempt to get data from queue, if process is not ready then skip and move on.
            try:
                # Get and unpack data from conversion data queue.
                point_cloud = pickle_deserialize(self.conversion_data_queue.get_nowait())
                # Remove dead process from process queue.
                process = self.conversion_process_queue.popleft()
                # If there are point_clouds in queue, check if the next process is done calculating, then pull data from it.
                if len(self.detection_process_queue) > 0:
                    # Attempt to get data from queue, if process is not ready then skip and move on.
                    try:
                        # Get and unpack data from detection data queue.
                        detection_data = self.detection_data_queue.get_nowait()
                        # Remove dead process from process queue.
                        process = self.detection_process_queue.popleft()
                        # Deserialize data.
                        self.processed_pcd = pickle_deserialize(detection_data[0])
                        boxes = detection_data[1]
                        self.object_bounding_boxes = []
                        for box in boxes:
                            self.object_bounding_boxes.append(
                                o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(box))
                            )
                        inliers = detection_data[2]
                        self.inlier_clouds = []
                        for cloud in inliers:
                            self.inlier_clouds.append(pickle_deserialize(cloud))
                    except Exception as e:
                        # Increment conversion exception counter.
                        self.detection_exceptions += 1
                        # Kill the first next up process in queue if it hangs.
                        if self.detection_exceptions > 2000:
                            # Remove and kill process.
                            for i in range(len(self.detection_process_queue)):
                                process = self.detection_process_queue.popleft()
                                process.terminate()
                            self.detection_process_queue.clear()
                            # Print info.
                            print("Detection process has hung...killing...", e)
                            self.detection_exceptions = 0
                # If the queue is not full, continue pulling new point clouds and starting new processes.
                if len(self.detection_process_queue) < detection_procs:
                    # To multiprocess or not to multiprocess.
                    if multiproc_mode:
                        # Create new process.
                        task = Process(
                            target=detect_object_clusters_proc,
                            args=(
                                self.detection_data_queue,
                                pickle_serialize(point_cloud),
                            ),
                        )
                        # Start new process.
                        task.start()
                    else:
                        task = DummyTask(
                            detect_object_clusters_proc(self.detection_data_queue, pickle_serialize(point_cloud))
                        )
                    self.detection_process_queue.append(task)
            except Exception as e:
                # Increment conversion exception counter.
                self.conversion_exceptions += 1
                # Kill the first next up process in queue if it hangs.
                if self.conversion_exceptions > 2000:
                    # Remove and kill process.
                    for i in range(len(self.conversion_process_queue)):
                        process = self.conversion_process_queue.popleft()
                        process.terminate()
                    self.conversion_process_queue.clear()
                    # Print info.
                    print("Conversion process has hung...killing...", e)
                    self.conversion_exceptions = 0
        # If the queue is not full, continue pulling new point clouds and starting new processes.
        if len(self.conversion_process_queue) < conversion_procs:
            # Get point cloud numpy array. (Must do it this way because zed objects are not picklable)
            self.zed_point_cloud = zed_point_cloud.get_data()
            # Check if zed points are empty.
            if len(self.zed_point_cloud) > 0:
                # To multiprocess or not to multiprocess.
                if multiproc_mode:
                    # Create new process.
                    task = Process(
                        target=convert_cloud_proc,
                        args=(
                            self.conversion_data_queue,
                            self.zed_point_cloud,
                        ),
                    )
                    # Start new process.
                    task.start()
                else:
                    task = DummyTask(convert_cloud_proc(self.conversion_data_queue, self.zed_point_cloud))
                self.conversion_process_queue.append(task)

        return self.processed_pcd, self.inlier_clouds, self.object_bounding_boxes

    def track_obstacle(self, object_bounding_boxes, reg_img=None, annotate=True, remove_floor=False):
        """
        Determine the object of interest from a list of objects.

        Parameters:
        -----------
            object_bounding_boxes - The list of Open3D bounding box objects from the detection method.
            reg_img - The normal colored image from the zed cam.
            annotate - Whether or not to display object bounding box and info on the image.
            remove_floor - Whether or not the detected floor plane should be removed from the image.

        Returns:
        --------
            angle - The angle in degrees of the object from the center line of the camera.
            object_distance - The straight-line distance of the object from the camera.
            closet_box - The bounding box of the object of interest.
        """
        # Declare instance variables.
        angle = 0

        # Find the distance of the closest object.
        closest_box, object_distance = find_closest_bounding_box(object_bounding_boxes)
        # Calculate the angle of the closest object.
        if closest_box is not None:
            angle = find_angle_from_camera_center(closest_box.get_center())

        return angle, object_distance, closest_box
