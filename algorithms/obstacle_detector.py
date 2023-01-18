from asyncio import constants
import logging
import numpy as np
import torch
import pyzed.sl as sl
import torch.backends.cudnn as cudnn
from numpy.core.numeric import NaN
import platform
import core.constants
import core
import math
import os
import cv2
import multiprocessing as mp
import time

# Import yolov5 tools.
from core.vision.yolov5.models.common import DetectMultiBackend
from core.vision.yolov5.utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from core.vision.yolov5.utils.torch_utils import select_device
from core.vision.yolov5.utils.augmentations import letterbox
from core.vision.yolov5.utils.plots import Annotator, colors


def img_preprocess(img, device, half, net_size):
    """
    Prepares the image from the camera by reformatting it and converting the numpy array to a torch/tensor object
    depending on the current device selected.

    Parameters:
    -----------
        img - The numpy array containing the camera image.
        device - The device type the array should be optimized for. (CPU or NVIDIA CUDA)
        half - Boolean determining if all the numbers in the array are converted from 32-bit to 16-bit.
        net_size - Tuple containing the size of the image.

    Returns:
    --------
        img - The converted, optimized, and normalized image.
        ratio - Tuple containing the width and height ratios.
        pad - Tuple containing width and height padding for the image.

    """
    net_image, ratio, pad = letterbox(img[:, :, :3], net_size, auto=False)
    net_image = net_image.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    net_image = np.ascontiguousarray(net_image)

    img = torch.from_numpy(net_image).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0

    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    return img, ratio, pad


def xywh2abcd(xywh, im_shape):
    """
    Given the x and y center point, and the width and height of a rectangle. This method calculates the four
    corners of the rectangle in the image and returns those corners in a 2d array.

    Parameters:
    -----------
        xywh - 1d array containing the x, y, width, height of the rectangle in terms of image pixels.
        im_shape - 1d array containing the shape/resolution of the image.

    Returns:
    --------
        output - A 2d array containing the box points of the rectangle.
    """
    output = np.zeros((4, 2))

    # Center / Width / Height -> BBox corners coordinates
    x_min = (xywh[0] - 0.5 * xywh[2]) * im_shape[1]
    x_max = (xywh[0] + 0.5 * xywh[2]) * im_shape[1]
    y_min = (xywh[1] - 0.5 * xywh[3]) * im_shape[0]
    y_max = (xywh[1] + 0.5 * xywh[3]) * im_shape[0]

    # A ------ B
    # | Object |
    # D ------ C

    output[0][0] = x_min
    output[0][1] = y_min

    output[1][0] = x_max
    output[1][1] = y_min

    output[2][0] = x_min
    output[2][1] = y_max

    output[3][0] = x_max
    output[3][1] = y_max
    return output


def detections_to_custom_box(detections, img, reg_img):
    """
    Takes in the list of detections from the NMS function in yolov5 (converts tensor objects to something kinda readable/workable), and
    uses that info to find a 3-dimensional bounding box withing the zed cameras point cloud.

    :params detections: List of detections, on (n,6) tensor per image [xyxy, conf, cls]. Use the non_max_suppression method from yolov5's utils.general.
    :params img: The converted camera image from the img_preprocess function.
    :params reg_img: The normal camera image. (numpy array from zed cam.)

    :returns output: Array containing info about object.
    """
    output = []
    for i, det in enumerate(detections):
        if len(det):
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], reg_img.shape).round()
            gn = torch.tensor(reg_img.shape)[[1, 0, 1, 0]]  # normalization gain whwh

            for *xyxy, conf, cls in reversed(det):
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh

                # Creating ingestable objects for the ZED SDK
                obj = []
                obj.append(xywh2abcd(xywh, reg_img.shape))
                obj.append(cls)
                obj.append(conf)
                obj.append(False)
                output.append(obj)
    return output


def torch_proc(img_queue, result_queue, weights, img_size, classes=None, conf_thres=0.2, iou_thres=0.45):
    """
    This method runs in a seperate python process and uses shared queues to pass data back and forth with its
    parent process. This method opens the given weights file, loads the model, and then runs inference.

    :params img_queue: The ctx.Queue object used to give new images to the process.
    :params result_queue: The ctx.Queue object used to return inference data to the parent.
    :params weights: The file path containing the weights.pt file.
    :params img_size: The image size to run inference with.
    :params conf_thres: The minimum confidence threshold to consider something a good prediction.
    :params iou_thres: The intersection over union threshold to consider something a good prediction.

    :returns: Nothing (Everything is put in the result queue)
    """
    # Create instance variables.
    imgsz = (img_size, img_size)
    half = False

    print("Intializing Neural Network...")
    # Create the device and model objects. (load model)
    device = select_device()
    model = DetectMultiBackend(
        weights,
        device=device,
        dnn=False,
        data=os.path.dirname(__file__) + "/../core/vision/yolov5/data/coco128.yaml",
    )
    cudnn.benchmark = True

    # Load model
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Half
    half &= (pt or jit or engine) and device.type != "cpu"  # half precision only supported by PyTorch on CUDA
    if pt or jit:
        model.model.half() if half else model.model.float()

    # Warmup/Optimize
    model.warmup(imgsz=(1, 3, *imgsz))
    while True:
        # Get image from queue.
        image_net = img_queue.get()

        # Record start time.
        s = time.time()
        # Reformat and convert the image to something the model can read.
        img, ratio, pad = img_preprocess(image_net, device, half, imgsz)
        # Run inference on the image using the model.
        pred = model(img)
        # Get/filter the model detection results.
        # # Select classes that we want track. For corresponding object labels check the order of classes in your
        # .yaml file for your dataset.
        predictions = non_max_suppression(pred, conf_thres, iou_thres, classes)
        # ZED CustomBox format (with inverse letterboxing tf applied)
        detections = detections_to_custom_box(predictions, img, image_net)
        inference_time = time.time() - s

        # Put results into queue.
        result_queue.put([predictions, detections, names, inference_time])


class YOLOObstacleDetector:
    def __init__(self, weights, model_image_size, min_confidence, classes=None):
        """
        Initializing the Obstacle Detector class variables and objects.

        :params weights: The path to the yolo.pt weights file exported from your data using the YOLOv5 repo.
        :params model_image_size: The image size the model was trained on.
        :params min_confidence: The minimum confidence to consider something a detected object.
        :params classes: The YOLO classes to enable for detection. These classes are indexes for the user-defined class names in the yolo YAML file. The default of NONE will detect all classes.
        """
        # Create objects and variables.
        self.logger = logging.getLogger(__name__)
        self.sim_active = False
        self.objects = None
        self.obj_param = None
        self.names = None
        self.predictions = None
        self.object_summary = ""
        self.inference_time = 0

        # Setup process creation method.
        if platform.system() == "Linux":
            # Set method.
            self.ctx = mp.get_context("forkserver")
            # Print info.
            self.logger.info(f"Platform is {platform.system()}. Selecting forkserver process method.")
        else:
            # Set method.
            self.ctx = mp.get_context("spawn")
            # Print info.
            self.logger.info(f"Platform is {platform.system()}. Selecting spawn process method.")
        # Create queues for results.
        self.detection_image_queue = self.ctx.Queue(maxsize=2)
        self.detection_result_queue = self.ctx.Queue()

        # Setup zed.
        # Try to use zed, if fails assume we are using the sim.
        try:
            # Setup the zed positional tracking.
            core.vision.camera_handler.enable_pose_tracking()
            # Setup object detection on zed camera.
            self.obj_param = core.vision.camera_handler.enable_camera_detection_module(enable_tracking=True)
        except Exception:
            self.logger.info(
                "Unable to invoke zed specific methods. object_detector.py is assuming the SIM is active."
            )
            self.sim_active = True

        # Create seperate process for running inference.
        self.detect_task = self.ctx.Process(
            target=torch_proc,
            args=(
                self.detection_image_queue,
                self.detection_result_queue,
                weights,
                model_image_size,
                classes,
                min_confidence,
            ),
        )
        # Start the new thread.
        self.detect_task.start()

    def detect_obstacles(self, zed_left_img):
        """
        Uses the yolov5 algorithm and a pretrained model to detect potential obstacles. The detected objects in the
        2d image are then cross referenced with the 3d point cloud to get their real world position.


        :params zed_left_img: The image to perform inference on.

        :returns objects: A numpy array containing bounding_box_2d, label, probability, and is_grounded data.
        :returns predictions: List of predictions, on (n,6) tensor per image [xyxy, conf, cls].
        """
        # Pass new zed normal image to the process through the queue.
        if not self.detection_image_queue.full():
            self.detection_image_queue.put(zed_left_img.copy())

        # Attempt to get data from the queue, if process is not ready then skip iteration.
        try:
            # Get data from queue.
            detection_data = self.detection_result_queue.get_nowait()
            # Split data.
            self.predictions = detection_data[0]
            self.objects = detection_data[1]
            self.names = detection_data[2]
            self.inference_time = detection_data[3]
        except Exception as e:
            pass

        return self.objects, self.predictions

    def track_obstacle(self, zed_point_cloud, reg_img, label_img=True):
        """
        Tracks the closest object and display it on screen. All of the others objects are also labeled.

        :params zed_point_cloud: The point cloud array returned from the zed api.
        :params reg_img: Zed left eye camera image.
        :params label_img: Toggle for drawing inferences on screen.

        :returns angle: The angle of the obstacle in relation to the left ZED camera
        :returns distance: The distance of the center of the obstacle from the ZED
        :returns object_summary: A string containing the names and quantity of objects detected.
        :returns inference_time: The total amount of time it took for the neural network to complete inferencing.
        :returns object_locations: A list of all the detected objects distances and angles.
        """
        # Create instance variables.
        object_distance = -1
        object_angle = 0
        object_locations = []

        # Check if we have any predictions.
        if self.predictions is not None:
            # Loop though each prediction
            for i, det in enumerate(self.predictions):  # per image
                annotator = Annotator(reg_img, line_width=2, example=str(self.names))
                self.object_summary = ""
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    # det[:, :4] = scale_coords(image_net.shape[2:], det[:, :4], image_net.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        self.object_summary += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        if label_img:  # Add bbox to image
                            c = int(cls)  # integer class
                            label = f"{self.names[c]} {conf:.2f}"
                            annotator.box_label(xyxy, label, color=colors(c, True))

                # Write results overlay onto image if toggle is set.
                if label_img:
                    reg_img = annotator.result()

        # Check if we have any objects.
        if self.objects is not None:
            # Loop through the object array and get info about each object.
            closest_point = None
            for i, obj in enumerate(self.objects):
                # Use the bounding box info to get the center point of the object in the point cloud
                point = (
                    int((obj[0][3][1] - obj[0][0][1]) / 2 + obj[0][0][1]),
                    int((obj[0][1][0] - obj[0][0][0]) / 2 + obj[0][0][0]),
                )
                # Scale the object center in the screen to the point cloud res.
                depth_res_x, depth_res_y = core.vision.camera_handler.get_depth_res()
                img_res_x, img_res_y = core.vision.camera_handler.get_reg_res()
                scaled_point = (
                    int((point[0] * depth_res_y) / img_res_y),
                    int((point[1] * depth_res_x) / img_res_x),
                )

                # Grab depth data from the zed.
                depth = core.vision.camera_handler.grab_depth_data()
                # Get distance of the object from the camera. Add zed offset from robot center.
                current_distance = depth[scaled_point[0]][scaled_point[1]][0] + core.constants.ZED_Z_OFFSET

                # Calculate the angle of the object using camera params
                angle_per_pixel = core.vision.camera_handler.get_hfov() / img_res_x
                pixel_offset = point[1] - (img_res_x / 2)
                angle = pixel_offset * angle_per_pixel
                # If angle and distance make sense, then add the object info the the object location array.
                if (angle > -90 and angle < 90) and not (math.isnan(current_distance)):
                    object_locations.append((angle, current_distance / 1000))

                # Determine if current point is the closest point. ########## CHECK IF THIS ACTUALL ADDS ALL OBJECTS TO OBJECT_LOCATIONS LIST.
                if (object_distance == -1 and not current_distance < 0) or object_distance > current_distance:
                    # Store the closest distance, angle, and point in image.
                    object_distance = current_distance
                    object_angle = angle
                    closest_point = point

            # Draw circle of current tracked object.
            if closest_point is not None:
                reg_img = cv2.circle(
                    reg_img,
                    (closest_point[1], closest_point[0]),
                    radius=5,
                    color=(0, 0, 255),
                    thickness=-1,
                )

        # return angle, distance
        return object_angle, object_distance, self.object_summary, self.inference_time, object_locations