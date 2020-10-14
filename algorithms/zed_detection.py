# using  a zed 2 camera and the ZED SDK for obstacle detection
# need to import pyzed.sl as sl
import time
import numpy as np


class ZEDDetector:

    def __init__(self):
        self.working = True  # will be false if cannot open camera
        self.image = sl.Mat()  # will be a multi-dimensional array of the most updated camera image
        self.depth = sl.Mat()  # will be a multi-dimensional array representing image depth

        # create camera object
        self.zed = sl.camera

        # Create InitParameters object, set parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # hd720 video mode
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        # x increases to the right, y increases down, z increases the farther away you are
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_DOWN
        init_params.sdk_verbose = True

        # open camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.working = False
        else:
            self.runtime_parameters = sl.RuntimeParameters()
            self.runtime_parameters.sensing_mode = sl.SENSING_MODE_STANDARD  # options can be explored here
            # set depth confidence parameters
            self.runtime_parameters.confidence_threshold = 100
            self.runtime_parameters.textureness_confidence_threshold = 100

    def update_image(self, timeout=100) -> bool:
        """
        update the image, timeout is ms of how long to wait before giving up
        :param timeout:
        :return:
        """
        if not self.working:
            return

        start_time_ms = time.time() * 1000
        current_time_ms = start_time_ms
        while current_time_ms - start_time_ms < timeout:
            current_time_ms = time.time() * 1000
            # a new image is available if grab() returns success
            if self.zed.grab(self.runtime_parameters) == s1.ERROR_CODE_SUCCESS:
                # get the left image from the zed camera
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
                # get the depth map
                self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
                self.depth = self.depth.get_data()  # creates numpy array
                return True  # successfully updated the image and depth

        return False  # failed to update the image and depth before the timeout

    def search_for_cliffs(self) -> np.array:
        """
        use the current image to look for cliffs, using a convolution of the image
        """
        # this convolution matrix will have large positive values at the bottom of large horizontal differences and
        # negative values of large magnitude at the top of large horizontal differences
        cliff_matrix = np.array([[1, 1],
                                 [-1, -1]])
        x_offset = len(cliff_matrix[0]) - 1  # the matrix will run off the right side of the image without this
        y_offset = len(cliff_matrix) - 1  # the matrix will run off the bottom of the image without this
        result_matrix = np.array([])
        # iterate across the matrix by intervals of how many columns the matrix has
        for x in range(0, self.image.get_width() - x_offset, len(cliff_matrix[0])):
            result_matrix = np.append(result_matrix, [])  # add a new row each time
            # iterate down the matrix by intervals of how many rows the matrix has
            for y in range(0, self.image.get_height() - y_offset, len(cliff_matrix)):
                # add the dot product of the matrix to the end of each row
                result_matrix[x] = np.append(result_matrix[x], self.calculate_dot_product((x, y), cliff_matrix))

        distance_difference = 4  # how many meters should qualify as a cliff
        # find the min and max of each column, which represent the top and bottom of the cliff
        final_matrix = np.array([])  # will hold the midpoint, radius, and depth of every column with a cliff
        for column in result_matrix:
            cliff_top = np.argmin(column)
            cliff_bottom = np.argmax(column)
            if column[cliff_top] < -distance_difference and column[cliff_bottom] > distance_difference and \
                    cliff_bottom > cliff_top:  # bottom should be farther down than the top
                midpoint = (cliff_bottom + cliff_top) / 2
                radius = cliff_bottom - midpoint
                depth = (-column[cliff_top] + column[cliff_bottom]) / 2  # average difference from surroundings
                final_matrix = np.append(final_matrix, np.array([midpoint, radius, depth]))
            else:
                final_matrix = np.append(final_matrix, -1)  # append -1 to show no cliffs found

        # split the results into vertical sections, and for each section report where cliffs (if any) are
        vertical_sections = 6
        return_matrix = np.array([[] for i in range(vertical_sections)])
        columns_per_section = len(final_matrix) // vertical_sections
        for section in range(vertical_sections):
            midpoints = []
            radii = []
            depths = []
            for column in range(section * columns_per_section, (section + 1) * columns_per_section):
                if column != -1:
                    midpoints.append(column[0])
                    radii.append(column[1])
                    depths.append(column[2])
            if len(midpoints) != 0:
                return_matrix[section] = [np.average(midpoints), np.average(radii), np.average(depths)]
            else:
                return_matrix[section] = [-1, -1, -1]
        return return_matrix

    def calculate_dot_product(self, image_location, matrix):
        """
        # calculate the dot product from a given position in the image and a given matrix
        """
        # find the smaller sub-matrix within image_matrix that starts at image_location and has
        # the same dimensions as the matrix passed in
        image_sub_matrix = self.depth[image_location[0]:image_location[0] + len(matrix[0]),
                           image_location[1]:image_location[1] + len(matrix)].flatten()
        # return the dot product of the two flattened matrices
        return np.dot(image_sub_matrix, matrix.flatten())

    # method to call to get all features
    def get_features(self):
        pass

    def close(self):
        self.zed.close()
