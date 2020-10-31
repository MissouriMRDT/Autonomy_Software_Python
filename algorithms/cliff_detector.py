import numpy as np


def search_for_cliffs(image_matrix) -> np.array:
    """
    use an image matrix to look for cliffs, using a convolution of the image
    """
    # this convolution matrix will have large positive values at the bottom of large horizontal differences and
    # negative values of large magnitude at the top of large horizontal differences
    cliff_matrix = np.array([[1, 1],
                             [-1, -1]])
    x_offset = len(cliff_matrix[0]) - 1  # the matrix will run off the right side of the image without this
    y_offset = len(cliff_matrix) - 1  # the matrix will run off the bottom of the image without this
    result_matrix = np.array([])
    # iterate across the matrix by intervals of how many columns the matrix has
    for x in range(0, len(image_matrix[0]) - x_offset, len(cliff_matrix[0])):
        result_matrix = np.append(result_matrix, [])  # add a new row each time
        # iterate down the matrix by intervals of how many rows the matrix has
        for y in range(0, len(image_matrix) - y_offset, len(cliff_matrix)):
            # add the dot product of the matrix to the end of each row
            result_matrix[x] = np.append(result_matrix[x], calculate_dot_product(image_matrix, (x, y), cliff_matrix))


    """
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
    """

    return result_matrix

def calculate_dot_product(image_matrix, image_location, con_matrix):
    """
    # calculate the dot product from a given position in the image and a given matrix
    """
    # find the smaller sub-matrix within image_matrix that starts at image_location and has
    # the same dimensions as the matrix passed in
    image_sub_matrix = image_matrix[image_location[0]:image_location[0] + len(con_matrix[0]),
                                    image_location[1]:image_location[1] + len(con_matrix)].flatten()
    # return the dot product of the two flattened matrices
    return np.dot(image_sub_matrix, con_matrix.flatten())