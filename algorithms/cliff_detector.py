import numpy as np
import math


def cliff_index(c_matrix, row_index, row_interval, col_index, col_interval, avg_amount):
    """Averages a certain number of the top values in the specified section of the matrix.
       The certain number is specified with avg_amount.
       This is used to determine how cliff-like a certain section of the matrix is"""
    sub_matrix = c_matrix[row_index:row_index + row_interval,
                 col_index:col_index + col_interval].flatten()
    return np.average(np.sort(sub_matrix)[-avg_amount:])


def perform_convolution(c_matrix, image_matrix, use_abs=False):
    """performs a convolution with the c_matrix as weights, returning the resulting matrix.
    The convolution can return the absolute value of the convolution using the use_abs parameter"""

    col_offset = len(c_matrix[0]) - 1  # the matrix will run off the right side of the image without this
    row_offset = len(c_matrix) - 1  # the matrix will run off the bottom of the image without this

    conv_matrix = []
    # iterate across the matrix by intervals of how many columns the cliff matrix has
    for row_num in range(0, len(image_matrix) - row_offset, 2):
        conv_matrix.append([])  # add a new row each time
        # iterate down the matrix by intervals of how many rows the matrix has
        for col_num in range(0, len(image_matrix[0]) - col_offset, 2):
            # add the dot product of the matrix to the end of each row
            dp = calculate_dot_product(image_matrix, (row_num, col_num), c_matrix)
            if use_abs:
                dp = math.fabs(dp)
            conv_matrix[-1].append(dp)

        conv_matrix[-1] = np.asarray(conv_matrix[-1])
    conv_matrix = np.asarray(conv_matrix)
    print('conv shape is', conv_matrix.shape)
    print('conv min is', np.min(conv_matrix), 'conv max is', np.max(conv_matrix))
    return conv_matrix


def calculate_dot_product(image_matrix, image_location, con_matrix):
    """
    calculate the dot product from a given position in the image and a given matrix
    """
    # find the smaller sub-matrix within image_matrix that starts at image_location and has
    # the same dimensions as the matrix passed in
    image_sub_matrix = image_matrix[image_location[0]:image_location[0] + len(con_matrix),
                       image_location[1]:image_location[1] + len(con_matrix[0])].flatten()
    # return the dot product of the two flattened matrices
    return np.dot(image_sub_matrix, con_matrix.flatten())


def search_for_cliffs(image_matrix, rows=32, cols=32, percentile=-1, depth=1.2) -> np.array:
    """
    Uses an image matrix to look for cliffs, using a convolution of the image.
    Rows and cols refer to the dimensions of the output array.
    If percentile is not the default of -1, every pixel above the (percentile parameter) percentile will be 1
    and all other pixels will be 0.
    If percentile is the default of -1, every pixel above the depth parameter is a 1 and all others are 0
    """

    # this convolution matrix will have large positive values at the bottom of large horizontal differences and
    # negative values of large magnitude at the top of large horizontal differences
    h_mat_length = int(len(image_matrix[0]) / cols)
    h_mat = np.array([[1 for i in range(h_mat_length)],
                      [-1 for i in range(h_mat_length)]])

    h_conv_mat = perform_convolution(h_mat, image_matrix)

    # this convolutional matrix will have large values with vertical differences
    v_mat_length = int(len(image_matrix) / rows)
    v_mat = np.array([[-1, 1] for i in range(v_mat_length)])

    v_conv_mat = perform_convolution(v_mat, image_matrix, use_abs=True)

    # find the average of the top values for each of the sections the matrix is divided up into
    max_matrix = []
    # vertical convolution has fewer rows
    row_interval = (len(v_conv_mat) // rows)
    # horizontal convolution has fewer columns
    col_interval = (len(h_conv_mat[0]) // cols)
    for row in range(rows):
        row_index = row * row_interval
        max_matrix.append([])
        for col in range(cols):
            # find the relevant matrix values for each of the convolutions, find the average of the highest values,
            # and calculate how cliff-like it is by sqrt(horizontal^2 + vertical^2)
            col_index = col * col_interval
            amount_to_average = min(h_mat_length, v_mat_length)

            h_cliff_index = cliff_index(h_conv_mat, row_index=row_index, row_interval=row_interval,
                                        col_index=col_index, col_interval=col_interval, avg_amount=amount_to_average)
            v_cliff_index = cliff_index(v_conv_mat, row_index=row_index, row_interval=row_interval,
                                        col_index=col_index, col_interval=col_interval, avg_amount=amount_to_average)
            cliff_value = math.sqrt(h_cliff_index**2 + v_cliff_index**2)
            max_matrix[-1].append(cliff_value)

        max_matrix[-1] = np.asarray(max_matrix[-1])
    max_matrix = np.asarray(max_matrix)

    if percentile != -1:
        fire_value = np.percentile(max_matrix, percentile)
    else:
        fire_value = depth

    for row_num in range(len(max_matrix)):
        max_matrix[row_num] = [1 if e > fire_value else 0 for e in max_matrix[row_num]]

    return max_matrix


def main():
    # for testing, load an image from a picture and show the result
    '''
    from PIL import Image
    import matplotlib.pyplot as plt
    image_array = np.asarray(Image.open("resources/10_13_0_Depth.png"))
    result = search_for_cliffs(image_array)
    plt.imshow(result)
    '''


if __name__ == '__main__':
    main()

