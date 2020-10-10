
import cv2
import numpy as np
from ArTagEncoding import decode, extract_hamming_code
from ArTag import MARKER_SIZE, HammingMarker

BORDER_COORDINATES = [
    [0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7], [0, 8],
    [1, 0], [1, 1], [1, 2], [1, 3], [1, 4], [1, 5], [1, 6], [1, 7], [1, 8],
    [7, 0], [7, 1], [7, 2], [7, 3], [7, 4], [7, 5], [7, 6], [7, 7], [7, 8],
    [8, 0], [8, 1], [8, 2], [8, 3], [8, 4], [8, 5], [8, 6], [8, 7], [8, 8],
    [2, 0], [2, 1], [2, 7], [2, 8], [3, 0], [3, 1], [3, 7], [3, 8],
    [4, 0], [4, 1], [4, 7], [4, 8], [5, 0], [5, 1], [5, 7], [5, 8],
    [6, 0], [6, 1], [6, 7], [6, 8],
]

ORIENTATION_MARKER_COORDINATES = [[2, 2], [2, 6], [6, 2], [6, 6]]
"""BORDER_COORDINATES = [
    [0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [1, 0], [1, 6], [2, 0], [2, 6], [3, 0],
    [3, 6], [4, 0], [4, 6], [5, 0], [5, 6], [6, 0], [6, 1], [6, 2], [6, 3], [6, 4], [6, 5], [6, 6],
]

ORIENTATION_MARKER_COORDINATES = [[1, 1], [1, 5], [5, 1], [5, 5]]"""

def validate_and_turn(marker):
    for crd in BORDER_COORDINATES:
        if marker[crd[0], crd[1]] != 0.0:
            raise ValueError('Border contains not entirely black parts.')
    #print(marker[1,1])
    #if marker[1,1] == 0.0 or marker[1,2] == 0.0 or marker[1,4] == 0.0 or marker[1,5] == 0.0:
        #raise ValueError('No orientation marker found.')
    print(marker)
    if marker[2,2] == 0.0 or marker[2,3] == 0.0 or marker [2,5] == 0.0 or marker[2,6] == 0.0:
        raise ValueError('No orientation marker found.')
    """ orientation_marker = None
    for crd in ORIENTATION_MARKER_COORDINATES:
        marker_found = False
        if marker[crd[0], crd[1]] == 1.0:
            marker_found = True
        if marker_found and orientation_marker:
            raise ValueError('More than 1 orientation_marker found.')
        elif marker_found:
            orientation_marker = crd
    if not orientation_marker:
        raise ValueError('No orientation marker found.')
    rotation = 0
    if orientation_marker == [2, 6]:
        rotation = 1
    elif orientation_marker == [6, 6]:
        rotation = 2
    elif orientation_marker == [6, 2]:
        rotation = 3
    marker = np.rot90(marker, k=rotation) """
    return marker

def detect_markers(img):
    """
    This is the main function for detecting markers in an image.

    Input:
      img: a color or grayscale image that may or may not contain a marker.

    Output:
      a list of found markers. If no markers are found, then it is an empty list.
    """
    if len(img.shape) > 2:
        width, height, _ = img.shape
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        width, height = img.shape
        gray = img
    #cv2.imshow("frame", gray)
    edges = cv2.Canny(gray, 10, 100)
    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]
    cv2.imshow("frame",edges)
    min_contour_length = min(width, height) / 50
    contours = [contour for contour in contours if len(contour) > min_contour_length]
    warped_size = 81
    canonical_marker_coords = np.array(
        (
            (0, 0),
            (warped_size - 1, 0),
            (warped_size - 1, warped_size - 1),
            (0, warped_size - 1)
        ),
        dtype = 'float32')
    #print(canonical_marker_coords)
    markers_list = []
    for contour in contours:
        approx_curve = cv2.approxPolyDP(contour, len(contour) * 0.01, True)
        if not (len(approx_curve) == 4 and cv2.isContourConvex(approx_curve)):
            continue
        
        sorted_curve = np.array(
            cv2.convexHull(approx_curve, clockwise=False),
            dtype='float32'
        )
        #print(sorted_curve)
        persp_trans = cv2.getPerspectiveTransform(sorted_curve, canonical_marker_coords)
        
        warped_img = cv2.warpPerspective(img, persp_trans, (warped_size, warped_size))
        #cv2.imshow("frame", warped_img)
        if len(warped_img.shape) > 2:
            warped_gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        else:
            warped_gray = warped_img

        _, warped_bin = cv2.threshold(warped_gray, 200, 255, cv2.THRESH_BINARY)
        #warped_bin = cv2.adaptiveThreshold(warped_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0) # try adaptive thresholding if needed after this actually works
        #cv2.imshow("frame", warped_bin)
        #marker = cv2.resize(warped_bin, (9,9), interpolation=cv2.INTER_AREA)
        marker = warped_bin.reshape([MARKER_SIZE, warped_size // MARKER_SIZE, MARKER_SIZE, warped_size // MARKER_SIZE]
        )
        marker = marker.mean(axis=3).mean(axis=1)
        marker[marker < 127] = 0
        marker[marker >= 127] = 1
        #cv2.imshow("frame", marker)

        try:
            marker = validate_and_turn(marker)
            #hamming_code = extract_hamming_code(marker)
            #marker_id = int(decode(hamming_code), 2)
            markers_list.append(HammingMarker(id=1, contours=approx_curve))
        except ValueError:
            continue
    return markers_list
