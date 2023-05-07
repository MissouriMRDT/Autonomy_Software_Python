import numpy as np
import cv2
import copy
import imutils
import math


def identify_tags(img):
    # Returns contorus for tags found in image

    # findContours works with grayscale
    grayscale_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur_img = cv2.GaussianBlur(grayscale_img, (5, 5), 0)

    # Discover edges
    edges = cv2.Canny(blur_img, 75, 200)

    # Find enclosed shapes
    cnts, h = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Identify contour indexes that have children
    possible_tags_index = list()
    for contour_hierarchy in h[0]:
        if contour_hierarchy[3] != -1:
            possible_tags_index.append(contour_hierarchy[3])

    contours = list()
    # Identify contours that are square with non-square children
    for tag_index in possible_tags_index:

        # Epsilon setting adjusts "maximum distance from contour to approximated contour" 
        #   aka accuracy/acceptance level
        epsilon = 0.02 * cv2.arcLength(cnts[tag_index], True)

        approx = cv2.approxPolyDP(cnts[tag_index], epsilon, True)
        child_approx = cv2.approxPolyDP(cnts[h[0][tag_index][2]], epsilon, True)

        if cv2.contourArea(approx) > 300 and len(approx) == 4 and len(child_approx) > 4:
            contours.append(approx)

    return contours


def order(pts):
    # Returns map of points to transform against

    rect = np.zeros((4, 2), dtype="float32")

    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect


# Constants for tag retrieval
AR_DIM = 32
TAG_MATRIX = np.array([
    [0, 0],
    [AR_DIM - 1, 0],
    [AR_DIM - 1, AR_DIM - 1],
    [0, AR_DIM - 1]], dtype="float32")


def retrieve_tags(frame, contours):
    # Retrieves tag[] from image, perspective corrected

    transformed_tags = list()
    for contour in contours:
        c_rez = contour[:, 0]

        # Get Tag
        transformation_matrix = cv2.getPerspectiveTransform(order(c_rez), TAG_MATRIX)
        tag_img = cv2.warpPerspective(frame, transformation_matrix, (AR_DIM, AR_DIM))

        # Convert to Two-Tone, B&W
        grayImage = cv2.cvtColor(tag_img, cv2.COLOR_BGR2GRAY)
        _, blackAndWhiteImage = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)

        transformed_tags.append(blackAndWhiteImage)

    return transformed_tags


def main():
    # Local execution program

    cv2.namedWindow('Contours')
    cv2.namedWindow('AR Tag')

    cap = cv2.VideoCapture('../resources/one_tag.mp4')

    while cap.isOpened():
        success, img = cap.read()
        if not success:
            break

        contours = identify_tags(img)

        # Display contour identification
        img_contours = copy.copy(img)
        cv2.drawContours(img_contours, contours, -1, (0, 0, 255), 2)
        cv2.imshow('Contours', img_contours)

        tags = retrieve_tags(img, contours)

        for tag in tags:
            cv2.imshow('AR Tag', tag)

        cv2.waitKey(5)


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
