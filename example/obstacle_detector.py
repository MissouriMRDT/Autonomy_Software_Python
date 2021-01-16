import cv2
import core
import numpy as np


def detect_obstacle(depth_data):
    depth_matrix = depth_data.get_data()
    width = 1280
    height = 720

    maskDepth = np.zeros([height, width], np.uint8)
    maskDepth = np.where((depth_matrix < 1.5) & (depth_matrix >= 0), 1, 0).astype(np.uint8)
    # print(depth_matrix)
    # print(maskDepth)
    contours, hierarchy = cv2.findContours(maskDepth, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # print(contours)
    # Only proceed if at least one blob is found.
    if not contours:
        return []

    # Choose the largest blob.
    blob = max(contours, key=cv2.contourArea)
    return blob
    # print(blob)
    # Compute the minimum enclosing circle and centroid of the blob.
    # ((x, y), radius) = cv2.minEnclosingCircle(blob)
    # print((x, y))
    # print(radius)
    # targetRadius = int(radius)
    # return int(x), int(y), targetRadius


def main():
    while True:
        depth = core.vision.camera_handler.grab_depth()
        reg = core.vision.camera_handler.grab_regular()

        depth_data = core.vision.camera_handler.grab_depth_data()

        cnt = detect_obstacle(depth_data)
        color = (255, 0, 0)
        thickness = 2

        print(cnt.shape)
        print(cnt)
        depth = cv2.drawContours(depth, [cnt], 0, (0, 255, 0), 3)

        # depth = cv2.circle(depth, (x, y), radius, (0, 255, 0), 4)

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        cv2.imshow("depth", depth)
        cv2.imshow("reg", reg)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    main()