import cv2
import core
import numpy as np
import core

depth = None
depth_data = None


def detect_obstacle(depth_data):
    depth_matrix = depth_data.get_data()
    width = 1280
    height = 720

    maskDepth = np.zeros([height, width], np.uint8)
    maskDepth = np.where((depth_matrix < 1.5) & (depth_matrix >= 0), 1, 0).astype(
        np.uint8
    )

    contours, hierarchy = cv2.findContours(maskDepth, 2, 1)

    # Only proceed if at least one blob is found.
    if not contours:
        return []

    # Choose the largest blob.
    blob = max(contours, key=cv2.contourArea)

    if cv2.contourArea(blob) < core.MIN_OBSTACLE_PIXEL_AREA:
        return []

    return blob


def click_print_depth(event, x, y, flags, param):
    global depth, depth_data
    if event == cv2.EVENT_LBUTTONDOWN:
        print(depth[y][x])
        print(depth_data.get_data()[y][x])


def main():
    global depth, depth_data
    cv2.namedWindow("depth")
    cv2.setMouseCallback("depth", click_print_depth)
    cnt = []
    width, height = 1280, 720
    while True:
        depth = core.vision.camera_handler.grab_depth()
        reg = core.vision.camera_handler.grab_regular()

        depth_data = core.vision.camera_handler.grab_depth_data()
        cnt = detect_obstacle(depth_data)

        if cnt != []:
            depth = cv2.drawContours(depth, [cnt], 0, (0, 255, 0), 3)
            # Find center of contour and mark it on image
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            reg = cv2.drawContours(reg, [cnt], 0, (0, 255, 0), 3)
            cv2.putText(
                reg,
                "Obstacle Detected",
                (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
            cv2.circle(reg, (cX, cY), 7, (255, 255, 255), -1)

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        cv2.imshow("depth", depth)
        cv2.imshow("reg", reg)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            breakx


if __name__ == "__main__":
    main()