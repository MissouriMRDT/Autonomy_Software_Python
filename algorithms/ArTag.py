import numpy as np
import cv2
from ArTagEncoding import encode, HAMMINGCODE_MARKER_POSITIONS
MARKER_SIZE = 9

class HammingMarker(object):
    def __init__(self, id, contours=None):
        self.id = id
        self.contours = contours

    def __repr__(self):
        return '<Marker id = {} center = {} >'.format(self.id, self.center)

    @property
    def center(self):
        if self.contours is None:
            return None
        center_array = np.mean(self.contours, axis=0).flatten()
        return (int(center_array[0]), int(center_array[1]))

    def generate_image(self):
        img = np.zeros((MARKER_SIZE, MARKER_SIZE))
        #img[1, 1] = 255
        for count1, ar in enumerate(self.id):
            for count2, ar2 in enumerate(ar):
                img[count1+1][count2+1] = self.id[count1][count2]
        # return zoom(img, zoom=50, order=0)
        height,width = img.shape[:2]
        res = cv2.resize(img, (50*width, 50*height), interpolation = cv2.INTER_NEAREST)
        #cv2.imshow("test1", res)
        return res

    def draw_contour(self, img, color=(0,255,0), linewidth=5):
        cv2.drawContours(img, [self.contours], -1, color, linewidth)

    def highlite_marker(self, img, contour_color=(0,255,0), text_color=(255,0,0), linewidth=5, text_thickness=2):
        """
        This draws a bounding box around the marker on the image. NOTE: it returns
        a BGR image so the highlite is in color.

        Input:
          img: image with detected marker
          contour_color: bounding box color, default is Green (0,255,0)
          text_color: text color, default is Blue (255,0,0)
          linewidth: thickness of bonding box line
          text_thickness: thickness of marker number text

        Output:
          A color image with the marker drawn on it
        """
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        self.draw_contour(img, color=contour_color, linewidth=linewidth)
        cv2.putText(img, str(self.id), self.center, cv2.FONT_HERSHEY_SIMPLEX, text_thickness, text_color)
        return img

    @classmethod
    def generate(cls):
        #return HammingMarker(id=np.random.randint(4096))
        return HammingMarker(id=[[255,255,0,255,255],[255,255,0,255,255],[255,0,255,0,255],[255,255,255,255,255],[255,255,255,255,255]])
    @property
    def id_as_binary(self):
        return np.binary_repr(self.id, width=12)

    @property
    def hamming_code(self):
        return encode(self.id_as_binary)
