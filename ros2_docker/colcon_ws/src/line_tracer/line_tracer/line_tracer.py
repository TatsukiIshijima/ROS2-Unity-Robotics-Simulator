import copy
import cv2
import numpy as np

class Singleton(object):
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(Singleton, cls).__new__(cls)
        return cls.instance

class LineTracer(Singleton):

    def __init__(self):
        self._frame = None
        self._threshold = 50
        self._kernel_size = 3

    def _binarize(self, src):
        gray = cv2.cvtColor(src=src, code=cv2.COLOR_BGR2GRAY)
        _, binary_img = cv2.threshold(src=gray, thresh=self._threshold, maxval=255, type=cv2.THRESH_BINARY_INV)
        return binary_img

    def _filter_noise(self, src):
        filtered_img = cv2.medianBlur(src=src, ksize=self._kernel_size)
        return filtered_img

    def _find_max_contour(self, src):
        max_area = 0
        max_contour_index = -1
        contours, hierarchy = cv2.findContours(image=src, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour_index = i

        if max_contour_index == -1:
            return None

        max_contours = contours[max_contour_index]
        return max_contours

    def _find_rect(self, contour):
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        return np.intp(box)

    def _get_rect_center(self, rect):
        rect_min_x = min(rect, key=lambda x: x[0])[0]
        rect_max_x = max(rect, key=lambda x: x[0])[0]
        rect_min_y = min(rect, key=lambda x: x[1])[1]
        rect_max_y = max(rect, key=lambda x: x[1])[1]
        rect_cx = int((rect_min_x + rect_max_x) / 2)
        rect_cy = int((rect_min_y + rect_max_y) / 2)
        if rect_cx < -1 or rect_cy < -1:
            return None, None
        return rect_cx, rect_cy

    def _get_moment_center(self, contour):
        moment = cv2.moments(contour)
        if moment is None:
            return None, None
        moment_cx = int(moment['m10'] / moment['m00'])
        moment_cy = int(moment['m01'] / moment['m00'])
        return moment_cx, moment_cy

    def process(self, capture_img):
        if capture_img is None:
            return

        self._frame = capture_img
        binary_img = self._binarize(src=self._frame)
        filtered_img = self._filter_noise(src=binary_img)
        max_contour = self._find_max_contour(src=filtered_img)

        if max_contour is None:
            return self._frame, None, None

        cv2.drawContours(image=self._frame, contours=[max_contour], contourIdx=-1, color=(0, 255, 0), thickness=3)

        rect = self._find_rect(max_contour)
        cv2.drawContours(image=self._frame, contours=[rect], contourIdx=-1, color=(255, 0, 0), thickness=3)
        rect_cx, rect_cy = self._get_rect_center(rect)
        if rect_cx is not None and rect_cy is not None:
            cv2.circle(img=self._frame, center=(rect_cx, rect_cy), radius=5, color=(0, 0, 255), thickness=-1)

        moment_cx, moment_cy = self._get_moment_center(max_contour)
        if moment_cx is not None and moment_cy is not None:
            cv2.circle(img=self._frame, center=(moment_cx, moment_cy), radius=5, color=(0, 255, 255), thickness=-1)

        return self._frame, moment_cx, moment_cy


if __name__ == '__main__':
    line_tracer = LineTracer()
    # input image
    frame = cv2.imread('frame_0.jpg')
    input_img = copy.deepcopy(frame)
    output_img, _, _ = line_tracer.process(input_img)
    cv2.imshow('Output', output_img)
    cv2.waitKey(0)
