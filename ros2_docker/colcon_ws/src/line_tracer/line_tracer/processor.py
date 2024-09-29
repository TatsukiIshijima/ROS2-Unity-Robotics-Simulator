import numpy as np
import cv2


class Processor:
    THRESHOLD = 50

    def __init__(self):
        self.frame = None

    def __threshold(self):
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray, self.THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        return binary_image

    def __max_rect(self, contours):
        max_area = 0
        max_rect = None
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            if area > max_area:
                max_area = area
                max_rect = (x, y, w, h)
        return max_rect

    def process(self, frame):
        self.frame = frame
        binary_image = self.__threshold()
        self.frame = binary_image
        # masked_image = self.__mask(frame, binary_image)

        # contours = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # max_rect = self.__max_rect(contours)

        # if max_rect is None:
        #     return
        # x, y, w, h = max_rect
        # cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    def show(self):
        cv2.imshow('LineTracer', self.frame)
        cv2.waitKey(1)
