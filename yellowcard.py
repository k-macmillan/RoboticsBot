import cv2 as cv
import numpy as np

class YellowCard():
    def __init__(self):
        self.card = False
        self.mask = 0

    def SetMask(self, hsv):

        # define range of yellow in HSV
        lower_yellow = np.array([23,86,120]) # 20 100 100
        upper_yellow = np.array([62,160,255]) # 30 255 255


        # Threshold the HSV image
        self.mask = 255 - cv.inRange(hsv, lower_yellow, upper_yellow)
        