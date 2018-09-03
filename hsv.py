import cv2 as cv
import numpy as np

class HSV():
    def __init__(self):
        self.frame = 0
        self.mask = 0
        self.hsv = 0

    def SetHSV(self, frame):
        """ Convert BGR to HSV """
        self.hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)


    def SetMask(self, frame):

        # define range of black in HSV
        lower_black = np.array([0,0,0])
        upper_black = np.array([255,255,40])


        # Threshold the HSV image
        self.mask = 255 - cv.inRange(self.hsv, lower_black, upper_black)
