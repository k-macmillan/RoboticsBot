import cv2 as cv

class YellowCard():
    def __init__(self):
        self.card = False

    def IsCard(self, hsv):

        # define range of black in HSV
        lower_yellow = np.array([64, 213,54])
        upper_yellow = np.array([66,255,153])


        # Threshold the HSV image
        self.mask = 255 - cv.inRange(hsv, lower_yellow, upper_yellow)
        