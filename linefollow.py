import hsv

# https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html

class LineFollow():
    def __init__(self):
        self.hsv_mask = hsv.HSV()
        self.offset = 0.0


    def _FindCenter(self, frame):
        # print('Code that, you monkey')
        self.offset = 0.0


    def FindOffset(self, frame):
        self.hsv_mask.blackout(frame)
        self._FindCenter(frame)
        return self.offset

