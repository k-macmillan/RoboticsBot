import hsv
import yellowcard as yc

# https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html

class LineFollow():
    def __init__(self):
        self.hsv_mask = hsv.HSV()
        self.stop_mask = yc.YellowCard()
        self.offset = 0.0

    def UpdateAll(self, frame):
        self.hsv_mask.SetHSV(frame)
        self.hsv_mask.SetMask(self.hsv_mask.hsv)
        self.stop_mask.SetMask(self.hsv_mask.hsv)


    def _FindCenter(self, frame):
        # print('Code that, you monkey')
        self.offset = 0.0


    def FindOffset(self, frame):
        self.hsv_mask.blackout(frame)
        self._FindCenter(frame)
        return self.offset

