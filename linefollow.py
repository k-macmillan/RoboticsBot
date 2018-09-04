import hsv
import yellowcard as yc

# https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html

class LineFollow():
    def __init__(self, width, height):
        self.hsv_mask = hsv.HSV()
        self.stop_mask = yc.YellowCard()
        self.road_slice = 0.0
        self.offset = 0.0
        self.slice_h = 25
        self.slice_w = width
        self.f_height = height

    def UpdateAll(self, frame):
        self.hsv_mask.SetHSV(frame)
        self.hsv_mask.SetMask(self.hsv_mask.hsv)
        self.stop_mask.SetMask(self.hsv_mask.hsv)
        self._FindCenter()


    def _FindCenter(self):
        # print('Code that, you monkey')
        self.road_slice = self.hsv_mask.mask[self.f_height - self.slice_h:self.f_height, 0:self.slice_w]
        self.offset = 0.0


    def FindOffset(self, frame):
        self.hsv_mask.blackout(frame)
        self._FindCenter(frame)
        return self.offset

